#!/usr/bin/env python3

import cv2
import rospy
import olympe
import utm
import math
import tf_conversions
import tf2_ros
import io

from std_msgs.msg import Float64, Empty, Int32, String
from sensor_msgs.msg import Image, NavSatFix, Range, NavSatStatus, CompressedImage
from geometry_msgs.msg import Vector3Stamped, Twist, Twist, TransformStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.gimbal import set_target
from olympe.messages.ardrone3.GPSSettingsState import HomeChanged
from olympe.messages.ardrone3.GPSSettings import HomeType

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})


class FlightListener(olympe.EventListener):
    def __init__(self, *contexts):
        self.last_pos = Vector3Stamped()
        super().__init__(*contexts)

    # this is an ugly workaround as we can't directly get the controller location without setting HomeType to pilot and listen to HomeChanged
    @olympe.listen_event(HomeChanged())
    def onControllerPositionChanged(self, event, scheduler):
        global pilot_location_pub
        location = event.args

        stamp = rospy.Time.now()

        gps = NavSatFix()
        gps.header.frame_id = param_map_frame_id
        gps.header.stamp = stamp
        gps.latitude = location['latitude']
        gps.longitude = location['longitude']
        gps.altitude = location['altitude']

        pilot_location_pub.publish(gps)

        odom = Odometry()        
        odom.header.stamp = stamp
        odom.header.frame_id = param_map_frame_id
        odom.child_frame_id = param_pilot_frame_id

        try:
            pilot_pos_x, pilot_pos_y, pilot_pos_zone_id, pilot_pos_zone_letter = utm.from_latlon(
                location['latitude'], location['longitude']
            )
        except utm.OutOfRangeError as e:
            rospy.logwarn('Bad pilot GPS location: {}'.format(e))
            return
        
        odom.pose.pose.position.x = pilot_pos_x
        odom.pose.pose.position.y = pilot_pos_y
        odom.pose.pose.position.z = location['altitude']

        odom.pose.pose.orientation.x = 1.0 # identity quat

        dt = stamp.to_sec() - self.last_pos.header.stamp.to_sec()

        odom.twist.twist.linear.x = (odom.pose.pose.position.x - self.last_pos.vector.x) / dt
        odom.twist.twist.linear.y = (odom.pose.pose.position.y - self.last_pos.vector.y) / dt
        odom.twist.twist.linear.z = (odom.pose.pose.position.z - self.last_pos.vector.z) / dt

        self.last_pos = Vector3Stamped()
        self.last_pos.header = odom.header
        self.last_pos.vector = odom.pose.pose.position

        pilot_odom_pub.publish(odom)

        tr = TransformStamped()
        tr.header.frame_id = param_map_frame_id
        tr.header.stamp = stamp
        tr.child_frame_id = param_pilot_frame_id
        tr.transform.translation = odom.pose.pose.position
        tr.transform.rotation = odom.pose.pose.orientation
        tf_broadcaster.sendTransform(tr)
        

def handle_takeoff(req):
    drone(TakeOff())

def handle_landing(req):
    drone(Landing())

def handle_pcmd(data: Twist):
    global drone
    try:
        if not drone._piloting:
            rospy.logwarn('Drone is not being piloted, ignoring pcmd')
            return
        pitch = int(data.linear.x)
        roll = -int(data.linear.y)
        gaz = int(data.linear.z)
        yaw = -int(data.angular.z)
        drone.piloting_pcmd(pitch=pitch, roll=roll, gaz=gaz, yaw=yaw, piloting_time=0.05)
    except NameError:
        rospy.logwarn('Drone is not initialized yet, ignoring pcmd')

def handle_gimbal(data: Float64):
    global drone
    try:
        drone(set_target(
            gimbal_id=0,
            control_mode='velocity', 
            pitch_frame_of_reference='absolute', 
            pitch=data.data,
            yaw_frame_of_reference='absolute',
            yaw=0.0,
            roll_frame_of_reference='absolute',
            roll=0.0))
    except Exception as e:
        rospy.logerr('Error setting gimbal angle: {}'.format(e))

def frame_cb(yuv_frame: olympe.VideoFrame):
    global odom_last_position
    try:
        info = yuv_frame.info()
        if 'metadata' not in info:
            rospy.logwarn('metadata does not exist in info, skipping frame')
            return
        metadata = info['metadata']

        # rospy.loginfo(json.dumps(info))

        drone_quat = metadata['drone_quat']
        location = metadata['location']
        ground_distance = metadata['ground_distance']
        speed = metadata['speed']
        air_speed = metadata['air_speed']
        battery_percentage = metadata['battery_percentage']
        state = metadata['state']

        stamp = rospy.Time.now()
        stamp_secs = stamp.to_sec()

        # --- IMAGE ---

        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info['yuv']['format']]

        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)
        # cv2frame = cv2.resize(cv2frame, (640, 360))
        frame = bridge.cv2_to_imgmsg(cv2frame, 'bgr8')
        frame.header.stamp = stamp
        camera_pub.publish(frame)

        # --- ODOMETRY ---

        odom = Odometry()
        odom.child_frame_id = 'base_link'
        odom.header.frame_id = param_map_frame_id
        odom.header.stamp = stamp
        odom_pose_roll, odom_pose_pitch, odom_pose_yaw = tf_conversions.transformations.euler_from_quaternion([drone_quat['x'], drone_quat['y'], drone_quat['z'], drone_quat['w']])

        odom_pose_pitch = -odom_pose_pitch
        odom_pose_yaw = -odom_pose_yaw + math.pi * 0.5

        odom_pose_quat_x, odom_pose_quat_y, odom_pose_quat_z, odom_pose_quat_w = tf_conversions.transformations.quaternion_from_euler(
            odom_pose_roll, odom_pose_pitch, odom_pose_yaw
        )

        odom.pose.pose.orientation.x = odom_pose_quat_x
        odom.pose.pose.orientation.y = odom_pose_quat_y
        odom.pose.pose.orientation.z = odom_pose_quat_z
        odom.pose.pose.orientation.w = odom_pose_quat_w

        world_velocity_x = speed['east']
        world_velocity_y = speed['north']
        world_velocity_z = -speed['down']

        odom_velocity_x = math.cos(odom_pose_yaw) * world_velocity_x + math.sin(odom_pose_yaw) * world_velocity_y
        odom_velocity_y = -math.sin(odom_pose_yaw) * world_velocity_x + math.cos(odom_pose_yaw) * world_velocity_y
        odom_velocity_z = world_velocity_z

        odom.twist.twist.linear.x = odom_velocity_x
        odom.twist.twist.linear.y = odom_velocity_y
        odom.twist.twist.linear.z = odom_velocity_z

        try:
            odom_pos_x, odom_pos_y, odom_pos_zone_id, odom_pos_zone_letter = utm.from_latlon(
                location['latitude'], location['longitude']
            )
            odom.pose.pose.position.x = odom_pos_x
            odom.pose.pose.position.y = odom_pos_y
            odom.pose.pose.position.z = location['altitude']
        except KeyError:
            rospy.logwarn('GPS is not fixed')
        
        odom_pub.publish(odom)

        tr = TransformStamped()
        tr.header.stamp = stamp
        tr.header.frame_id = param_map_frame_id
        tr.child_frame_id = 'base_link'
        tr.transform.translation = odom.pose.pose.position
        tr.transform.rotation = odom.pose.pose.orientation
        tf_broadcaster.sendTransform(tr)

        # base_link without roll or pitch
        tr.child_frame_id = 'base_link_norp'
        tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w = tf_conversions.transformations.quaternion_from_euler(0, 0, odom_pose_yaw)
        
        tf_broadcaster.sendTransform(tr)

        # --- LOCATION ---

        location_msg = NavSatFix()
        location_msg.header.stamp = stamp
        try:
            location_msg.status.status = NavSatStatus.STATUS_FIX
            location_msg.latitude = location['latitude']
            location_msg.longitude = location['longitude']
            location_msg.altitude = location['altitude']
        except KeyError:
            location_msg.status.status = NavSatStatus.STATUS_NO_FIX
        location_pub.publish(location_msg)

        # --- GROUND DISTANCE ---

        ground_distance_msg = Range()
        ground_distance_msg.header.stamp = stamp
        ground_distance_msg.min_range = 0.0
        ground_distance_msg.max_range = float('inf')
        ground_distance_msg.range = ground_distance
        ground_distance_pub.publish(ground_distance_msg)

        # --- AIR SPEED ---
        air_speed_pub.publish(data=air_speed)

        # --- BATTERY PERCENTAGE ---
        battery_percentage_pub.publish(data=battery_percentage)

        # --- STATE ---
        state_pub.publish(data=state)
        
        last_stamp_secs = stamp.to_sec()
        
    except KeyError as e:
        rospy.logerr(e)
        rospy.logerr(info)

def frame_compressed_cb(h264_frame: olympe.VideoFrame):
    header_io = io.BytesIO()
    h264_frame._stream['h264_header'].tofile(header_io)
    header = header_io.getvalue()
    
    h264_header_msg = CompressedImage()
    h264_header_msg.format = 'h264_header'
    h264_header_msg.header.stamp = rospy.Time.now()
    h264_header_msg.data = header
    camera_h264_header_pub.publish(h264_header_msg)

    h264_frame_msg = CompressedImage()
    h264_frame_msg.format = 'h264'
    h264_frame_msg.header.stamp = rospy.Time.now()
    h264_frame_msg.data = h264_frame.as_ndarray().tostring()
    camera_h264_pub.publish(h264_frame_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('alpha_olympe_bridge')

        param_map_frame_id = rospy.get_param('map_frame_id', 'map')
        param_pilot_frame_id = rospy.get_param('pilot_frame_id', 'pilot_raw')

        camera_pub = rospy.Publisher('camera/image_raw', Image, queue_size=2)
        camera_h264_pub = rospy.Publisher('camera/image_h264', CompressedImage, queue_size=2, latch=True)
        camera_h264_header_pub = rospy.Publisher('camera/image_h264_header', CompressedImage, queue_size=1, latch=True)

        location_pub = rospy.Publisher('gps', NavSatFix, queue_size=2, latch=True)
        odom_pub = rospy.Publisher('odom', Odometry, queue_size=2, latch=True)
        pilot_location_pub = rospy.Publisher('pilot/gps', NavSatFix, queue_size=2, latch=True)
        pilot_odom_pub = rospy.Publisher('pilot/odom_raw', Odometry, queue_size=2, latch=True)
        ground_distance_pub = rospy.Publisher('ground_distance', Range, queue_size=2, latch=True)
        air_speed_pub = rospy.Publisher('air_speed', Float64, queue_size=2, latch=True)
        battery_percentage_pub = rospy.Publisher('battery_percentage', Int32, queue_size=2, latch=True)
        state_pub = rospy.Publisher('state', String, queue_size=2, latch=True)
        tf_broadcaster = tf2_ros.TransformBroadcaster()

        takeoff_sub = rospy.Subscriber('takeoff', Empty, handle_takeoff)
        landing_sub = rospy.Subscriber('landing', Empty, handle_landing)
        pcmd_sub = rospy.Subscriber('pcmd', Twist, handle_pcmd)
        gimbal_sub = rospy.Subscriber('gimbal', Float64, handle_gimbal)
        
        bridge = CvBridge()

        drone_host = rospy.get_param('~drone_host', '10.202.0.1')

        drone = olympe.Drone(drone_host)

        with FlightListener(drone):
            drone.connect()
            drone.set_streaming_callbacks(raw_cb=frame_cb, h264_cb=frame_compressed_cb)
            drone.start_video_streaming()
            drone.start_piloting()
            drone.set_streaming_output_files()

            # this is an ugly workaround as we can't directly get the controller location without setting HomeType to pilot and listen to HomeChanged
            drone(HomeType('PILOT')).wait()

            rospy.spin()

    except rospy.ROSInterruptException:
        pass
