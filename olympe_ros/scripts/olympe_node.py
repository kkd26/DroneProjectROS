#!/usr/bin/env python3

import cv2
from std_msgs.msg import Header, Time
from sensor_msgs.msg import Image, NavSatFix, Range
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, Quaternion, Vector3, Twist
from std_srvs.srv import Trigger, TriggerResponse
import rospy
from cv_bridge import CvBridge
import olympe
import json
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

DRONE_IP = '10.202.0.1'

def handle_takeoff(req):
    # TODO: error handling
    return TriggerResponse(success=drone(TakeOff()).wait().success())

def handle_landing(req):
    return TriggerResponse(success=drone(Landing()).wait().success())

def handle_pcmd(data: Twist):
    global drone
    if drone is None or not drone._piloting:
        return
    # rospy.loginfo(data)
    pitch, roll, gaz = int(data.linear.x), int(data.linear.y), int(data.linear.z)
    yaw = int(data.angular.z)
    drone.piloting_pcmd(pitch=pitch, roll=roll, gaz=gaz, yaw=yaw, piloting_time=0.05)

def frame_cb(yuv_frame: olympe.VideoFrame):
    # global seq
    info = yuv_frame.info()
    if 'metadata' not in info:
        rospy.logwarn('metadata does not exist in info, skipping frame')
        return
    metadata = info['metadata']

    # rospy.loginfo(json.dumps(info))
    cv2_cvt_color_flag = {
        olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
        olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
    }[info['yuv']['format']]

    cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)

    drone_quat = metadata['drone_quat']
    location = metadata['location']
    ground_distance = metadata['ground_distance']
    speed = metadata['speed']

    frame = bridge.cv2_to_imgmsg(cv2frame, 'passthrough')
    header = Header()
    header.stamp = rospy.Time.now()

    frame.header = header
    camera_pub.publish(frame)
    quat_pub.publish(
        quaternion=Quaternion(
            drone_quat['x'], 
            drone_quat['y'], 
            drone_quat['z'], 
            drone_quat['w']),
        header=header)
    
    location_pub.publish(
        latitude=location['latitude'],
        longitude=location['longitude'],
        altitude=location['altitude'],
        header=header)
    
    speed_pub.publish(
        vector=Vector3(
            x=speed['east'],
            y=speed['north'],
            z=speed['down']),
        header=header)
    
    ground_distance_pub.publish(
        min_range=0,
        max_range=float('inf'),
        range=ground_distance,
        header=header)
    # rospy.loginfo('seq = {}'.format(seq))
    # seq += 1

if __name__ == '__main__':
    try:
        rospy.init_node('olympe')

        camera_pub = rospy.Publisher('camera', Image, queue_size=2)
        quat_pub = rospy.Publisher('quat', QuaternionStamped, queue_size=2)
        location_pub = rospy.Publisher('location', NavSatFix, queue_size=2)
        speed_pub = rospy.Publisher('speed', Vector3Stamped, queue_size=2)
        ground_distance_pub = rospy.Publisher('ground_distance', Range, queue_size=2)

        takeoff_srv = rospy.Service('takeoff', Trigger, handle_takeoff)
        landing_srv = rospy.Service('landing', Trigger, handle_landing)

        pcmd_sub = rospy.Subscriber('pcmd', Twist, handle_pcmd)
        
        bridge = CvBridge()
        # seq = 0

        drone = olympe.Drone(DRONE_IP)
        drone.connect()
        drone.set_streaming_callbacks(raw_cb=frame_cb)
        drone.start_video_streaming()
        drone.start_piloting()

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass
    finally:
        drone.stop_piloting()
        drone.stop_video_streaming()
        drone.disconnect()
