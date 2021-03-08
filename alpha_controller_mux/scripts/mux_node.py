#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Twist
import message_filters

last_ctrl = (0, 0.0) # (priority, time)
timeout = 2

def is_twist_zero(data: Twist):
    return data.linear.x == 0.0 and data.linear.y == 0.0 and data.linear.z == 0.0 and data.angular.x == 0.0 and data.angular.y == 0.0 and data.angular.z == 0.0

def handle_teleop_ctrl(cmd_vel: Twist, gimbal: Float64):
    global last_ctrl

    if is_twist_zero(cmd_vel) and gimbal.data == 0.0:
        return
    
    last_ctrl = (10, rospy.get_rostime().to_sec())

    pcmd_pub.publish(cmd_vel)
    gimbal_pub.publish(gimbal)
    ctrl_source_pub.publish('teleop')

def handle_autopilot_ctrl(x: Float64, y: Float64, z: Float64, yaw: Float64, gimbal: Float64):
    global last_ctrl
    stamp_secs = rospy.get_rostime().to_sec()
    last_priority, last_stamp_secs = last_ctrl

    if last_priority > 0 and stamp_secs - last_stamp_secs <= timeout:
        rospy.loginfo_throttle_identical(1, 'A source with a higher priority is controlling, skipping the current control request')
        return
    
    last_ctrl = (0, stamp_secs)

    cmd_vel = Twist()
    cmd_vel.linear.x = x.data
    cmd_vel.linear.y = y.data
    cmd_vel.linear.z = z.data
    cmd_vel.angular.z = yaw.data

    pcmd_pub.publish(cmd_vel)
    gimbal_pub.publish(gimbal)
    ctrl_source_pub.publish('pid_controller')

if __name__ == '__main__':
    try:
        rospy.init_node('alpha_controller_mux')
        pcmd_pub = rospy.Publisher('pcmd', Twist, queue_size=1)
        gimbal_pub = rospy.Publisher('gimbal', Float64, queue_size=1)
        ctrl_source_pub = rospy.Publisher('control_source', String, queue_size=1, latch=True)

        teleop_pcmd_sub = message_filters.Subscriber('teleop_pcmd', Twist)
        teleop_gimbal_sub = message_filters.Subscriber('teleop_gimbal', Float64)

        teleop_ctrl_sync = message_filters.ApproximateTimeSynchronizer([teleop_pcmd_sub, teleop_gimbal_sub], queue_size=3, slop=0.06, allow_headerless=True)
        teleop_ctrl_sync.registerCallback(handle_teleop_ctrl)

        autopilot_ctrl_x_sub = message_filters.Subscriber('autopilot_x', Float64)
        autopilot_ctrl_y_sub = message_filters.Subscriber('autopilot_y', Float64)
        autopilot_ctrl_z_sub = message_filters.Subscriber('autopilot_z', Float64)
        autopilot_ctrl_yaw_sub = message_filters.Subscriber('autopilot_yaw', Float64)
        autopilot_ctrl_gimbal_sub = message_filters.Subscriber('autopilot_gimbal', Float64)

        autopilot_ctrl_sync = message_filters.ApproximateTimeSynchronizer(
            [autopilot_ctrl_x_sub, autopilot_ctrl_y_sub, autopilot_ctrl_z_sub, autopilot_ctrl_yaw_sub, autopilot_ctrl_gimbal_sub],
            queue_size=10, slop=0.06, allow_headerless=True
        )
        autopilot_ctrl_sync.registerCallback(handle_autopilot_ctrl)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
