#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Twist, Vector3

last_ctrl = (0, 0.0) # (priority, time)
timeout = 2

def twist_is_zero(data: Twist):
    return data.linear.x == 0.0 and data.linear.y == 0.0 and data.linear.z == 0.0 and data.angular.x == 0.0 and data.angular.y == 0.0 and data.angular.z == 0.0

def handle_teleop_pcmd(data: Twist):
    global last_ctrl, pcmd_pub, ctrl_source_pub

    if twist_is_zero(data):
        return
    
    last_ctrl = (10, rospy.get_rostime().to_sec())
    pcmd_pub.publish(data)
    ctrl_source_pub.publish('teleop')

def handle_teleop_gimbal(data: Float64):
    global last_ctrl, gimbal_pub

    if data.data == 0.0:
        return

    last_ctrl = (10, rospy.get_rostime().to_sec())
    gimbal_pub.publish(data)
    ctrl_source_pub.publish('teleop')

def handle_yaw_ctrl_yaw(data: Float64):
    global last_ctrl, pcmd_pub, ctrl_source_pub
    ctime = rospy.get_rostime().to_sec()
    p, t = last_ctrl
    if p > 0 and ctime - t <= timeout:
        rospy.loginfo_throttle_identical(1, 'A source with a higher priority is controlling, skipping the current control request')
        return
    
    last_ctrl = (0, ctime)
    pcmd_pub.publish(Twist(angular=Vector3(z=data.data)))
    ctrl_source_pub.publish('pid_controller')

def handle_gimbal_ctrl_gimbal(data: Float64):
    global last_ctrl, gimbal_pub, ctrl_source_pub
    ctime = rospy.get_rostime().to_sec()
    p, t = last_ctrl
    if p > 0 and ctime - t <= timeout:
        rospy.loginfo_throttle_identical(1, 'A source with a higher priority is controlling, skipping the current control request')
        return
    
    last_ctrl = (0, ctime)
    gimbal_pub.publish(data)
    ctrl_source_pub.publish('pid_controller')

if __name__ == '__main__':
    try:
        rospy.init_node('controller_mux')
        pcmd_pub = rospy.Publisher('pcmd', Twist, queue_size=1)
        gimbal_pub = rospy.Publisher('gimbal', Float64, queue_size=1)
        ctrl_source_pub = rospy.Publisher('control_source', String, queue_size=1, latch=True)
        teleop_pcmd_sub = rospy.Subscriber('teleop_pcmd', Twist, handle_teleop_pcmd)
        teleop_gimbal_sub = rospy.Subscriber('teleop_gimbal', Float64, handle_teleop_gimbal)
        yaw_ctrl_yaw_sub = rospy.Subscriber('yaw_controller_yaw', Float64, handle_yaw_ctrl_yaw)
        gimbal_ctrl_gimbal_sub = rospy.Subscriber('gimbal_controller_gimbal', Float64, handle_gimbal_ctrl_gimbal)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass