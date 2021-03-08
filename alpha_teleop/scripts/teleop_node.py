#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64, Bool, Empty

last_takeoff = 0
last_landing = 0
last_toggle_follower = 0

follower_status = False

def handle_joystick_msg(data: Joy):
    global last_takeoff, last_landing, last_toggle_follower, follower_status, axis_map
    try:
        yaw = data.axes[axis_map[0]]
        gaz = data.axes[axis_map[1]]
        roll = data.axes[axis_map[2]]
        pitch = data.axes[axis_map[3]]
        gimbal_pitch = data.axes[axis_map[4]]
        takeoff = data.buttons[0]
        landing = data.buttons[1]
        toggle_follower = data.buttons[2]

    except IndexError:
        rospy.logwarn_throttle_identical(5.0, 'Invalid Joy message received')
        return
    
    if toggle_follower == 1 and last_toggle_follower == 0:
        follower_status = not follower_status
        follower_enabled_pub.publish(data=follower_status)
        
    last_toggle_follower = toggle_follower

    if takeoff == 1 and last_takeoff != 1:
        rospy.loginfo('Taking off')
        takeoff_pub.publish()
    last_takeoff = takeoff

    if landing == 1 and last_landing != 1:
        rospy.loginfo('Landing')
        landing_pub.publish()
    last_landing = landing

    pcmd_pub.publish(Twist(
        linear=Vector3(pitch * 100.0, roll * 100.0, gaz * 100.0),
        angular=Vector3(0, 0, yaw * 100.0)
    ))

    gimbal_pub.publish(data=gimbal_pitch)

if __name__ == '__main__':
    try:
        rospy.init_node('alpha_teleop')
        axis_map = [int(x) for x in rospy.get_param('~axis_map', '0 1 3 4 7').split(' ')]
        assert len(axis_map) == 5
        rospy.loginfo('Using axis map: {}'.format(axis_map))
        pcmd_pub = rospy.Publisher('pcmd', Twist, queue_size=1)
        gimbal_pub = rospy.Publisher('gimbal', Float64, queue_size=1)
        joy_sub = rospy.Subscriber('joy', Joy, handle_joystick_msg)
        takeoff_pub = rospy.Publisher('takeoff', Empty, queue_size=1)
        landing_pub = rospy.Publisher('landing', Empty, queue_size=1)
        follower_enabled_pub = rospy.Publisher('follower_enabled', Bool, queue_size=1, latch=True)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
