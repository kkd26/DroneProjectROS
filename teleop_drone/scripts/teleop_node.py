#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3
from std_srvs.srv import Trigger

last_takeoff = 0
last_landing = 0

def handle_joystick_msg(data: Joy):
    global last_takeoff, last_landing
    gaz = data.axes[1]
    yaw = data.axes[0]
    pitch = data.axes[4]
    roll = data.axes[3]

    takeoff = data.buttons[0]
    if takeoff == 1 and last_takeoff != 1:
        rospy.loginfo('Taking off')
        takeoff_func()
    last_takeoff = takeoff

    landing = data.buttons[1]
    if landing == 1 and last_landing != 1:
        rospy.loginfo('Landing')
        landing_func()
    last_landing = landing

    pcmd_pub.publish(Twist(
        linear=Vector3(pitch * 100.0, -roll * 100.0, gaz * 100.0),
        angular=Vector3(0, 0, -yaw * 100.0)
    ))

if __name__ == '__main__':
    try:
        rospy.init_node('teleop_drone')
        pcmd_pub = rospy.Publisher('pcmd', Twist, queue_size=1)
        joy_sub = rospy.Subscriber('joy', Joy, handle_joystick_msg)
        takeoff_func = rospy.ServiceProxy('takeoff', Trigger)
        landing_func = rospy.ServiceProxy('landing', Trigger)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
