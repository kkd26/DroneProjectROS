#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

last_takeoff = 0
last_landing = 0

def handle_joystick_msg(data: Joy):
    global last_takeoff, last_landing, axis_map
    yaw = data.axes[axis_map[0]]
    gaz = data.axes[axis_map[1]]
    roll = data.axes[axis_map[2]]
    pitch = data.axes[axis_map[3]]
    gimbal_pitch = data.axes[axis_map[4]]

    takeoff = data.buttons[0]
    if takeoff == 1 and last_takeoff != 1:
        # TODO: call takeoff_func asynchronously
        rospy.loginfo('Taking off')
        takeoff_func()
    last_takeoff = takeoff

    landing = data.buttons[1]
    if landing == 1 and last_landing != 1:
        # TODO: call landing_func asynchronously
        rospy.loginfo('Landing')
        landing_func()
    last_landing = landing

    pcmd_pub.publish(Twist(
        linear=Vector3(pitch * 100.0, -roll * 100.0, gaz * 100.0),
        angular=Vector3(0, 0, -yaw * 100.0)
    ))

    gimbal_pub.publish(data=gimbal_pitch)

if __name__ == '__main__':
    try:
        rospy.init_node('teleop_drone')
        axis_map = [int(x) for x in rospy.get_param('~axis_map', '0 1 2 3 7').split(' ')]
        assert len(axis_map) == 5
        rospy.loginfo('Using axis map: {}'.format(axis_map))
        pcmd_pub = rospy.Publisher('pcmd', Twist, queue_size=1)
        gimbal_pub = rospy.Publisher('gimbal', Float64, queue_size=1)
        joy_sub = rospy.Subscriber('joy', Joy, handle_joystick_msg)
        takeoff_func = rospy.ServiceProxy(rospy.get_param('~takeoff_service', 'takeoff'), Trigger)
        landing_func = rospy.ServiceProxy(rospy.get_param('~landing_service', 'landing'), Trigger)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
