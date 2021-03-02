#!/usr/bin/env python3

import tf2_ros
import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

last_odom = Odometry()

def handle_pilot_odom(odom: Odometry):
    global last_odom
    last_odom = odom

def handle_sync(sync: Odometry):
    global last_odom
    stamp = sync.header.stamp
    dt = stamp.to_sec() - last_odom.header.stamp.to_sec()

    last_pos = last_odom.pose.pose.position
    last_vel = last_odom.twist.twist.linear

    odom = Odometry()
    odom.header.stamp = stamp
    odom.header.frame_id = last_odom.header.frame_id
    odom.child_frame_id = param_pilot_frame_id

    odom.pose.pose.position.x = last_pos.x + dt * last_vel.x
    odom.pose.pose.position.y = last_pos.y + dt * last_vel.y
    odom.pose.pose.position.z = last_pos.z + dt * last_vel.z
    odom.pose.pose.orientation = last_odom.pose.pose.orientation

    odom.twist.twist.linear = last_vel

    odom_pub.publish(odom)

    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = last_odom.header.frame_id
    tf.child_frame_id = param_pilot_frame_id

    tf.transform.translation = odom.pose.pose.position
    tf.transform.rotation = odom.pose.pose.orientation
    tf_broadcaster.sendTransform(tf)

if __name__ == '__main__':
    rospy.init_node('alpha_target_extrapolator')
    param_pilot_frame_id = rospy.get_param('pilot_frame_id', 'pilot')
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=2)
    rospy.Subscriber('odom_raw', Odometry, handle_pilot_odom)
    rospy.Subscriber('sync', Odometry, handle_sync)
    rospy.spin()
