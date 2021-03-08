#!/usr/bin/env python3

import rospy
import tf2_ros
import message_filters

from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import TransformStamped, Vector3

class TfSubscriber(message_filters.Subscriber):
    def __init__(self, *args, target_frame: str = None, source_frame: str = None, **kwargs):
        self.target_frame = target_frame
        self.source_frame = source_frame
        super().__init__(*args, **kwargs)
    
    def callback(self, msg):
        for tr in msg.transforms:
            if (tr.header.frame_id == self.source_frame or self.source_frame is None) and (tr.child_frame_id == self.target_frame or self.target_frame is None):
                self.signalMessage(tr)

class TfSynchorizer(message_filters.TimeSynchronizer):
    def __init__(self, pairs, tf_callback = None):
        assert len(pairs) >= 1
        subs = [TfSubscriber('/tf', TFMessage, target_frame=target_frame, source_frame=source_frame) for (target_frame, source_frame) in pairs]
        super().__init__(subs, len(subs))
        self.buffer = tf2_ros.Buffer()
        self.registerCallback(self.handle_synced_transforms)
        self.tf_callbacks = {}
        if tf_callback is not None:
            self.register_tf_callback(tf_callback)
    
    def handle_synced_transforms(self, *args: TransformStamped):
        stamp = args[0].header.stamp

        for transform in args:
            self.buffer.set_transform(transform, 'default_authority')
        
        for (cb, args) in self.tf_callbacks.values():
            cb(*(self.buffer, stamp, *args))
    
    def register_tf_callback(self, cb, *args):
        conn = len(self.tf_callbacks)
        self.tf_callbacks[conn] = (cb, args)
        return conn

GOAL_RELATIVE_TO_PILOT = 'pilot'
GOAL_RELATIVE_TO_WORLD = 'map'

param_map_frame_id = 'map'
param_goal_frame_id = 'goal'

class FollowerBase():
    TRANSFORM_RELATIONS = [('base_link', 'map'), ('pilot', 'map'), ('base_link_norp', 'map')]
    def __init__(self, frame_of_reference: str, active_axes: str = 'xyz', enabled: bool = False) -> None:
        TfSynchorizer(FollowerBase.TRANSFORM_RELATIONS, self.handle_update_base)

        # for easy interfacing with pid package
        self.d_x_pub = rospy.Publisher('dx', Float64, queue_size=2)
        self.d_y_pub = rospy.Publisher('dy', Float64, queue_size=2)
        self.d_z_pub = rospy.Publisher('dz', Float64, queue_size=2)

        self.enable_x_pub = rospy.Publisher('enable_x', Bool, queue_size=1, latch=True)
        self.enable_y_pub = rospy.Publisher('enable_y', Bool, queue_size=1, latch=True)
        self.enable_z_pub = rospy.Publisher('enable_z', Bool, queue_size=1, latch=True)

        self.active_axes = active_axes
        self.tf_pub = tf2_ros.TransformBroadcaster()
        self.frame_of_reference = frame_of_reference
        self.set_enabled(enabled)
        
    def set_enabled(self, enabled: bool):
        self.enabled = enabled
        self.enable_x_pub.publish(data=(('x' in self.active_axes) and enabled))
        self.enable_y_pub.publish(data=(('y' in self.active_axes) and enabled))
        self.enable_z_pub.publish(data=(('z' in self.active_axes) and enabled))

    def handle_update(self, tf_buffer: tf2_ros.Buffer, stamp: rospy.Time) -> Vector3:
        return None

    def handle_update_base(self, tf_buffer: tf2_ros.Buffer, stamp: rospy.Time) -> Vector3:
        goal_vector = self.handle_update(tf_buffer, stamp)
        if goal_vector is None:
            return
        
        goal_tr = TransformStamped()
        goal_tr.header.stamp = stamp
        goal_tr.header.frame_id = self.frame_of_reference
        goal_tr.child_frame_id = param_goal_frame_id
        goal_tr.transform.rotation.x = 1.0 # identity quat
        goal_tr.transform.translation = goal_vector
        self.tf_pub.sendTransform(goal_tr)
        tf_buffer.set_transform(goal_tr, 'default_authority')
        d_tr = tf_buffer.lookup_transform('base_link_norp', param_goal_frame_id, stamp)
        self.d_x_pub.publish(data=-d_tr.transform.translation.x)
        self.d_y_pub.publish(data=-d_tr.transform.translation.y)
        self.d_z_pub.publish(data=-d_tr.transform.translation.z)

# Follow the target keeping the same vector while looking at the target
class GeographicFollower(FollowerBase):
    def __init__(self) -> None:
        super().__init__(frame_of_reference=GOAL_RELATIVE_TO_PILOT, active_axes='xy')
    
    def handle_update(self, tf_buffer: tf2_ros.Buffer, stamp: rospy.Time) -> Vector3:
        # TODO: parameterize these offsets
        return Vector3(x=10.0, y=0.0, z=0.0)

# Look at the target without moving automatically
class LookAtFollower(FollowerBase):
    def __init__(self) -> None:
        super().__init__(frame_of_reference=GOAL_RELATIVE_TO_WORLD, active_axes='')
    
    def handle_update(self, tf_buffer: tf2_ros.Buffer, stamp: rospy.Time) -> Vector3:
        return Vector3(x=0.0, y=0.0, z=0.0)

# Follow the target keeping the same orientation to its direction
class RelativeFollower(FollowerBase):
    def __init__(self) -> None:
        super().__init__(frame_of_reference=GOAL_RELATIVE_TO_PILOT, active_axes='xy')
    
    def handle_update(self, tf_buffer: tf2_ros.Buffer, stamp: rospy.Time) -> Vector3:
        raise NotImplementedError()

# Follow the target as it was held by a leash
class LeashFollower(FollowerBase):
    def __init__(self) -> None:
        super().__init__(frame_of_reference=GOAL_RELATIVE_TO_PILOT, active_axes='xy')
    
    def handle_update(self, tf_buffer: tf2_ros.Buffer, stamp: rospy.Time) -> Vector3:
        raise NotImplementedError()

class CircleAroundFollower(FollowerBase):
    def __init__(self) -> None:
        super().__init__(frame_of_reference=GOAL_RELATIVE_TO_PILOT, active_axes='xy')
    
    def handle_update(self, tf_buffer: tf2_ros.Buffer, stamp: rospy.Time) -> Vector3:
        raise NotImplementedError()

class NearestPointOnBezierCurveFollower(FollowerBase):
    def __init__(self) -> None:
        super().__init__(frame_of_reference=GOAL_RELATIVE_TO_WORLD, active_axes='xy')
    
    def handle_update(self, tf_buffer: tf2_ros.Buffer, stamp: rospy.Time) -> Vector3:
        raise NotImplementedError()

# Follow a Bezier Curve at constant velocity
class BezierCurveFollower(FollowerBase):
    def __init__(self) -> None:
        super().__init__(frame_of_reference=GOAL_RELATIVE_TO_WORLD, active_axes='xy')
    
    def handle_update(self, tf_buffer: tf2_ros.Buffer, stamp: rospy.Time) -> Vector3:
        raise NotImplementedError()

# Follow a Bezier Curve at constant velocity with distance constraint to pilot
class BezierCurveWithDistanceConstraintFollower(FollowerBase):
    def __init__(self) -> None:
        super().__init__(frame_of_reference=GOAL_RELATIVE_TO_WORLD, active_axes='xy')
    
    def handle_update(self, tf_buffer: tf2_ros.Buffer, stamp: rospy.Time) -> Vector3:
        raise NotImplementedError()

def handle_enable(enabled: Bool):
    # TODO: run sanity checks before starting
    rospy.logwarn('Setting follower enabled to {}'.format(enabled.data))
    follower.set_enabled(enabled.data)

if __name__ == '__main__':
    rospy.init_node('alpha_target_follower')
    follower = GeographicFollower()
    enabled_sub = rospy.Subscriber('enabled', Bool, handle_enable)
    rospy.spin()

