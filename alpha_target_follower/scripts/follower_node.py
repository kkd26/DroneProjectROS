#!/usr/bin/env python3

import rospy
import tf2_ros
import message_filters
import json
import numpy

from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import TransformStamped, Vector3
from alpha_target_follower.srv import SetFollower, SetFollowerRequest, SetFollowerResponse

class TfSubscriber(message_filters.Subscriber):
    def __init__(self, *args, target_frame: str = None, source_frame: str = None, **kwargs):
        self.target_frame = target_frame
        self.source_frame = source_frame
        super().__init__(*args, **kwargs)
    
    def callback(self, msg):
        for tr in msg.transforms:
            if (tr.header.frame_id == self.source_frame or self.source_frame is None) and (tr.child_frame_id == self.target_frame or self.target_frame is None):
                self.signalMessage(tr)
    
    def unregister(self):
        self.sub.unregister()

class TfSynchorizer(message_filters.TimeSynchronizer):
    def __init__(self, pairs, tf_callback = None):
        assert len(pairs) >= 1
        self.subs = subs = [TfSubscriber('/tf', TFMessage, target_frame=target_frame, source_frame=source_frame) for (target_frame, source_frame) in pairs]
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
    
    def unregister(self):
        for sub in self.subs:
            sub.unregister()

GOAL_RELATIVE_TO_PILOT = 'pilot'
GOAL_RELATIVE_TO_WORLD = 'map'

param_map_frame_id = 'map'
param_goal_frame_id = 'goal'

class FollowerBase():
    TRANSFORM_RELATIONS = [('base_link', 'map'), ('pilot', 'map'), ('base_link_norp', 'map')]
    def __init__(self, frame_of_reference: str, active_axes: str = 'xyz', enabled: bool = True) -> None:
        self.tf_sync = TfSynchorizer(FollowerBase.TRANSFORM_RELATIONS, self.handle_update_base)

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

        self.is_first_update = True
        
    def set_enabled(self, enabled: bool):
        self.enabled = enabled
        self.enable_x_pub.publish(data=(('x' in self.active_axes) and enabled))
        self.enable_y_pub.publish(data=(('y' in self.active_axes) and enabled))
        self.enable_z_pub.publish(data=(('z' in self.active_axes) and enabled))

    def handle_update(self, tf_buffer: tf2_ros.Buffer, stamp: rospy.Time, is_first_update: bool) -> Vector3:
        return None

    def handle_update_base(self, tf_buffer: tf2_ros.Buffer, stamp: rospy.Time) -> Vector3:
        goal_vector = self.handle_update(tf_buffer, stamp, self.is_first_update)
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
        self.is_first_update = False
    
    def unregister(self):
        self.tf_sync.unregister()
        self.d_x_pub.unregister()
        self.d_y_pub.unregister()
        self.d_z_pub.unregister()
        self.enable_x_pub.unregister()
        self.enable_y_pub.unregister()
        self.enable_z_pub.unregister()
        self.tf_pub.pub_tf.unregister()

# Follow the target keeping the same vector while looking at the target
class GeographicFollower(FollowerBase):
    def __init__(self) -> None:
        super().__init__(frame_of_reference=GOAL_RELATIVE_TO_PILOT, active_axes='xyz')
    
    def handle_update(self, tf_buffer: tf2_ros.Buffer, stamp: rospy.Time, is_first_update: bool) -> Vector3:
        if is_first_update:
            d_tr = tf_buffer.lookup_transform('pilot', 'base_link_norp', stamp)
            rospy.loginfo('GeographicFollower: initial offset is {}'.format(d_tr.transform.translation))
            self.offset_vector = d_tr.transform.translation

        return self.offset_vector

class DummyFollower(FollowerBase):
    def __init__(self) -> None:
        super().__init__(frame_of_reference=GOAL_RELATIVE_TO_WORLD, active_axes='')
    
    def handle_update(self, tf_buffer: tf2_ros.Buffer, stamp: rospy.Time, is_first_update: bool) -> Vector3:
        return Vector3(x=0.0, y=0.0, z=0.0)

class NearestPointOnBezierCurveFollower(FollowerBase):
    def __init__(self, path) -> None:
        super().__init__(frame_of_reference=GOAL_RELATIVE_TO_WORLD, active_axes='xy')
    
    def handle_update(self, tf_buffer: tf2_ros.Buffer, stamp: rospy.Time, is_first_update: bool) -> Vector3:
        pilot_location = tf_buffer.lookup_transform('map', 'pilot', stamp)
        drone_location = tf_buffer.lookup_transform('map', 'base_link_norp', stamp)
        pilot_location = TransformStamped()
        
        pilot_location_np = numpy.array(
            [
                pilot_location.transform.translation.x, 
                pilot_location.transform.translation.y, 
                pilot_location.transform.translation.z
            ]
        )

        drone_location_np = numpy.array(
            [
                drone_location.transform.translation.x, 
                drone_location.transform.translation.y, 
                drone_location.transform.translation.z
            ]
        )
        rospy.loginfo('Pilot location: {}'.format(pilot_location_np))
        rospy.loginfo('Drone location: {}'.format(drone_location_np))

        raise NotImplementedError()


FOLLOWER_MAPPINGS = {
    'dummy': DummyFollower,
    'geographic': GeographicFollower,
    'bezier': NearestPointOnBezierCurveFollower
}

def handle_set_follower(req: SetFollowerRequest):
    global follower
    success = False
    param = ({} if req.param == '' else json.loads(req.param))
    follower.unregister()
    try:
        follower = FOLLOWER_MAPPINGS[req.name](**param)
        rospy.loginfo('Follower mode {} activated'.format(req.name))
        success = True
    except Exception as e:
        rospy.logerr('Error setting follower mode to {}: {}'.format(req.name, e))
        rospy.logwarn('Falling back to Dummy follower')
        follower = DummyFollower()
        success = False

    for k, v in FOLLOWER_MAPPINGS.items():
        if type(follower) is v:
            follower_mode_pub.publish(data=k)
            break
        
    return SetFollowerResponse(success=success)

if __name__ == '__main__':
    rospy.init_node('alpha_target_follower')
    follower = DummyFollower()

    set_follower_srv = rospy.Service('set_follower', SetFollower, handle_set_follower)
    follower_mode_pub = rospy.Publisher('follower_mode', String, queue_size=1, latch=True)
    follower_mode_pub.publish(data='dummy')
    rospy.spin()

