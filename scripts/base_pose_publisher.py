#!/usr/bin/env python3
"""Sole owner of Ranger's odom->base_link static transform.

Replaces the plain tf2_ros static_transform_publisher CLI node
ranger_xarm6.launch.py used to launch directly. That CLI tool only
publishes once, from fixed launch args, and can't be told to republish --
so when wbc_visualize.py's "Initial Position" button teleports the Gazebo
entity, there was no way to update the TF to match without a second
static-transform publisher fighting the first one over the same
odom->base_link edge. Two publishers latching the same edge on
/tf_static is a real race for any subscriber connecting after the
teleport (tf2_echo, a restarted RViz, wbc.py started late): whichever
publisher's retained history is redelivered last wins, non-deterministically.

This node is the only thing that ever publishes odom->base_link for
Ranger. wbc_visualize.py updates it by publishing to 'set_base_pose'
instead of broadcasting the transform itself.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose2D, TransformStamped
from tf2_msgs.msg import TFMessage
from tf_transformations import quaternion_from_euler


class BasePosePublisher(Node):
    def __init__(self):
        super().__init__('base_pose_publisher')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        # NOT tf2_ros.StaticTransformBroadcaster: its sendTransform() only
        # ever ADDS a child_frame_id to its internal set and republishes the
        # accumulated message -- calling it again for a child_frame_id it's
        # already seen is a silent no-op, so it can never actually update an
        # existing static transform (confirmed by reading its source,
        # tf2_ros/static_transform_broadcaster.py). Publishing TFMessage
        # directly with the same QoS that class would have used sidesteps
        # that and lets us genuinely replace the transform on every call.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          history=HistoryPolicy.KEEP_LAST)
        self.pub_tf_static = self.create_publisher(TFMessage, '/tf_static', qos)

        # z isn't part of set_base_pose (Pose2D has no z) -- Ranger only
        # ever repositions in-plane (teleporting to face the wall), so z
        # stays at whatever it was launched with; keep it as instance state
        # rather than silently resetting it to 0 on every update.
        self.z = self.get_parameter('z').value
        self._publish(
            self.get_parameter('x').value,
            self.get_parameter('y').value,
            self.get_parameter('yaw').value,
        )

        self.create_subscription(Pose2D, 'set_base_pose', self._set_base_pose_cb, 10)
        self.get_logger().info(
            f'Base pose publisher ready ({self.frame_id} -> {self.child_frame_id})')

    def _set_base_pose_cb(self, msg):
        self._publish(msg.x, msg.y, msg.theta)
        self.get_logger().info(f'Base pose updated: x={msg.x} y={msg.y} yaw={msg.theta}')

    def _publish(self, x, y, yaw):
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = self.z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.pub_tf_static.publish(TFMessage(transforms=[t]))


def main(args=None):
    rclpy.init(args=args)
    node = BasePosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
