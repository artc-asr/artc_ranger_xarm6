#!/usr/bin/env python3
"""Publish the base's pose as MoveIt's base joint states.

MoveIt's URDF (urdf/ranger_xarm6_moveit.urdf.xacro) models the mobile
base as base_z_joint / base_x_joint / base_y_joint / base_theta_joint
between odom and base_link. Nothing on the robot has those joints: the
base's pose is the odom -> base_link TF (base_pose_publisher.py in sim,
the Ranger driver on hardware). This node reads that TF and publishes it
on joint_states as those joints, next to joint_state_broadcaster's arm/wheel joints, so
move_group's current state (and the planning scene) follows the real
base. Roll and pitch have no joint to go to (the Ranger's odom has none).
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


class BaseJointStatePublisher(Node):
    def __init__(self):
        super().__init__('base_joint_state_publisher')
        prefix = self.declare_parameter('prefix', '').value
        self.odom_frame = self.declare_parameter('odom_frame', f'{prefix}odom').value
        self.base_frame = self.declare_parameter('base_frame', f'{prefix}base_link').value
        rate = self.declare_parameter('rate', 50.0).value
        self.names = [f'{prefix}base_x_joint', f'{prefix}base_y_joint', f'{prefix}base_z_joint',
                      f'{prefix}base_theta_joint']
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(1.0 / rate, self.publish)
        self.warned = False

    def publish(self):
        try:
            t = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())
        except TransformException as e:
            if not self.warned:
                self.get_logger().warn(f'Waiting for {self.odom_frame} -> {self.base_frame}: {e}')
                self.warned = True
            return
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        msg = JointState()
        msg.header.stamp = t.header.stamp
        msg.name = self.names
        p = t.transform.translation
        msg.position = [p.x, p.y, p.z, yaw]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = BaseJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
