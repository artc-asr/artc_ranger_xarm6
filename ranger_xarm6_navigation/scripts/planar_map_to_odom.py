#!/usr/bin/env python3
"""The localizer's map -> odom, flattened to the plane (x, y, yaw).

lidar_localization_ros2 matches in 6D and publishes <ndt_frame> -> odom
(navigation.launch.py localization:=ndt: robot_a_map_3d). Its z, roll and
pitch follow the FAST-LIO map's own height drift, not the robot's (which
never leaves the floor): 0.14 m and 2.2 deg were seen, which raised and
tilted everything placed in the map frame for MoveIt (tabletops, arm
poses) enough that no arm pose over a table was collision-free.

map -> odom should be only x, y and yaw of it, like AMCL's. odom can't
have a second parent, so this publishes the correction map -> <ndt_frame>
= planar(T) * T^-1 (T: the latest <ndt_frame> -> odom), making map ->
odom planar(T) through the chain, at 'rate' (stamped transform_tolerance
ahead, as AMCL does, so lookups at the latest sensor time don't need to
extrapolate).

Also relays 'initialpose' (RViz's 2D Pose Estimate, in map) to
'localization/initialpose' in the ndt frame, where the localizer takes it:
the two frames agree in x, y and yaw.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from tf_transformations import euler_from_matrix, quaternion_from_matrix, quaternion_matrix


class PlanarMapToOdom(Node):
    def __init__(self):
        super().__init__('planar_map_to_odom')
        p = self.declare_parameter
        self.map_frame = p('map_frame', 'map').value
        self.ndt_frame = p('ndt_frame', 'map_3d').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.tolerance = Duration(seconds=p('transform_tolerance', 0.2).value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, 'localization/initialpose', 10)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self._initialpose_cb, 10)
        self.create_timer(1.0 / p('rate', 20.0).value, self._tick)
        self.have = False

    def _tick(self):
        try:
            t = self.buffer.lookup_transform(self.ndt_frame, self.odom_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001  not localized yet
            return
        q, p = t.transform.rotation, t.transform.translation
        full = quaternion_matrix([q.x, q.y, q.z, q.w])
        full[:3, 3] = [p.x, p.y, p.z]
        yaw = euler_from_matrix(full, 'rzyx')[0]
        planar = np.eye(4)
        planar[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
        planar[:2, 3] = [p.x, p.y]
        correction = planar @ np.linalg.inv(full)
        out = TransformStamped()
        out.header.stamp = (self.get_clock().now() + self.tolerance).to_msg()
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.ndt_frame
        out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = correction[:3, 3]
        (out.transform.rotation.x, out.transform.rotation.y,
         out.transform.rotation.z, out.transform.rotation.w) = quaternion_from_matrix(correction)
        self.broadcaster.sendTransform(out)
        if not self.have:
            self.have = True
            self.get_logger().info(f'{self.map_frame} -> {self.odom_frame}: {self.ndt_frame}\'s, planar (x, y, yaw)')

    def _initialpose_cb(self, msg):
        msg.header.frame_id = self.ndt_frame
        self.initialpose_pub.publish(msg)


def main():
    rclpy.init()
    node = PlanarMapToOdom()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
