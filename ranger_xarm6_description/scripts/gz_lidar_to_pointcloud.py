#!/usr/bin/env python3
"""Convert a gz-sim gpu_lidar's raw multi-channel scan into a real
sensor_msgs/PointCloud2, subscribing to gz-transport directly.

gz-sim's gpu_lidar sensor type has no native point-cloud output (checked:
libgz-sensors8-gpu-lidar.so has zero references to "points" anywhere,
unlike libgz-sensors8-rgbd_camera.so, which does -- rgbd_camera is a
special combined sensor class purpose-built to also emit
PointCloudPacked, gpu_lidar never was). It only ever publishes a flat
gz.msgs.LaserScan.

ros_gz_bridge's own sensor_msgs/LaserScan converter was tried first and
turned out to silently truncate a multi-vertical-channel scan down to
just its first `count` (horizontal-only) values -- verified live: a
1200x32 sensor bridged to a ROS LaserScan came through with exactly 1200
ranges, not 38400. gz.msgs.LaserScan's own proto (gz.msgs10.laserscan_pb2)
carries the full data ROS's 2D-only LaserScan type has no room for
(vertical_count, vertical_angle_min/max/step, and a full count*
vertical_count-sized ranges/intensities array), so this node subscribes
to the raw gz topic directly via gz-transport (same
gz.transport13/gz.msgs10 Python bindings scripts/base_pose_publisher.py
already uses for teleporting), reshapes the flat arrays into a
(vertical_count, count) grid, converts each valid beam to XYZ via
spherical-to-Cartesian (X-forward/Y-left/Z-up, gz-sim's own lidar
convention), and republishes as a proper sensor_msgs/PointCloud2.
frame_id comes straight from the message's own 'frame' field (populated
by the sensor tag's <gz_frame_id>, same mechanism verified working for
the IMU); the ROS-side stamp uses this node's own sim-time-aware clock
(use_sim_time, set in ranger_xarm6.launch.py), same convention
base_pose_publisher.py already uses, rather than parsing the gz message's
own embedded timestamp.
"""
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from gz.msgs10 import laserscan_pb2
from gz.transport13 import Node as GzNode


class LidarToPointCloud(Node):
    def __init__(self, gz_topic, ros_topic):
        super().__init__('gz_lidar_to_pointcloud')
        self.pub = self.create_publisher(PointCloud2, ros_topic, 10)
        self._gz_node = GzNode()
        self._gz_node.subscribe(laserscan_pb2.LaserScan, gz_topic, self._cb)

    def _cb(self, msg):
        h_count = msg.count
        v_count = max(msg.vertical_count, 1)
        expected = h_count * v_count
        if len(msg.ranges) != expected:
            self.get_logger().warn(
                f'Got {len(msg.ranges)} ranges, expected '
                f'{h_count}x{v_count}={expected}; dropping scan.',
                throttle_duration_sec=5.0)
            return

        ranges = np.array(msg.ranges, dtype=np.float64).reshape(v_count, h_count)
        if msg.intensities:
            intensities = np.array(msg.intensities, dtype=np.float64).reshape(v_count, h_count)
        else:
            intensities = np.zeros_like(ranges)

        horizontal_angles = msg.angle_min + np.arange(h_count) * msg.angle_step
        if v_count > 1:
            vertical_angles = msg.vertical_angle_min + np.arange(v_count) * msg.vertical_angle_step
        else:
            vertical_angles = np.array([0.0])

        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        v_idx, h_idx = np.nonzero(valid)
        r = ranges[v_idx, h_idx]
        h_ang = horizontal_angles[h_idx]
        v_ang = vertical_angles[v_idx]

        cos_v = np.cos(v_ang)
        x = r * cos_v * np.cos(h_ang)
        y = r * cos_v * np.sin(h_ang)
        z = r * np.sin(v_ang)
        intensity = intensities[v_idx, h_idx]

        points = np.column_stack((x, y, z, intensity)).astype(np.float32)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        header = Header()
        header.frame_id = msg.frame
        header.stamp = self.get_clock().now().to_msg()
        cloud = point_cloud2.create_cloud(header, fields, points)
        self.pub.publish(cloud)


def main():
    rclpy.init()
    node = LidarToPointCloud(gz_topic=sys.argv[1], ros_topic=sys.argv[2])
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
