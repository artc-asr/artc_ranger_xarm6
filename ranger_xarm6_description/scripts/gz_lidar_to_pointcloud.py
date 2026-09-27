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
the IMU).

The cloud has livox_ros_driver2's own PointCloud2 layout (xfer_format 0,
LivoxPointXyzrtlt, packed, 26 bytes): x y z intensity (float32), tag,
line (uint8), timestamp (float64, the point's absolute time in ns), so
anything reading it works the same on hardware. line is the channel
index mod 4 (the Mid-360 has 4 lines); tag 0. gz-sim renders a whole
scan at one instant, so every point gets the scan's time, and
header.stamp is that time too (the real driver stamps the first point's
time): the gz message's own sim-time stamp, i.e. when it was captured.
The sim therefore has no motion distortion within a scan, unlike the
real sensor.
"""
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

from gz.msgs10 import laserscan_pb2
from gz.transport13 import Node as GzNode

# livox_ros_driver2's LivoxPointXyzrtlt (comm.h, #pragma pack(1)).
LIVOX_POINT = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
    ('tag', 'u1'), ('line', 'u1'), ('timestamp', '<f8'),
])
LIVOX_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='tag', offset=16, datatype=PointField.UINT8, count=1),
    PointField(name='line', offset=17, datatype=PointField.UINT8, count=1),
    PointField(name='timestamp', offset=18, datatype=PointField.FLOAT64, count=1),
]


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

        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nsec
        points = np.zeros(len(r), dtype=LIVOX_POINT)
        points['x'], points['y'], points['z'] = x, y, z
        points['intensity'] = intensity
        points['line'] = v_idx % 4
        points['timestamp'] = float(stamp_ns)

        cloud = PointCloud2()
        cloud.header.frame_id = msg.frame
        cloud.header.stamp.sec, cloud.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = LIVOX_FIELDS
        cloud.is_bigendian = False
        cloud.point_step = LIVOX_POINT.itemsize
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = points.tobytes()
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
