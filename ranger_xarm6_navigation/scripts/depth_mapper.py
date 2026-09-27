#!/usr/bin/env python3
"""Front D435i depth -> FAST-LIO's map frame, while mapping: the low obstacles.

The Mid-360 (0.84 m up, lowest beam -7 deg) never sees what's low and
close, so the FAST-LIO map misses low obstacles the robot drove past. This
node places the front depth camera's cloud (depth_to_cloud's, 5 cm voxels,
within 3 m) in FAST-LIO's 'camera_init' frame with FAST-LIO's pose at each
depth frame, and writes the voxels seen in at least min_frames frames to
<map>_depth.pcd (every save_period s and on exit). pcd_to_grid.py merges
that file into the 2D grid.

Heights are cut against the floor under the robot (base_link's height
above it, base_height), not against FAST-LIO's z, which drifts a few cm:
kept are points z_min..z_max above that floor. So a low obstacle, which
the lidar map's band has to start above (0.15 m), is kept from 0.05 m.

Topics (in the robot's namespace): fast_lio/odometry (camera_init ->
the IMU, FAST-LIO's 'body'), points (the depth cloud, any frame fixed to
the base).
"""
import math
import os
import threading
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def _stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _matrix(rotation, translation):
    m = np.eye(4)
    m[:3, :3] = rotation.as_matrix()
    m[:3, 3] = translation
    return m


def write_pcd(path, xyz):
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    tmp = path + '.tmp'
    with open(tmp, 'wb') as f:
        f.write((f'# .PCD v0.7 - depth_mapper.py\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
                 f'COUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                 f'POINTS {len(xyz)}\nDATA binary\n').encode())
        f.write(xyz.tobytes())
    os.replace(tmp, path)


class DepthMapper(Node):
    def __init__(self):
        super().__init__('depth_mapper')
        self.output = os.path.expanduser(self.declare_parameter('output', '').value)
        self.body_frame = self.declare_parameter('body_frame', 'livox_imu_frame').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.base_height = self.declare_parameter('base_height', 0.315).value
        self.z_min = self.declare_parameter('z_min', 0.05).value
        self.z_max = self.declare_parameter('z_max', 1.5).value
        self.voxel = self.declare_parameter('voxel_size', 0.05).value
        self.min_frames = self.declare_parameter('min_frames', 3).value
        # Skip depth frames while the pose moves faster than this: motion
        # between the depth and pose stamps smears the obstacles.
        self.max_speed = self.declare_parameter('max_angular_speed', 0.6).value
        save_period = self.declare_parameter('save_period', 10.0).value
        if not self.output:
            raise SystemExit('depth_mapper: set the output parameter (the .pcd to write)')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static = {}           # cloud frame -> (base <- cloud, body <- cloud)
        self.poses = deque(maxlen=400)   # (t, R, p, wz): camera_init <- body
        self.clouds = deque(maxlen=30)
        self.counts = {}           # voxel key -> frames it was seen in
        self.frames = 0
        self.lock = threading.Lock()

        self.create_subscription(Odometry, 'fast_lio/odometry', self._odometry, 50)
        self.create_subscription(PointCloud2, 'points', self._cloud, qos_profile_sensor_data)
        self.create_timer(save_period, self.save)

    def _odometry(self, msg):
        q, p = msg.pose.pose.orientation, msg.pose.pose.position
        self.poses.append((_stamp(msg.header), Rotation.from_quat([q.x, q.y, q.z, q.w]),
                           np.array([p.x, p.y, p.z]), abs(msg.twist.twist.angular.z)))
        self._process()

    def _cloud(self, msg):
        self.clouds.append(msg)
        self._process()

    def _pose_at(self, t):
        """camera_init <- body at t, interpolated; None if not bracketed."""
        poses = self.poses
        if len(poses) < 2 or t < poses[0][0] or t > poses[-1][0]:
            return None
        i = next(k for k in range(1, len(poses)) if poses[k][0] >= t)
        (t0, r0, p0, w0), (t1, r1, p1, w1) = poses[i - 1], poses[i]
        if t1 - t0 > 0.5:
            return None
        a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
        rot = Slerp([0.0, 1.0], Rotation.concatenate([r0, r1]))(a)
        rate = (r0.inv() * r1).magnitude() / max(t1 - t0, 1e-3)
        return _matrix(rot, p0 + a * (p1 - p0)), max(w0, w1, rate)

    def _static(self, frame):
        if frame not in self.static:
            try:
                tfs = [self.tf_buffer.lookup_transform(target, frame, Time()) for target in (self.base_frame, self.body_frame)]
            except TransformException:
                return None
            self.static[frame] = [
                _matrix(Rotation.from_quat([t.transform.rotation.x, t.transform.rotation.y,
                                            t.transform.rotation.z, t.transform.rotation.w]),
                        [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
                for t in tfs]
        return self.static[frame]

    def _process(self):
        while self.clouds and self.poses:
            msg = self.clouds[0]
            t = _stamp(msg.header)
            if t > self.poses[-1][0]:
                return  # wait for FAST-LIO to get there
            self.clouds.popleft()
            pose = self._pose_at(t)
            static = self._static(msg.header.frame_id)
            if pose is None or static is None or pose[1] > self.max_speed:
                continue
            world_body, _ = pose
            base_cloud, body_cloud = static
            xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
            if not len(xyz):
                continue
            h = xyz @ base_cloud[:3, :3].T + base_cloud[:3, 3]
            keep = (h[:, 2] + self.base_height >= self.z_min) & (h[:, 2] + self.base_height <= self.z_max)
            world = world_body @ body_cloud
            pts = xyz[keep] @ world[:3, :3].T + world[:3, 3]
            keys = set(map(tuple, np.floor(pts / self.voxel).astype(np.int64)))
            with self.lock:
                for k in keys:
                    self.counts[k] = self.counts.get(k, 0) + 1
                self.frames += 1

    def save(self):
        with self.lock:
            keys = [k for k, n in self.counts.items() if n >= self.min_frames]
            frames = self.frames
        xyz = (np.array(keys, dtype=np.float64).reshape(-1, 3) + 0.5) * self.voxel
        write_pcd(self.output, xyz)
        self.get_logger().info(f'{len(keys)} voxels from {frames} depth frames -> {self.output}',
                               throttle_duration_sec=60.0)


def main():
    rclpy.init()
    node = DepthMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
