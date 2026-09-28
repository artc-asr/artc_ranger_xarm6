"""map -> odom for publishers of the world's models (world_collision_objects.py,
world_markers.py).

The world file's models are placed in the map frame: the room. Their
consumers work in odom (MoveIt's planning frame, the description's RViz),
which is the room only while odometry doesn't drift (the sim's ground-truth
TF, with an identity map -> odom). With the EKF and a localizer, odom
drifts and map -> odom corrects it, so the models are sent in odom through
the current map -> odom, and sent again when it moves.

No map frame (Nav2 isn't running, e.g. MoveIt alone): the identity, right
for the sim's ground-truth odom.

Smoothed (x, y, yaw; map -> odom is planar) with time constant tau: the
localizer's estimate jitters ±2 cm scan to scan while the drift it
corrects moves slowly, and resending the tables at every jitter moved them
under a docking plan mid-motion (MoveIt aborted it:
MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE).
"""
import math

import numpy as np
import rclpy
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion


class MapToOdom:
    def __init__(self, node, map_frame, odom_frame, tolerance=0.02, angle_tolerance=0.01, tau=5.0):
        self.node = node
        self.map_frame, self.odom_frame = map_frame, odom_frame
        self.tolerance, self.angle_tolerance = tolerance, angle_tolerance  # [m], [rad]: moved enough to resend
        self.tau = tau  # [s] smoothing
        self.xyyaw = None  # smoothed map -> odom
        self.last = None   # ROS time [s] of the last update
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.matrix = np.eye(4)   # points in map -> odom
        self.sent = None          # the matrix last sent with
        self.have_map = False

    def update(self):
        """Look up the latest map -> odom; keep the last one (or the identity) if there's none."""
        if self.map_frame == self.odom_frame:
            return self.matrix
        try:
            t = self.buffer.lookup_transform(self.odom_frame, self.map_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001  no map frame (yet, or at all)
            return self.matrix
        q, p = t.transform.rotation, t.transform.translation
        x, y, yaw = p.x, p.y, euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        now = self.node.get_clock().now().nanoseconds * 1e-9
        if self.xyyaw is None:
            self.xyyaw = [x, y, yaw]
            self.node.get_logger().info(f'{self.map_frame} -> {self.odom_frame} found: world models follow it')
        else:
            k = 1.0 - math.exp(-max(0.0, now - self.last) / self.tau)
            sx, sy, syaw = self.xyyaw
            self.xyyaw = [sx + k * (x - sx), sy + k * (y - sy), syaw + k * math.remainder(yaw - syaw, 2 * math.pi)]
        self.last = now
        self.have_map = True
        sx, sy, syaw = self.xyyaw
        m = np.eye(4)
        m[:2, :2] = [[math.cos(syaw), -math.sin(syaw)], [math.sin(syaw), math.cos(syaw)]]
        m[:2, 3] = [sx, sy]
        self.matrix = m
        return m

    def moved(self):
        """Whether map -> odom moved past the tolerances since mark_sent()."""
        if self.sent is None:
            return True
        d = np.linalg.inv(self.sent) @ self.matrix
        angle = np.arccos(np.clip((np.trace(d[:3, :3]) - 1.0) / 2.0, -1.0, 1.0))
        return np.linalg.norm(d[:3, 3]) > self.tolerance or angle > self.angle_tolerance

    def mark_sent(self):
        self.sent = self.matrix.copy()
