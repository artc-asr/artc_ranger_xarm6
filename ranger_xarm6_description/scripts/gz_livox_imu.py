#!/usr/bin/env python3
"""Republish the simulated Mid-360 IMU the way livox_ros_driver2 does.

Subscribes to the gz-sim IMU sensor on livox_imu_frame (see
ranger_xarm6.urdf.xacro) over gz-transport, like gz_lidar_to_pointcloud.py,
and publishes sensor_msgs/Imu on 'livox/imu' with exactly what the real
driver's Lddc::InitImuMsg fills in, so anything downstream sees the same
message in sim and on hardware:
  - linear_acceleration in g, not m/s^2 (the Mid-360 reports g and the
    driver passes it through; ~1.0 on z at rest);
  - header.frame_id the literal "livox_frame", unprefixed (hardcoded in
    the driver, ignoring its frame_id param); the axes are livox_frame's;
  - orientation and all covariances left at their defaults.
Stamped with this node's sim-time clock.

The base's own motion is added here: base_pose_publisher.py teleports
the base, which gives its links no velocity, so Gazebo's IMU only ever
senses gravity, noise, bias and the arm shaking the chassis (a 0.3 rad/s
spin read at most 0.006 rad/s). From base_pose_publisher.py's
'ground_truth/odom' (body twist of base_link, planar), this adds the
rigid-body angular velocity and the acceleration at the IMU's position
(base acceleration + angular-acceleration and centripetal terms), both
rotated into the IMU's axes.

Usage: gz_livox_imu.py <gz_topic> <ros_topic>
Parameters: base_frame, imu_frame (the TF frame the sensor sits on),
ground_truth_topic.
"""
import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import quaternion_matrix

from gz.msgs10 import imu_pb2
from gz.transport13 import Node as GzNode

STANDARD_GRAVITY = 9.80665  # [m/s^2]
DRIVER_FRAME_ID = 'livox_frame'


class GzLivoxImu(Node):
    def __init__(self, gz_topic, ros_topic):
        super().__init__('gz_livox_imu')
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.imu_frame = self.declare_parameter('imu_frame', 'livox_imu_frame').value
        ground_truth_topic = self.declare_parameter('ground_truth_topic', 'ground_truth/odom').value

        self.lock = threading.Lock()
        # IMU extrinsic in base_link: rotation (columns = IMU axes) and position.
        self.rot = None
        self.pos = None
        # Base motion, base_link frame: angular velocity, and linear
        # acceleration of base_link's origin / angular acceleration.
        self.omega = np.zeros(3)
        self.accel = np.zeros(3)
        self.alpha = np.zeros(3)
        self._last_twist = None  # (stamp [s], v, omega)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._tf_timer = self.create_timer(0.5, self._lookup_extrinsic)
        self.create_subscription(Odometry, ground_truth_topic, self._ground_truth_cb, 10)

        self.pub = self.create_publisher(Imu, ros_topic, 100)
        self._gz_node = GzNode()
        self._gz_node.subscribe(imu_pb2.IMU, gz_topic, self._cb)

    def _lookup_extrinsic(self):
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.imu_frame, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f'Waiting for {self.base_frame} -> {self.imu_frame}: {e}',
                                   throttle_duration_sec=5.0)
            return
        q = t.transform.rotation
        p = t.transform.translation
        with self.lock:
            self.rot = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
            self.pos = np.array([p.x, p.y, p.z])
        self._tf_timer.cancel()

    def _ground_truth_cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        tw = msg.twist.twist
        v = np.array([tw.linear.x, tw.linear.y, tw.linear.z])
        omega = np.array([tw.angular.x, tw.angular.y, tw.angular.z])
        with self.lock:
            if self._last_twist is not None and stamp > self._last_twist[0]:
                dt = stamp - self._last_twist[0]
                # Acceleration of a point fixed in a rotating body frame,
                # expressed in that frame: dv/dt + omega x v.
                self.accel = (v - self._last_twist[1]) / dt + np.cross(omega, v)
                self.alpha = (omega - self._last_twist[2]) / dt
            self.omega = omega
            self._last_twist = (stamp, v, omega)

    def _cb(self, msg):
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        with self.lock:
            if self.rot is not None:
                w, r = self.omega, self.pos
                accel_at_imu = self.accel + np.cross(self.alpha, r) + np.cross(w, np.cross(w, r))
                gyro = gyro + self.rot.T @ w
                acc = acc + self.rot.T @ accel_at_imu

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = DRIVER_FRAME_ID
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gyro.tolist()
        acc_g = acc / STANDARD_GRAVITY
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = acc_g.tolist()
        self.pub.publish(imu)


def main():
    rclpy.init()
    node = GzLivoxImu(gz_topic=sys.argv[1], ros_topic=sys.argv[2])
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
