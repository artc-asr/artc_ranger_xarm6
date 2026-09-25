#!/usr/bin/env python3
"""Simulated Mid-360 built-in IMU, published the way livox_ros_driver2 does.

Computed from the base's ground-truth motion ('ground_truth/odom' from
base_pose_publisher.py), not by a Gazebo IMU sensor. The base is
kinematic (teleported), so the Gazebo chassis's own physics motion isn't
the robot's: between teleports it creeps (settling on its passive wheels,
the steering joints relaxing), and the teleport that snaps it back carries
no velocity. A Gazebo IMU senses only the creep, a one-sided rotation the
real robot doesn't have (-0.012 rad/s measured during a straight crab,
enough to turn the EKF's heading 2.6 deg in 6 s). What's lost: the arm's
reaction shaking the chassis.

At 'rate' (sim time), in the IMU's axes (imu_frame, looked up in TF from
base_frame):
  - angular velocity: the base's (planar: z only in base_link);
  - specific force: gravity's reaction (the base stays level) plus the
    acceleration at the IMU's position (base acceleration from
    differentiating the ground-truth twist, plus angular-acceleration and
    centripetal terms);
  - each with white noise and a constant turn-on bias drawn from 'seed'
    (the same every run unless seed changes; ICM-40609-D datasheet noise over the 100 Hz band: gyro 0.0045
    deg/s/rtHz -> 7.9e-4 rad/s, accel 100 ug/rtHz -> 9.8e-3 m/s^2; the
    biases are a moderate guess for a factory-calibrated unit).
Published on 'livox/imu' exactly as the real driver's Lddc::InitImuMsg
fills it in: linear_acceleration in g, header.frame_id the literal
unprefixed "livox_frame", orientation and covariances left at defaults.
"""
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import quaternion_matrix

STANDARD_GRAVITY = 9.80665  # [m/s^2]
DRIVER_FRAME_ID = 'livox_frame'


class SimLivoxImu(Node):
    def __init__(self):
        super().__init__('sim_livox_imu')
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.imu_frame = self.declare_parameter('imu_frame', 'livox_imu_frame').value
        ground_truth_topic = self.declare_parameter('ground_truth_topic', 'ground_truth/odom').value
        imu_topic = self.declare_parameter('imu_topic', 'livox/imu').value
        rate = self.declare_parameter('rate', 200.0).value                  # [Hz], the Mid-360's
        self.gyro_noise = self.declare_parameter('gyro_noise', 7.9e-4).value     # [rad/s] stddev
        self.accel_noise = self.declare_parameter('accel_noise', 9.8e-3).value   # [m/s^2] stddev
        gyro_bias = self.declare_parameter('gyro_bias_stddev', 1e-3).value       # [rad/s]
        accel_bias = self.declare_parameter('accel_bias_stddev', 0.02).value     # [m/s^2]
        seed = self.declare_parameter('seed', 0).value

        self.rng = np.random.default_rng(seed)
        self.gyro_bias = self.rng.normal(0.0, gyro_bias, 3)
        self.accel_bias = self.rng.normal(0.0, accel_bias, 3)

        self.lock = threading.Lock()
        self.rot = None  # IMU axes in base_link (columns)
        self.pos = None  # IMU position in base_link
        # Base motion, base_link frame: angular velocity, linear
        # acceleration of base_link's origin, angular acceleration.
        self.omega = np.zeros(3)
        self.accel = np.zeros(3)
        self.alpha = np.zeros(3)
        self._last_twist = None  # (stamp [s], v, omega)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._tf_timer = self.create_timer(0.5, self._lookup_extrinsic)
        self.create_subscription(Odometry, ground_truth_topic, self._ground_truth_cb, 10)
        self.pub = self.create_publisher(Imu, imu_topic, 100)
        self.create_timer(1.0 / rate, self._publish)

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

    def _publish(self):
        with self.lock:
            if self.rot is None or self._last_twist is None:
                return
            w, r, rot = self.omega, self.pos, self.rot
            accel_at_imu = self.accel + np.cross(self.alpha, r) + np.cross(w, np.cross(w, r))
        specific_force = accel_at_imu + np.array([0.0, 0.0, STANDARD_GRAVITY])
        gyro = rot.T @ w + self.gyro_bias + self.rng.normal(0.0, self.gyro_noise, 3)
        acc = rot.T @ specific_force + self.accel_bias + self.rng.normal(0.0, self.accel_noise, 3)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = DRIVER_FRAME_ID
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gyro.tolist()
        acc_g = acc / STANDARD_GRAVITY
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = acc_g.tolist()
        self.pub.publish(imu)


def main():
    rclpy.init()
    node = SimLivoxImu()
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
