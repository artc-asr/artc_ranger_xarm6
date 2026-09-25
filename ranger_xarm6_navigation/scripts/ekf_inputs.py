#!/usr/bin/env python3
"""Turn the raw driver messages into inputs robot_localization can fuse.

Both drivers publish messages the EKF can't use as they are (sim and real
alike: ranger_xarm6_description's sim publishes the same formats):

  livox/imu (livox_ros_driver2) -> ekf/livox_imu
    - linear_acceleration is in g: scaled to m/s^2;
    - frame_id is the literal, unprefixed "livox_frame": replaced with
      imu_frame (the chip's own frame, livox_frame's axes), so the EKF
      can look up where it sits;
    - covariances are all zero, which robot_localization replaces with a
      tiny value, i.e. near-certainty: set from parameters instead, and
      orientation_covariance[0] = -1 (REP-145: no orientation, the
      Mid-360's IMU has no magnetometer or attitude filter).
    - gyro bias: subtracted, estimated while the robot stands still
      (zero-velocity update): the wheel odometry reads exactly zero, so
      the gyro's mean is its bias. robot_localization doesn't estimate
      IMU biases, and the EKF trusts the gyro over the wheels for
      heading, so an uncorrected bias turns heading while parked or
      crabbing (~0.002 rad/s in sim, from the sim IMU's turn-on bias).
    - restamp: stamp with this node's clock instead. The Mid-360 stamps
      with its own clock unless it's time-synced (PTP/gPTP), so on real
      hardware without sync its stamps aren't ROS time.

  odom (ranger_base) -> ekf/wheel_odom
    - covariances are all zero: the twist's set from parameters. Only the
      twist is meant to be fused; the pose is left as it is.

Parameters are in config/ekf.yaml; frames are set by odometry.launch.py.
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

STANDARD_GRAVITY = 9.80665  # [m/s^2]


def _diagonal(variances):
    """Row-major covariance matrix with 'variances' on the diagonal."""
    n = len(variances)
    cov = [0.0] * (n * n)
    for i, v in enumerate(variances):
        cov[i * n + i] = float(v)
    return cov


class EkfInputs(Node):
    def __init__(self):
        super().__init__('ekf_inputs')
        self.imu_frame = self.declare_parameter('imu_frame', 'livox_imu_frame').value
        self.restamp = self.declare_parameter('imu_restamp', False).value
        gyro_var = self.declare_parameter('imu_angular_velocity_variance', 1e-4).value
        accel_var = self.declare_parameter('imu_linear_acceleration_variance', 1e-2).value
        self.imu_gyro_cov = _diagonal([gyro_var] * 3)
        self.imu_accel_cov = _diagonal([accel_var] * 3)
        # Twist order: vx, vy, vz, vroll, vpitch, vyaw. The Ranger never
        # moves in z/roll/pitch; those get a large variance (unused in 2D).
        lin_var = self.declare_parameter('wheel_linear_velocity_variance', 2.5e-3).value
        ang_var = self.declare_parameter('wheel_angular_velocity_variance', 1e-2).value
        self.wheel_twist_cov = _diagonal([lin_var, lin_var, 1e3, 1e3, 1e3, ang_var])
        # Zero-velocity gyro bias estimate: after the wheels have read
        # still (every twist component below still_velocity) for
        # still_settle_time, a running mean of the gyro with time constant
        # gyro_bias_time_constant.
        self.still_velocity = self.declare_parameter('still_velocity', 1e-3).value
        self.still_settle_time = self.declare_parameter('still_settle_time', 0.5).value
        self.bias_tau = self.declare_parameter('gyro_bias_time_constant', 2.0).value
        self.gyro_bias = [0.0, 0.0, 0.0]
        self._still_since = None   # wheel-odom stamp [s] the robot stopped at
        self._still = False
        self._last_imu_stamp = None

        self.imu_pub = self.create_publisher(Imu, 'ekf/livox_imu', 100)
        self.odom_pub = self.create_publisher(Odometry, 'ekf/wheel_odom', 10)
        self.create_subscription(Imu, 'livox/imu', self._imu_cb, 100)
        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)

    def _imu_cb(self, msg):
        if self.restamp:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        gyro = msg.angular_velocity
        raw = (gyro.x, gyro.y, gyro.z)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._still and self._last_imu_stamp is not None and stamp > self._last_imu_stamp:
            k = 1.0 - math.exp(-(stamp - self._last_imu_stamp) / self.bias_tau)
            self.gyro_bias = [b + k * (r - b) for b, r in zip(self.gyro_bias, raw)]
        self._last_imu_stamp = stamp
        gyro.x, gyro.y, gyro.z = (r - b for r, b in zip(raw, self.gyro_bias))
        acc = msg.linear_acceleration
        acc.x *= STANDARD_GRAVITY
        acc.y *= STANDARD_GRAVITY
        acc.z *= STANDARD_GRAVITY
        msg.orientation_covariance = [-1.0] + [0.0] * 8
        msg.angular_velocity_covariance = self.imu_gyro_cov
        msg.linear_acceleration_covariance = self.imu_accel_cov
        self.imu_pub.publish(msg)

    def _odom_cb(self, msg):
        tw = msg.twist.twist
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if max(abs(tw.linear.x), abs(tw.linear.y), abs(tw.angular.z)) < self.still_velocity:
            if self._still_since is None:
                self._still_since = stamp
            self._still = stamp - self._still_since >= self.still_settle_time
        else:
            self._still_since = None
            self._still = False
        msg.twist.covariance = self.wheel_twist_cov
        self.odom_pub.publish(msg)


def main():
    rclpy.init()
    node = EkfInputs()
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
