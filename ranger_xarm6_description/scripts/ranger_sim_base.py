#!/usr/bin/env python3
"""Simulated Ranger Mini 3.0 base, driven by physics (sim_base_drive physics).

Stands in for the real base driver (westonrobot_ranger_ros2's ranger_base)
and for Gazebo's view of where the robot is:

1. cmd_vel -> wheels. The same motion mode the real driver picks
   (RangerROSMessenger::TwistCmdCallback), and its limits:
     - linear.y != 0: parallel (crab), every wheel at atan(vy/vx),
       clamped to max_steer_angle_parallel;
     - else turn radius |vx|/|wz| < min_turn_radius: spinning in place,
       vx ignored;
     - else dual Ackermann, turning about a point on the lateral axis
       through the base's centre.
   The mode's body twist is turned into each wheel's steering angle and
   speed from where the wheel is (TF, base_link -> *_steering_wheel_link;
   the rear-right one isn't where the others' symmetry puts it), for
   steering_position_controller and wheel_velocity_controller. A wheel
   picks the nearer of its two equivalent angles (flipping its speed). A
   big re-steer (any wheel more than resteer_angle off, e.g. a mode
   change) first brakes to a stop on the old angles, then turns the wheels
   standing still, like the real base; smaller ones steer while driving,
   at speed * cos(error). Wheel speeds change at most max_wheel_accel.
   Driving on through a big re-steer moved the base along the wheels' old
   heading (a crab went diagonally); stopping the wheels at once locked
   them and the base skidded on (out of a spin, turning with the wheels
   reading zero); stopping for every small re-aim left MoveIt's docking
   corrections (a few cm, in changing directions) stop-starting short of
   the dock.
2. Wheel odometry on 'odom', as the real driver publishes it (frame odom,
   child base_link, twist in base_link, no TF), from the wheels' measured
   state: the body twist that best fits the four wheel velocities (least
   squares), integrated. So it carries what the wheels really did, slip
   and steering lag included, plus the same fixed scale error and white
   noise base_pose_publisher.py adds (a wheel radius that's not quite
   right).
3. Ground truth from Gazebo (its OdometryPublisher system, see
   ranger_xarm6.urdf.xacro): the odom -> base_link TF (publish_tf; off
   when the EKF owns it) and 'ground_truth/odom' (sim_livox_imu.py).
   odom is Gazebo's world frame, as with base_pose_publisher.py.

'set_base_pose' (Pose2D) teleports the model there and restarts the wheel
odometry from it, as base_pose_publisher.py's does.
"""
import math
import random
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from tf_transformations import quaternion_from_euler

from gz.transport13 import Node as GzNode
from gz.msgs10 import boolean_pb2, odometry_pb2, pose_pb2

CORNERS = ('fl', 'fr', 'rl', 'rr')  # the controllers' joint order


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RangerSimBase(Node):
    def __init__(self):
        super().__init__('ranger_sim_base')
        p = self.declare_parameter
        self.prefix = p('joint_prefix', '').value
        self.frame_id = p('frame_id', 'odom').value
        self.child_frame_id = p('child_frame_id', 'base_link').value
        self.publish_tf = p('publish_tf', True).value
        self.gz_world = p('gz_world', 'default').value
        self.gz_entity_name = p('gz_entity_name', 'robot_a').value
        self.rate = p('rate', 50.0).value                       # [Hz] wheel commands + odometry
        self.cmd_vel_timeout = p('cmd_vel_timeout', 0.5).value  # [s] stale cmd_vel -> stop
        self.wheel_radius = p('wheel_radius', 0.09).value       # [m]
        self.steer_limit = p('steering_limit', 2.1).value       # [rad], the joints' URDF limit
        self.resteer_angle = p('resteer_angle', 0.5).value      # [rad] more: stop, then steer
        self.max_wheel_accel = p('max_wheel_accel', 1.0).value  # [m/s^2] at the tyre
        self.stop_speed = p('stop_speed', 0.01).value            # [m/s] slow enough to re-steer
        # The real driver's limits (ranger_params.hpp: Ranger Mini V3 uses
        # RangerMiniV2Params).
        self.max_linear = p('max_linear_speed', 1.5).value
        self.max_angular = p('max_angular_speed', 4.8).value
        self.max_parallel = p('max_steer_angle_parallel', 1.570).value
        self.min_turn_radius = p('min_turn_radius', 0.4764).value
        # Wheel odometry error, as base_pose_publisher.py's.
        self.odom_topic = p('odom_topic', 'odom').value
        self.odom_linear_scale = p('odom_linear_scale', 1.02).value
        self.odom_angular_scale = p('odom_angular_scale', 0.97).value
        self.odom_linear_noise = p('odom_linear_noise', 0.01).value    # [m/s] stddev
        self.odom_angular_noise = p('odom_angular_noise', 0.01).value  # [rad/s] stddev
        self.wheel_deadband = p('wheel_deadband', 0.002).value         # [m/s] slower reads as still
        self._rng = random.Random(p('odom_noise_seed', 0).value)
        ground_truth_topic = p('ground_truth_odom_topic', 'ground_truth/odom').value

        self.lock = threading.Lock()
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = None
        self.steering = {}  # joint name -> measured position
        self.wheels = {}    # joint name -> measured velocity
        self.targets = None  # last steering targets (joint angles), held while stopped
        self.wheel_speed = [0.0] * 4  # last commanded tyre speeds [m/s]
        self.odom_pose = [p('x', 0.0).value, p('y', 0.0).value, p('yaw', 0.0).value]
        self.last_odom_time = None
        self.wheel_xy = None  # (4, 2) wheel positions in base_link, from TF
        self.fit = None       # least squares: body twist from the 8 wheel velocity components

        self.steer_pub = self.create_publisher(Float64MultiArray, 'steering_position_controller/commands', 10)
        self.wheel_pub = self.create_publisher(Float64MultiArray, 'wheel_velocity_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.ground_truth_pub = self.create_publisher(Odometry, ground_truth_topic, 10)
        self.broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(JointState, 'joint_states', self._joint_states_cb, 10)
        self.create_subscription(Pose2D, 'set_base_pose', self._set_base_pose_cb, 10)
        self.create_timer(1.0 / self.rate, self._tick)

        self.gz_node = GzNode()
        self.gz_node.subscribe(odometry_pb2.Odometry, f'/model/{self.gz_entity_name}/odometry', self._gz_odometry_cb)
        self.get_logger().info(
            f'Ranger sim base ready: cmd_vel -> steering_position_controller + wheel_velocity_controller, '
            f'wheel odometry on {self.odom_topic}, ground truth from gz entity "{self.gz_entity_name}"')

    def _joint(self, corner, kind):
        return f'{self.prefix}{corner}_{"steering_joint" if kind == "steering" else "wheel"}'

    def _cmd_vel_cb(self, msg):
        with self.lock:
            self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
            self.cmd_time = self.get_clock().now()

    def _joint_states_cb(self, msg):
        with self.lock:
            for i, name in enumerate(msg.name):
                if name.endswith('_steering_joint') and i < len(msg.position):
                    self.steering[name] = msg.position[i]
                elif name.endswith('_wheel') and i < len(msg.velocity):
                    self.wheels[name] = msg.velocity[i]

    def _wheel_geometry(self):
        """Wheel positions in base_link (TF), once; the least-squares fit from them."""
        if self.wheel_xy is not None:
            return True
        xy = []
        for corner in CORNERS:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.child_frame_id, f'{self.prefix}{corner}_steering_wheel_link', rclpy.time.Time())
            except Exception:
                return False
            xy.append((t.transform.translation.x, t.transform.translation.y))
        self.wheel_xy = np.array(xy)
        # Rows: wheel i's velocity (x, y) = (vx - wz*y_i, vy + wz*x_i).
        a = np.zeros((8, 3))
        for i, (x, y) in enumerate(xy):
            a[2 * i] = (1.0, 0.0, -y)
            a[2 * i + 1] = (0.0, 1.0, x)
        self.fit = np.linalg.pinv(a)
        self.get_logger().info('wheels at ' + ', '.join(
            f'{c} ({x:.3f}, {y:.3f})' for c, (x, y) in zip(CORNERS, xy)))
        return True

    def _mode_twist(self, vx, vy, wz):
        """The body twist the real driver's motion mode for this cmd_vel produces."""
        clamp = lambda v, m: max(-m, min(m, v))  # noqa: E731
        if vy != 0.0:  # parallel
            steer = math.atan(vy / vx) if vx != 0.0 else math.copysign(math.pi / 2, vy)
            steer = clamp(steer, self.max_parallel)
            speed = clamp((1.0 if vx >= 0.0 else -1.0) * math.hypot(vx, vy), self.max_linear)
            return speed * math.cos(steer), speed * math.sin(steer), 0.0
        if vx == 0.0 and wz == 0.0:
            return 0.0, 0.0, 0.0
        radius = abs(vx) / abs(wz) if wz != 0.0 else math.inf
        if radius < self.min_turn_radius:  # spinning
            return 0.0, 0.0, clamp(wz, self.max_angular)
        v = clamp(vx, self.max_linear)  # dual Ackermann, same turning radius
        return v, 0.0, wz * (v / vx)

    def _tick(self):
        if not self._wheel_geometry():
            return
        now = self.get_clock().now()
        with self.lock:
            cmd = self.cmd
            if self.cmd_time is None or (now - self.cmd_time).nanoseconds * 1e-9 > self.cmd_vel_timeout:
                cmd = (0.0, 0.0, 0.0)
            measured = [self.steering.get(self._joint(c, 'steering')) for c in CORNERS]
            speeds = [self.wheels.get(self._joint(c, 'wheel')) for c in CORNERS]
        if None in measured or None in speeds:
            return
        self._command_wheels(self._mode_twist(*cmd), measured)
        self._publish_odometry(measured, speeds, now)

    def _command_wheels(self, twist, measured):
        vx, vy, wz = twist
        if self.targets is None:
            self.targets = list(measured)
        targets, wheel_speed = [], []
        for i, (x, y) in enumerate(self.wheel_xy):
            wvx, wvy = vx - wz * y, vy + wz * x
            speed = math.hypot(wvx, wvy)
            target = self.targets[i]
            if speed > 1e-4:
                # The steering joint turns about -z: joint angle = -heading.
                # Of the two ways to get this velocity (heading, or heading
                # + pi rolling backwards), the one nearer the current angle.
                q = -math.atan2(wvy, wvx)
                options = [(q, speed), (wrap(q + math.pi), -speed)]
                options = [o for o in options if abs(o[0]) <= self.steer_limit] or options
                target, speed = min(options, key=lambda o: abs(o[0] - measured[i]))
            else:
                speed = 0.0
            targets.append(target)
            wheel_speed.append(speed)
        errors = [abs(t - m) for t, m in zip(targets, measured)]
        if max(errors) > self.resteer_angle:
            wheel_speed = [0.0] * 4
            if max(abs(v) for v in self.wheel_speed) > self.stop_speed:
                targets = list(self.targets)  # still rolling: brake on the old angles first
        else:
            wheel_speed = [v * math.cos(e) for v, e in zip(wheel_speed, errors)]
        self.targets = targets
        step = self.max_wheel_accel / self.rate
        self.wheel_speed = [c + max(-step, min(step, g - c)) for c, g in zip(self.wheel_speed, wheel_speed)]
        steer_cmd = targets
        wheel_cmd = [v / self.wheel_radius for v in self.wheel_speed]
        self.steer_pub.publish(Float64MultiArray(data=steer_cmd))
        self.wheel_pub.publish(Float64MultiArray(data=wheel_cmd))

    def _publish_odometry(self, measured, speeds, now):
        """Wheel odometry from the wheels' measured steering and speed."""
        v = np.zeros(8)
        for i, (q, w) in enumerate(zip(measured, speeds)):
            speed = w * self.wheel_radius
            if abs(speed) < self.wheel_deadband:
                continue
            v[2 * i], v[2 * i + 1] = speed * math.cos(-q), speed * math.sin(-q)
        twist = self.fit @ v
        scales = (self.odom_linear_scale, self.odom_linear_scale, self.odom_angular_scale)
        noises = (self.odom_linear_noise, self.odom_linear_noise, self.odom_angular_noise)
        vx, vy, wz = (
            s * t + self._rng.gauss(0.0, n) if abs(t) > 1e-6 else 0.0
            for t, s, n in zip(twist, scales, noises))
        dt = 0.0 if self.last_odom_time is None else (now - self.last_odom_time).nanoseconds * 1e-9
        self.last_odom_time = now
        with self.lock:
            x, y, yaw = self.odom_pose
            c, s = math.cos(yaw), math.sin(yaw)
            x += (vx * c - vy * s) * dt
            y += (vx * s + vy * c) * dt
            yaw = wrap(yaw + wz * dt)
            self.odom_pose = [x, y, yaw]
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        q = quaternion_from_euler(0.0, 0.0, yaw)
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = q
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z = vx, vy, wz
        self.odom_pub.publish(msg)

    def _gz_odometry_cb(self, msg):
        """Gazebo's true pose and twist: ground-truth TF and odometry."""
        stamp = rclpy.time.Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nsec).to_msg()
        p, q = msg.pose.position, msg.pose.orientation
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.frame_id
            t.child_frame_id = self.child_frame_id
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
            t.transform.rotation.x, t.transform.rotation.y = q.x, q.y
            t.transform.rotation.z, t.transform.rotation.w = q.z, q.w
            self.broadcaster.sendTransform(t)
        gt = Odometry()
        gt.header.stamp = stamp
        gt.header.frame_id = self.frame_id
        gt.child_frame_id = self.child_frame_id
        gt.pose.pose.position.x, gt.pose.pose.position.y, gt.pose.pose.position.z = p.x, p.y, p.z
        gt.pose.pose.orientation.x, gt.pose.pose.orientation.y = q.x, q.y
        gt.pose.pose.orientation.z, gt.pose.pose.orientation.w = q.z, q.w
        lin, ang = msg.twist.linear, msg.twist.angular
        gt.twist.twist.linear.x, gt.twist.twist.linear.y, gt.twist.twist.linear.z = lin.x, lin.y, lin.z
        gt.twist.twist.angular.x, gt.twist.twist.angular.y, gt.twist.twist.angular.z = ang.x, ang.y, ang.z
        self.ground_truth_pub.publish(gt)

    def _set_base_pose_cb(self, msg):
        req = pose_pb2.Pose()
        req.name = self.gz_entity_name
        req.position.x, req.position.y = msg.x, msg.y
        req.position.z = 0.315
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, msg.theta)
        req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w = qx, qy, qz, qw
        try:
            ok, rep = self.gz_node.request(f'/world/{self.gz_world}/set_pose', req,
                                           pose_pb2.Pose, boolean_pb2.Boolean, 1000)
        except Exception as e:  # noqa: BLE001
            ok, rep = False, None
            self.get_logger().warn(f'gz set_pose failed: {e}')
        with self.lock:
            self.odom_pose = [msg.x, msg.y, msg.theta]
            self.cmd = (0.0, 0.0, 0.0)
        self.get_logger().info(f'Base pose set: x={msg.x} y={msg.y} yaw={msg.theta} '
                               f'({"ok" if ok and rep and rep.data else "gz refused"})')


def main():
    rclpy.init()
    node = RangerSimBase()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
