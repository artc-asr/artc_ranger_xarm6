#!/usr/bin/env python3
"""FollowJointTrajectory server that drives the Ranger base through waypoints.

Action: <ns>/base_trajectory_controller/follow_joint_trajectory
(control_msgs/FollowJointTrajectory). Joint names are the SRDF planar
virtual joint's variables, matched by suffix: '<...>/x', '<...>/y',
'<...>/theta'. Waypoint positions are in the odom frame.

Tracks geometry, not timing (time_from_start is ignored): each waypoint is
reached by a straight translation, then a rotation to its theta. Every
velocity command it sends maps onto exactly ONE of the Ranger's steering
modes, as chosen by westonrobot_ranger_ros2's TwistCmdCallback:
  - translation: fixed body-frame direction, angular.z = 0. linear.y != 0
    -> parallel (crab) mode; a direction within straight_tolerance of
    straight ahead/back is sent with linear.y = 0 exactly (dual-Ackermann
    with zero steer), so the driver never flips modes mid-segment.
  - rotation: linear.x = linear.y = 0 -> spinning mode.
  - a stop of mode_switch_pause seconds between the two, so the real
    wheels can re-steer before moving.
Any lateral residual left after a translation (drift, sim/real slip) is
cleaned up by another translation pass, up to max_passes.

Feedback comes from TF (odom -> base_link): base_pose_publisher.py in
sim, the Ranger driver's odometry on hardware. Only one node should
publish cmd_vel at a time: stop wbc.py (or anything else driving the
base) while this runs.
"""
import math
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class BaseTrajectoryServer(Node):
    def __init__(self):
        super().__init__('base_trajectory_server')
        p = self.declare_parameter
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_link').value
        self.max_linear = p('max_linear_speed', 0.3).value         # m/s
        self.max_angular = p('max_angular_speed', 0.5).value       # rad/s
        self.linear_accel = p('linear_acceleration', 0.3).value    # m/s^2
        self.angular_accel = p('angular_acceleration', 0.8).value  # rad/s^2
        self.kp_linear = p('kp_linear', 1.5).value                 # 1/s
        self.kp_angular = p('kp_angular', 2.0).value               # 1/s
        self.xy_tolerance = p('xy_tolerance', 0.02).value          # m
        self.yaw_tolerance = p('yaw_tolerance', 0.02).value        # rad
        self.straight_tolerance = p('straight_tolerance', math.radians(2.0)).value
        self.mode_switch_pause = p('mode_switch_pause', 0.5).value  # s
        self.max_passes = p('max_passes', 3).value
        self.rate = p('control_rate', 20.0).value                  # Hz
        self.timeout_margin = p('timeout_margin', 3.0).value       # x nominal duration

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._server = ActionServer(
            self, FollowJointTrajectory, 'base_trajectory_controller/follow_joint_trajectory',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self._busy = False
        self.get_logger().info(
            f'Base trajectory server ready ({self.odom_frame} -> {self.base_frame}, '
            f'{self.max_linear} m/s, {self.max_angular} rad/s)')

    # ---- helpers -------------------------------------------------------
    def _pose(self):
        t = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def _send(self, vx=0.0, vy=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.cmd_pub.publish(msg)

    def _stop(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self._send()
            time.sleep(1.0 / self.rate)

    @staticmethod
    def _waypoints(trajectory):
        index = {}
        for i, name in enumerate(trajectory.joint_names):
            for axis in ('x', 'y', 'theta'):
                if name.endswith(f'/{axis}'):
                    index[axis] = i
        if set(index) != {'x', 'y', 'theta'}:
            return None
        return [(pt.positions[index['x']], pt.positions[index['y']], pt.positions[index['theta']])
                for pt in trajectory.points]

    # ---- action callbacks ------------------------------------------------
    def _on_goal(self, goal):
        if self._busy:
            self.get_logger().warn('Rejecting goal: already executing one')
            return GoalResponse.REJECT
        if not goal.trajectory.points or self._waypoints(goal.trajectory) is None:
            self.get_logger().warn("Rejecting goal: needs points and joints '*/x', '*/y', '*/theta'")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        self._busy = True
        result = FollowJointTrajectory.Result()
        try:
            waypoints = self._waypoints(goal_handle.request.trajectory)
            names = list(goal_handle.request.trajectory.joint_names)
            for i, wp in enumerate(waypoints):
                outcome = self._reach(goal_handle, wp, names)
                if outcome != 'ok':
                    result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    result.error_string = f'waypoint {i + 1}/{len(waypoints)}: {outcome}'
                    if outcome == 'canceled':
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                    return result
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
            return result
        except TransformException as e:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f'no {self.odom_frame} -> {self.base_frame} transform: {e}'
            goal_handle.abort()
            return result
        finally:
            self._stop(0.2)
            self._busy = False

    # ---- motion ------------------------------------------------------------
    def _reach(self, goal_handle, wp, names):
        """Translate then rotate to one waypoint. Returns 'ok', 'failed' or 'canceled'."""
        gx, gy, gyaw = wp
        for _ in range(self.max_passes):
            x, y, _yaw = self._pose()
            if math.hypot(gx - x, gy - y) <= self.xy_tolerance:
                break
            outcome = self._translate(goal_handle, gx, gy, gyaw, names)
            if outcome != 'ok':
                return outcome
            self._stop(self.mode_switch_pause)
        else:
            x, y, _yaw = self._pose()
            if math.hypot(gx - x, gy - y) > self.xy_tolerance:
                self.get_logger().warn(f'Position error {math.hypot(gx - x, gy - y):.3f} m after {self.max_passes} passes')
                return 'failed'

        _x, _y, yaw = self._pose()
        if abs(wrap(gyaw - yaw)) > self.yaw_tolerance:
            outcome = self._rotate(goal_handle, gx, gy, gyaw, names)
            if outcome != 'ok':
                return outcome
            self._stop(self.mode_switch_pause)
        return 'ok'

    def _translate(self, goal_handle, gx, gy, gyaw, names):
        x0, y0, yaw0 = self._pose()
        dist = math.hypot(gx - x0, gy - y0)
        # Fixed direction for the whole segment, in the body frame (heading
        # doesn't change while translating).
        heading = wrap(math.atan2(gy - y0, gx - x0) - yaw0)
        ux, uy = math.cos(heading), math.sin(heading)
        if abs(math.sin(heading)) < math.sin(self.straight_tolerance):
            ux, uy = math.copysign(1.0, ux), 0.0  # straight: keep linear.y exactly 0
        deadline = time.monotonic() + self.timeout_margin * (dist / self.max_linear + 2.0)
        speed = 0.0
        dt = 1.0 / self.rate
        while True:
            if goal_handle.is_cancel_requested:
                return 'canceled'
            if time.monotonic() > deadline:
                self.get_logger().warn('Translation timed out')
                return 'failed'
            x, y, yaw = self._pose()
            # Remaining distance along the segment's fixed direction.
            dir_x = math.cos(yaw0) * ux - math.sin(yaw0) * uy
            dir_y = math.sin(yaw0) * ux + math.cos(yaw0) * uy
            remaining = (gx - x) * dir_x + (gy - y) * dir_y
            if remaining <= self.xy_tolerance / 2:
                return 'ok'
            target = min(self.max_linear, self.kp_linear * remaining)
            speed = min(target, speed + self.linear_accel * dt)
            self._send(vx=speed * ux, vy=speed * uy)
            self._feedback(goal_handle, names, (gx, gy, gyaw), (x, y, yaw))
            time.sleep(dt)

    def _rotate(self, goal_handle, gx, gy, gyaw, names):
        _x, _y, yaw0 = self._pose()
        deadline = time.monotonic() + self.timeout_margin * (abs(wrap(gyaw - yaw0)) / self.max_angular + 2.0)
        rate = 0.0
        dt = 1.0 / self.rate
        while True:
            if goal_handle.is_cancel_requested:
                return 'canceled'
            if time.monotonic() > deadline:
                self.get_logger().warn('Rotation timed out')
                return 'failed'
            x, y, yaw = self._pose()
            err = wrap(gyaw - yaw)
            if abs(err) <= self.yaw_tolerance:
                return 'ok'
            target = min(self.max_angular, self.kp_angular * abs(err))
            rate = min(target, rate + self.angular_accel * dt)
            self._send(wz=math.copysign(rate, err))
            self._feedback(goal_handle, names, (gx, gy, gyaw), (x, y, yaw))
            time.sleep(dt)

    def _feedback(self, goal_handle, names, desired, actual):
        fb = FollowJointTrajectory.Feedback()
        fb.joint_names = names
        order = ['x' if n.endswith('/x') else 'y' if n.endswith('/y') else 'theta' for n in names]
        values = lambda p: [dict(zip(('x', 'y', 'theta'), p))[a] for a in order]
        fb.desired = JointTrajectoryPoint(positions=values(desired))
        fb.actual = JointTrajectoryPoint(positions=values(actual))
        fb.error = JointTrajectoryPoint(positions=[d - a for d, a in zip(values(desired), values(actual))])
        goal_handle.publish_feedback(fb)


def main():
    rclpy.init()
    node = BaseTrajectoryServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._send()
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
