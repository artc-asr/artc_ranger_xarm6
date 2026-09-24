#!/usr/bin/env python3
"""FollowJointTrajectory server that drives the Ranger base along MoveIt plans.

Action: <ns>/base_trajectory_controller/follow_joint_trajectory
(control_msgs/FollowJointTrajectory), MoveIt's 'base_trajectory_controller'
(ranger_xarm6_moveit_config/config/moveit_controllers.yaml). Joints are
MoveIt's base joints (ranger_xarm6_moveit.urdf.xacro), matched by suffix:
'base_x_joint', 'base_y_joint' (odom frame, m), 'base_theta_joint'
(yaw, rad). A trajectory may carry any subset; missing ones are held
where the base was when it started.

Tracks the trajectory in time (so the base stays in step with the arm's
half of a whole-body plan, which MoveIt executes in parallel): each tick
commands the trajectory's velocity at that instant plus a proportional
correction toward its position. Every trajectory must be ONE of the
Ranger's steering modes, as chosen by westonrobot_ranger_ros2's
TwistCmdCallback, and is rejected otherwise:
  - translation (x/y move, yaw constant): body-frame linear.x/linear.y,
    angular.z = 0, linear.y never exactly 0 -> parallel (crab) mode the
    whole way, never dual-Ackermann.
  - rotation (yaw moves, x/y constant): linear = 0 -> spinning mode.
Heading drift during a translation can't be corrected (that would need
rotation) and position drift during a rotation likewise; the next
stage's plan starts from wherever the base really is.
After the last point it holds still for mode_switch_pause, so the real
wheels can re-steer before the next stage.

Feedback comes from TF (odom -> base_link): base_pose_publisher.py in
sim, the Ranger driver's odometry on hardware. Only one node should
publish cmd_vel at a time: stop wbc.py (or anything else driving the
base) while this runs.
"""
import bisect
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

AXES = ('x', 'y', 'theta')
SUFFIXES = {'x': 'base_x_joint', 'y': 'base_y_joint', 'theta': 'base_theta_joint'}


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


class BaseTrajectoryServer(Node):
    def __init__(self):
        super().__init__('base_trajectory_server')
        p = self.declare_parameter
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_link').value
        self.max_linear = p('max_linear_speed', 1.0).value         # m/s, command clamp
        self.max_angular = p('max_angular_speed', 0.78).value      # rad/s, command clamp
        self.kp_linear = p('kp_linear', 1.5).value                 # 1/s
        self.kp_angular = p('kp_angular', 2.0).value               # 1/s
        self.xy_tolerance = p('xy_tolerance', 0.02).value          # m, final
        self.yaw_tolerance = p('yaw_tolerance', 0.02).value        # rad, final
        self.path_xy_tolerance = p('path_xy_tolerance', 0.3).value    # m, while tracking
        self.path_yaw_tolerance = p('path_yaw_tolerance', 0.3).value  # rad, while tracking
        self.goal_time_tolerance = p('goal_time_tolerance', 3.0).value  # s after the last point
        self.mode_switch_pause = p('mode_switch_pause', 0.5).value  # s stopped at the end
        # Total motion below these counts as "not moving" when classifying
        # a trajectory as translation or rotation.
        self.still_xy = p('still_xy', 0.005).value                 # m
        self.still_yaw = p('still_yaw', 0.01).value                # rad
        self.rate = p('control_rate', 20.0).value                  # Hz

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
            f'<= {self.max_linear} m/s, <= {self.max_angular} rad/s)')

    # ---- helpers -------------------------------------------------------
    def _pose(self):
        t = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _send(self, vx=0.0, vy=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.cmd_pub.publish(msg)

    def _stop(self, duration):
        end = time.monotonic() + duration
        while True:
            self._send()
            if time.monotonic() >= end:
                return
            time.sleep(1.0 / self.rate)

    @staticmethod
    def _axis_index(joint_names):
        index = {}
        for i, name in enumerate(joint_names):
            for axis, suffix in SUFFIXES.items():
                if name.endswith(suffix):
                    index[axis] = i
        return index

    def _classify(self, trajectory):
        """'translate', 'rotate' or 'still', or raise ValueError if it does both."""
        index = self._axis_index(trajectory.joint_names)
        travel = rotation = 0.0
        for a, b in zip(trajectory.points, trajectory.points[1:]):
            d = {axis: b.positions[j] - a.positions[j] for axis, j in index.items()}
            travel += math.hypot(d.get('x', 0.0), d.get('y', 0.0))
            rotation += abs(wrap(d.get('theta', 0.0)))
        translates, rotates = travel > self.still_xy, rotation > self.still_yaw
        if translates and rotates:
            raise ValueError(f'translates {travel:.3f} m and rotates {rotation:.3f} rad; '
                             'the Ranger does one or the other per trajectory')
        return 'translate' if translates else 'rotate' if rotates else 'still'

    # ---- action callbacks ------------------------------------------------
    def _on_goal(self, goal):
        traj = goal.trajectory
        if self._busy:
            self.get_logger().warn('Rejecting goal: already executing one')
            return GoalResponse.REJECT
        index = self._axis_index(traj.joint_names)
        if not traj.points or not index or len(index) != len(traj.joint_names):
            self.get_logger().warn(f'Rejecting goal: needs points and only joints ending in {sorted(SUFFIXES.values())}, '
                                   f'got {list(traj.joint_names)}')
            return GoalResponse.REJECT
        try:
            self._classify(traj)
        except ValueError as e:
            self.get_logger().warn(f'Rejecting goal: {e}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        self._busy = True
        result = FollowJointTrajectory.Result()
        traj = goal_handle.request.trajectory
        try:
            mode = self._classify(traj)
            start_pose = self._pose()
            outcome = self._track(goal_handle, traj, mode, start_pose)
            if outcome == 'ok':
                self._stop(self.mode_switch_pause)
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                goal_handle.succeed()
            else:
                code, message = outcome
                result.error_code = code
                result.error_string = message
                self.get_logger().warn(message)
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
            return result
        except TransformException as e:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f'no {self.odom_frame} -> {self.base_frame} transform: {e}'
            goal_handle.abort()
            return result
        finally:
            self._stop(0.1)
            self._busy = False

    # ---- tracking ------------------------------------------------------------
    def _sampler(self, traj, start_pose):
        """(t -> desired (x, y, yaw), velocity (vx, vy, wz)) and the trajectory's duration."""
        index = self._axis_index(traj.joint_names)
        times = [seconds(pt.time_from_start) for pt in traj.points]
        held = dict(zip(AXES, start_pose))

        def value(pt, axis, field):
            if axis not in index:
                return held[axis] if field == 'positions' else 0.0
            values = getattr(pt, field)
            return values[index[axis]] if len(values) > index[axis] else None

        def sample(t):
            i = bisect.bisect_right(times, t)
            if i == 0:
                i = 1
            if i >= len(times):
                last = traj.points[-1]
                return tuple(value(last, a, 'positions') for a in AXES), (0.0, 0.0, 0.0)
            a, b = traj.points[i - 1], traj.points[i]
            span = max(times[i] - times[i - 1], 1e-9)
            s = min(max((t - times[i - 1]) / span, 0.0), 1.0)
            pos, vel = [], []
            for axis in AXES:
                pa, pb = value(a, axis, 'positions'), value(b, axis, 'positions')
                delta = wrap(pb - pa) if axis == 'theta' else pb - pa
                pos.append(pa + s * delta)
                va, vb = value(a, axis, 'velocities'), value(b, axis, 'velocities')
                vel.append(delta / span if va is None or vb is None else va + s * (vb - va))
            return tuple(pos), tuple(vel)

        return sample, times[-1]

    def _track(self, goal_handle, traj, mode, start_pose):
        """Returns 'ok' or (error_code, message)."""
        R = FollowJointTrajectory.Result
        sample, duration = self._sampler(traj, start_pose)
        names = list(traj.joint_names)
        dt = 1.0 / self.rate
        t0 = self._now()
        while True:
            if goal_handle.is_cancel_requested:
                return R.PATH_TOLERANCE_VIOLATED, 'canceled'
            t = self._now() - t0
            (dx, dy, dyaw), (vx, vy, wz) = sample(t)
            x, y, yaw = self._pose()
            ex, ey, eyaw = dx - x, dy - y, wrap(dyaw - yaw)
            self._feedback(goal_handle, names, (dx, dy, dyaw), (x, y, yaw))

            if t >= duration:
                # Only the axis this mode can correct: a translation can't
                # fix heading drift, nor a rotation position drift.
                xy_ok = mode != 'translate' or math.hypot(ex, ey) <= self.xy_tolerance
                yaw_ok = mode != 'rotate' or abs(eyaw) <= self.yaw_tolerance
                if xy_ok and yaw_ok:
                    return 'ok'
                if t > duration + self.goal_time_tolerance:
                    return (R.GOAL_TOLERANCE_VIOLATED,
                            f'not at the goal {self.goal_time_tolerance}s after the trajectory ended '
                            f'(error {math.hypot(ex, ey):.3f} m, {eyaw:.3f} rad)')
            elif mode == 'translate' and math.hypot(ex, ey) > self.path_xy_tolerance:
                return (R.PATH_TOLERANCE_VIOLATED, f'{math.hypot(ex, ey):.3f} m off the path at t={t:.2f}s '
                        f'(want x={dx:.3f} y={dy:.3f}, at x={x:.3f} y={y:.3f})')
            elif mode == 'rotate' and abs(eyaw) > self.path_yaw_tolerance:
                return (R.PATH_TOLERANCE_VIOLATED, f'{eyaw:.3f} rad off the path at t={t:.2f}s '
                        f'(want {dyaw:.3f}, at {yaw:.3f})')

            if mode == 'translate':
                wx, wy = vx + self.kp_linear * ex, vy + self.kp_linear * ey
                speed = math.hypot(wx, wy)
                if speed > self.max_linear:
                    wx, wy = wx * self.max_linear / speed, wy * self.max_linear / speed
                bx = math.cos(yaw) * wx + math.sin(yaw) * wy
                by = -math.sin(yaw) * wx + math.cos(yaw) * wy
                # linear.y == 0 exactly would drop the driver into
                # dual-Ackermann mode; stay in parallel mode throughout.
                self._send(vx=bx, vy=by if by != 0.0 else 1e-9)
            elif mode == 'rotate':
                w = wz + self.kp_angular * eyaw
                self._send(wz=max(-self.max_angular, min(self.max_angular, w)))
            else:
                self._send()
            time.sleep(dt)

    def _feedback(self, goal_handle, names, desired, actual):
        index = {axis: i for axis, i in zip(AXES, range(3))}
        order = [next(a for a, s in SUFFIXES.items() if n.endswith(s)) for n in names]
        values = lambda p: [p[index[a]] for a in order]
        fb = FollowJointTrajectory.Feedback()
        fb.joint_names = names
        fb.desired = JointTrajectoryPoint(positions=values(desired))
        fb.actual = JointTrajectoryPoint(positions=values(actual))
        fb.error = JointTrajectoryPoint(positions=[
            wrap(d - a) if o == 'theta' else d - a for d, a, o in zip(values(desired), values(actual), order)])
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
