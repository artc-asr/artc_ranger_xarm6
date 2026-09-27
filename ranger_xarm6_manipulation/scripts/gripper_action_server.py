#!/usr/bin/env python3
"""GripperCommand action server for the G2 gripper: <ns>/gripper_command.

control_msgs/GripperCommand, for callers that need a result (the task
layer's Gripper node, ranger_xarm6_tasks):
  goal.command.position   drive_joint [rad]: 0 open .. 0.85 closed (not a gap
                          in metres as the message's comment says)
  goal.command.max_effort ignored (the controller is position-only)
  result.position         where drive_joint settled
  result.reached_goal     within tolerance of the target
  result.stalled          stopped short while closing: holding something

Sends the target to gripper_position_controller/commands (after the
controller is connected, and a few times: it subscribes best-effort),
then waits until drive_joint reaches it or stops moving for
settle_time, up to timeout. Cancelling stops the wait; the fingers keep
their last target.
"""
import threading
import time

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

OPEN, CLOSED = 0.0, 0.85


class GripperActionServer(Node):
    def __init__(self):
        super().__init__('gripper_action_server')
        self.tolerance = self.declare_parameter('tolerance', 0.02).value      # rad
        self.settle_time = self.declare_parameter('settle_time', 1.0).value   # s without motion = stopped
        self.timeout = self.declare_parameter('timeout', 6.0).value           # s
        self.joint = None
        self.lock = threading.Lock()
        group = ReentrantCallbackGroup()
        self.create_subscription(JointState, 'joint_states', self._on_joints, 10, callback_group=group)
        self.pub = self.create_publisher(Float64MultiArray, 'gripper_position_controller/commands', 10)
        self.busy = False
        ActionServer(self, GripperCommand, 'gripper_command', execute_callback=self._execute,
                     goal_callback=self._on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                     callback_group=group)
        self.get_logger().info('gripper_command ready (position: 0 open .. 0.85 closed, rad)')

    def _on_joints(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name.endswith('drive_joint'):
                with self.lock:
                    self.joint = pos

    def _position(self):
        with self.lock:
            return self.joint

    def _on_goal(self, goal):
        target = goal.command.position
        if self.busy or not OPEN - 1e-6 <= target <= CLOSED + 1e-6:
            self.get_logger().warn(f'rejected gripper goal {target:.3f} ({"busy" if self.busy else "out of 0..0.85"})')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        self.busy = True
        target = goal_handle.request.command.position
        result = GripperCommand.Result()
        try:
            end = time.monotonic() + self.timeout
            while (self.pub.get_subscription_count() == 0 or self._position() is None) and time.monotonic() < end:
                time.sleep(0.05)
            if self.pub.get_subscription_count() == 0 or self._position() is None:
                self.get_logger().error('no gripper_position_controller or no joint_states')
                goal_handle.abort()
                return result
            for _ in range(3):
                self.pub.publish(Float64MultiArray(data=[float(target)]))
                time.sleep(0.05)
            last, still_since = self._position(), time.monotonic()
            while time.monotonic() < end:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.position = self._position()
                    return result
                pos = self._position()
                if abs(pos - last) > 1e-3:
                    last, still_since = pos, time.monotonic()
                if abs(pos - target) < self.tolerance or time.monotonic() - still_since > self.settle_time:
                    break
                fb = GripperCommand.Feedback()
                fb.position = pos
                goal_handle.publish_feedback(fb)
                time.sleep(0.05)
            pos = self._position()
            result.position = pos
            result.reached_goal = abs(pos - target) < self.tolerance
            result.stalled = not result.reached_goal and target > pos
            self.get_logger().info(f'gripper -> {target:.2f}: at {pos:.3f}'
                                   + (' (stalled: holding something)' if result.stalled else ''))
            goal_handle.succeed()
            return result
        finally:
            self.busy = False


def main():
    rclpy.init()
    node = GripperActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
