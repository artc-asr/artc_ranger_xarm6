#!/usr/bin/env python3
"""Run a task (a behavior tree in trees/) by name, list them, or cancel.

    ros2 run ranger_xarm6_tasks run_task.py --list
    ros2 run ranger_xarm6_tasks run_task.py PickCube
    (Ctrl-C while it runs cancels the task and halts the robot's current action)

The same as `ros2 action send_goal /robot_a/execute_task
btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: PickCube}"`, with
the result spelled out.
"""
import argparse
import signal
import sys
import time

import rclpy
from btcpp_ros2_interfaces.action import ExecuteTree
from btcpp_ros2_interfaces.srv import GetTrees
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('task', nargs='?')
    p.add_argument('--list', action='store_true', help='the tasks the server has loaded')
    p.add_argument('--robot-id', default='robot_a')
    args = p.parse_args()
    ns = f'/{args.robot_id}' if args.robot_id else ''
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl-C must still send the cancel
    node = rclpy.create_node('run_task_cli')
    try:
        if args.list or not args.task:
            cli = node.create_client(GetTrees, f'{ns}/get_loaded_trees')
            if not cli.wait_for_service(timeout_sec=5.0):
                sys.exit('no task server: ros2 launch ranger_xarm6_tasks tasks.launch.py')
            fut = cli.call_async(GetTrees.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
            print('Tasks:', ' '.join(sorted(t for t in fut.result().tree_ids if not t.startswith('_'))))
            return
        client = ActionClient(node, ExecuteTree, f'{ns}/execute_task')
        if not client.wait_for_server(timeout_sec=5.0):
            sys.exit('no task server: ros2 launch ranger_xarm6_tasks tasks.launch.py')
        # Discovery of the result/feedback readers can lag the server's (FastDDS).
        end = node.get_clock().now().nanoseconds + 2e9
        while node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        send = client.send_goal_async(ExecuteTree.Goal(target_tree=args.task),
                                      feedback_callback=lambda f: print('  ', f.feedback.message))
        rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            sys.exit(f"task '{args.task}' refused: unknown, or another task is running (see the task server's log)")
        print(f"task '{args.task}' running (Ctrl-C cancels)")
        # Ctrl-C only sets a flag: a KeyboardInterrupt inside rclpy's spin
        # lost the cancel request (the task ran on).
        stop = []
        signal.signal(signal.SIGINT, lambda *_: stop.append(True))
        result = handle.get_result_async()
        cancel_deadline = None
        while not result.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            if stop and cancel_deadline is None:
                print('cancelling...')
                handle.cancel_goal_async()
                cancel_deadline = time.monotonic() + 20.0
            if cancel_deadline and time.monotonic() > cancel_deadline:
                sys.exit('no answer to the cancel')
        r = result.result()
        if r is None:
            sys.exit('no result')
        ok = r.result.node_status.status == r.result.node_status.SUCCESS
        print(('SUCCESS' if ok else 'FAILED') + ':', r.result.return_message)
        sys.exit(0 if ok else 1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
