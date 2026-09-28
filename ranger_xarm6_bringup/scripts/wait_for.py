#!/usr/bin/env python3
"""Block until parts of the robot's stack are up, then exit 0 (1 on timeout).

sim.launch.py starts each stage when the one before is ready, by starting
the next stage when this exits. Every condition given must hold:

    wait_for.py --controllers /robot_a/controller_manager joint_state_broadcaster,arm_velocity_controller
    wait_for.py --lifecycle /robot_a/bt_navigator
    wait_for.py --action /robot_a/mobile_manipulation/move_to_goal
    wait_for.py --tf robot_a_odom robot_a_base_link
"""
import argparse
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.action import get_action_names_and_types
from tf2_ros import Buffer, TransformListener


def call(node, client, request, timeout=2.0):
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result() if future.done() else None


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--controllers', nargs=2, metavar=('MANAGER', 'NAMES'),
                   help='a controller_manager and comma-separated controllers that must be active')
    p.add_argument('--lifecycle', action='append', default=[], help='a lifecycle node that must be active')
    p.add_argument('--action', action='append', default=[], help='an action server that must exist')
    p.add_argument('--tf', nargs=2, metavar=('PARENT', 'CHILD'), help='a transform that must be available')
    p.add_argument('--timeout', type=float, default=300.0, help='[s] wall time')
    p.add_argument('--label', default='', help='printed with the result')
    args, _ = p.parse_known_args()

    rclpy.init()
    node = rclpy.create_node('wait_for')
    buffer = Buffer()
    TransformListener(buffer, node, spin_thread=False)
    controller_client = node.create_client(ListControllers, f'{args.controllers[0]}/list_controllers') \
        if args.controllers else None
    lifecycle_clients = {n: node.create_client(GetState, f'{n}/get_state') for n in args.lifecycle}

    def missing():
        out = []
        if controller_client:
            reply = call(node, controller_client, ListControllers.Request())
            active = {c.name for c in reply.controller if c.state == 'active'} if reply else set()
            out += [f'controller {c}' for c in args.controllers[1].split(',') if c not in active]
        for name, client in lifecycle_clients.items():
            reply = call(node, client, GetState.Request())
            if not reply or reply.current_state.id != State.PRIMARY_STATE_ACTIVE:
                out.append(f'{name} active')
        actions = {n for n, _ in get_action_names_and_types(node)}
        out += [f'action {a}' for a in args.action if a not in actions]
        if args.tf and not buffer.can_transform(args.tf[0], args.tf[1], rclpy.time.Time()):
            out.append(f'tf {args.tf[0]} -> {args.tf[1]}')
        return out

    start = time.monotonic()
    waiting = None
    while True:
        rclpy.spin_once(node, timeout_sec=0.5)
        waiting = missing()
        if not waiting:
            print(f'[wait_for] {args.label} ready after {time.monotonic() - start:.0f} s', flush=True)
            code = 0
            break
        if time.monotonic() - start > args.timeout:
            print(f'[wait_for] {args.label} NOT ready after {args.timeout:.0f} s: {", ".join(waiting)}', flush=True)
            code = 1
            break
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
