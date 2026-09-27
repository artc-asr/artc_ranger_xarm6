#!/usr/bin/env python3
"""Open or close the G2 gripper, and report where it stopped.

    ros2 run ranger_xarm6_manipulation gripper.py open
    ros2 run ranger_xarm6_manipulation gripper.py close
    ros2 run ranger_xarm6_manipulation gripper.py 0.4      # drive_joint [rad], 0 open .. 0.85 closed

Publishes to <robot_id>/gripper_position_controller/commands once the
controller is connected (a bare `ros2 topic pub --once` can go out before
it is, and the controller subscribes best-effort), then waits until
drive_joint settles. Stopping short of 'close' means the fingers are
holding something.
"""
import argparse
import time

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

TARGETS = {'open': 0.0, 'close': 0.85}


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('target', help="'open', 'close', or drive_joint in rad (0..0.85)")
    p.add_argument('--robot-id', default='robot_a')
    p.add_argument('--timeout', type=float, default=5.0)
    args, _ = p.parse_known_args()
    target = TARGETS.get(args.target)
    if target is None:
        target = float(args.target)
    if not 0.0 <= target <= 0.85:
        p.error('target must be within 0 (open) .. 0.85 (closed)')
    ns = f'/{args.robot_id}' if args.robot_id else ''

    rclpy.init()
    node = rclpy.create_node('gripper_cli')
    joint = [None]

    def on_joints(msg):
        for name, pos in zip(msg.name, msg.position):
            if name.endswith('drive_joint'):
                joint[0] = pos

    node.create_subscription(JointState, f'{ns}/joint_states', on_joints, 10)
    pub = node.create_publisher(Float64MultiArray, f'{ns}/gripper_position_controller/commands', 10)
    end = time.time() + args.timeout
    while (pub.get_subscription_count() == 0 or joint[0] is None) and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if pub.get_subscription_count() == 0:
        raise SystemExit(f'no {ns}/gripper_position_controller: is the robot up?')
    for _ in range(3):  # best-effort subscriber; repeating a position target is harmless
        pub.publish(Float64MultiArray(data=[target]))
        rclpy.spin_once(node, timeout_sec=0.05)

    last, still_since = joint[0], time.time()
    end = time.time() + args.timeout
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if joint[0] is not None and abs(joint[0] - last) > 1e-3:
            last, still_since = joint[0], time.time()
        if abs(joint[0] - target) < 0.02 or time.time() - still_since > 1.0:
            break
    held = target > joint[0] + 0.05
    print(f'drive_joint {joint[0]:.3f} (target {target:.2f})' + ('  -> stopped short: holding something' if held else ''))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
