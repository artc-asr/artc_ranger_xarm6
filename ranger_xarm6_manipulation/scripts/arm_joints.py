#!/usr/bin/env python3
"""Print the arm's current joints, ready to paste as a new named pose in the SRDF.

    ros2 run ranger_xarm6_manipulation arm_joints.py my_pose

Move the arm where you want it first (a MoveToGoal pose goal, or RViz's
MotionPlanning panel), then paste the output into
ranger_xarm6_moveit_config/srdf/ranger_xarm6.srdf.xacro next to "home" and
"stow", rebuild ranger_xarm6_moveit_config and restart control.launch.py:
'my_pose' then works as arm_named_goal / transit_state.
"""
import argparse
import time

import rclpy
from sensor_msgs.msg import JointState


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('name', nargs='?', default='my_pose', help='name for the new pose')
    p.add_argument('--robot-id', default='robot_a')
    args, _ = p.parse_known_args()
    name, robot_id = args.name, args.robot_id
    prefix = f'{robot_id}_' if robot_id else ''
    rclpy.init()
    node = rclpy.create_node('arm_joints_cli')
    joints = {}

    def on_joints(msg):
        for n, pos in zip(msg.name, msg.position):
            for i in range(1, 7):
                if n == f'{prefix}joint{i}':
                    joints[i] = pos

    node.create_subscription(JointState, (f'/{robot_id}' if robot_id else '') + '/joint_states', on_joints, 10)
    end = time.time() + 5.0
    while len(joints) < 6 and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if len(joints) < 6:
        raise SystemExit('no arm joints on joint_states: is the robot up?')
    print(f'  <group_state name="{name}" group="arm">')
    for i in range(1, 7):
        print(f'    <joint name="${{prefix}}joint{i}" value="{joints[i]:.4f}" />')
    print('  </group_state>')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
