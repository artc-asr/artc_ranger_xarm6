#!/usr/bin/env python3
"""Save the gripper's current pose as a named arm pose (config/arm_poses.yaml).

    ros2 run ranger_xarm6_tasks save_arm_pose.py above_cube_4                  # in odom (a place in the room)
    ros2 run ranger_xarm6_tasks save_arm_pose.py holding_up --frame robot_a_base_link   # relative to the base

Move the arm there first (MoveIt: a pose goal, or RViz's MotionPlanning
drag + Execute). Stored is link_tcp (between the fingertips): position and
orientation. In odom it's a fixed place in the room; in base_link it's
relative to the base, wherever the base is. Used by ArmToPose and
WholeBodyToPose (pose), and TurnBase (back_towards) from the next task
run on. An existing pose of that name is replaced.
"""
import argparse
import os

from save_waypoint import lookup, tasks_dir, upsert


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('name')
    p.add_argument('--robot-id', default='robot_a')
    p.add_argument('--frame', help='default: <robot_id>_odom')
    p.add_argument('--tasks-dir', help='default: the ranger_xarm6_tasks source folder')
    args = p.parse_args()
    prefix = f'{args.robot_id}_' if args.robot_id else ''
    frame = args.frame or f'{prefix}odom'
    t = lookup(frame, f'{prefix}link_tcp')
    v, q = t.translation, t.rotation
    line = (f'{args.name}: {{frame: {frame}, position: [{v.x:.4f}, {v.y:.4f}, {v.z:.4f}], '
            f'orientation: [{q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f}]}}')
    path = os.path.join(tasks_dir(args.tasks_dir), 'config', 'arm_poses.yaml')
    upsert(path, args.name, line)
    print(f'{path}:\n  {line}')


if __name__ == '__main__':
    main()
