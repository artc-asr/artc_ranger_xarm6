#!/usr/bin/env python3
"""Save the robot's current pose as a named waypoint (config/waypoints.yaml).

    ros2 run ranger_xarm6_tasks save_waypoint.py table_ne_pick
    ros2 run ranger_xarm6_tasks save_waypoint.py table_ne_pick --yaw-deg -90   # override the heading

Drive the robot there first (teleop, Nav2, MoveIt). The pose is base_link
in the map frame (Nav2's goal frame). An existing waypoint of that name is
replaced; other lines and comments are kept. Used by the task nodes
NavigateToWaypoint, BaseToPose, TurnBase (back_towards) and
WholeBodyToPose (base_waypoint) from the next task run on.
"""
import argparse
import math
import os
import re
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener


def tasks_dir(arg):
    if arg:
        return os.path.expanduser(arg)
    share = get_package_share_directory('ranger_xarm6_tasks')
    with open(os.path.join(share, 'source_dir.txt')) as f:
        return f.read().strip()


def lookup(frame, child, timeout=5.0):
    rclpy.init()
    node = rclpy.create_node('save_pose_cli')
    buf = Buffer()
    TransformListener(buf, node)
    end = time.time() + timeout
    try:
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if buf.can_transform(frame, child, rclpy.time.Time()):
                return buf.lookup_transform(frame, child, rclpy.time.Time()).transform
        raise SystemExit(f'no TF {frame} -> {child}: is the robot (and, for the map frame, Nav2) up?')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def upsert(path, name, line):
    """Replace the line starting with 'name:' or append; keep everything else."""
    lines = open(path).read().splitlines() if os.path.exists(path) else []
    pattern = re.compile(rf'^{re.escape(name)}\s*:')
    for i, old in enumerate(lines):
        if pattern.match(old):
            lines[i] = line
            break
    else:
        lines.append(line)
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('name')
    p.add_argument('--robot-id', default='robot_a')
    p.add_argument('--frame', help='default: <robot_id>_map')
    p.add_argument('--yaw-deg', type=float, help='store this heading instead of the current one')
    p.add_argument('--tasks-dir', help='default: the ranger_xarm6_tasks source folder')
    args = p.parse_args()
    prefix = f'{args.robot_id}_' if args.robot_id else ''
    frame = args.frame or f'{prefix}map'
    t = lookup(frame, f'{prefix}base_link')
    q = t.rotation
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    if args.yaw_deg is not None:
        yaw = args.yaw_deg
    line = (f'{args.name}: {{frame: {frame}, x: {t.translation.x:.3f}, y: {t.translation.y:.3f}, '
            f'yaw_deg: {yaw:.1f}}}')
    path = os.path.join(tasks_dir(args.tasks_dir), 'config', 'waypoints.yaml')
    upsert(path, args.name, line)
    print(f'{path}:\n  {line}')


if __name__ == '__main__':
    main()
