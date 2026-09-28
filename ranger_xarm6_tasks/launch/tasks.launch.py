#!/usr/bin/env python3
"""The task layer: task_server (behavior trees) + the gripper action server.

On top of the robot, Nav2 and MoveIt, each in its own terminal (see the
repo README's usage guide):

    ros2 launch ranger_xarm6_description ranger_xarm6.launch.py run_rviz:=false \\
        world:=artc_lab.world x:=0.94 y:=4.35 yaw:=-1.5708
    ros2 launch ranger_xarm6_navigation navigation.launch.py map:=~/ranger_xarm6_maps/artc_lab.yaml
    ros2 launch ranger_xarm6_manipulation control.launch.py
    ros2 launch ranger_xarm6_tasks tasks.launch.py

Then a task by name:

    ros2 run ranger_xarm6_tasks run_task.py --list
    ros2 run ranger_xarm6_tasks run_task.py DemoPickPlace

Groot2 to edit tasks, klein-bt to watch them run (ranger_xarm6_tasks/README.md).

tasks_dir: where trees/, config/ and the Groot2 project are; default the
package's source folder (recorded at build), so what Groot2 saves and the
teach tools write is what runs, under version control, without a rebuild.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_tasks_dir():
    share = get_package_share_directory('ranger_xarm6_tasks')
    try:
        with open(os.path.join(share, 'source_dir.txt')) as f:
            source = f.read().strip()
        if os.path.isdir(os.path.join(source, 'trees')):
            return source
    except OSError:
        pass
    return share


def launch_setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1', 'yes')
    tasks_dir = os.path.expanduser(LaunchConfiguration('tasks_dir').perform(context)) or default_tasks_dir()
    prefix = f'{robot_id}_' if robot_id else ''
    return [
        Node(
            package='ranger_xarm6_manipulation',
            executable='gripper_action_server.py',
            namespace=robot_id,
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='ranger_xarm6_tasks',
            executable='task_server',
            name='task_server',
            namespace=robot_id,
            parameters=[{
                'use_sim_time': use_sim_time,
                'action_name': 'execute_task',
                'tasks_dir': tasks_dir,
                'frame_prefix': prefix,
                'groot2_port': int(LaunchConfiguration('groot2_port').perform(context)),
                'tick_frequency': 20,
            }],
            output='screen',
            emulate_tty=True,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot_a', description='ROS namespace + frame prefix; must match the robot'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='true with Gazebo, false on real hardware'),
        DeclareLaunchArgument('tasks_dir', default_value='', description="trees/ + config/ folder; '' = the package source"),
        DeclareLaunchArgument('groot2_port', default_value='1667', description='Live view of a running task (Groot2 protocol): klein-bt connects here'),
        OpaqueFunction(function=launch_setup),
    ])
