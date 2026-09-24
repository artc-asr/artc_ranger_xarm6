#!/usr/bin/env python3
"""MoveIt + base executor + mobile manipulation coordinator.

Run after ranger_xarm6_description's bringup (sim or real), same robot_id:

    ros2 launch ranger_xarm6_description ranger_xarm6.launch.py run_rviz:=false
    ros2 launch ranger_xarm6_manipulation manipulation.launch.py

Then send goals to /<robot_id>/mobile_manipulation/move_to_goal (see
README.md for examples). Stop wbc.py first: this switches the arm to
arm_trajectory_controller and publishes cmd_vel for the base.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1', 'yes')
    prefix = f'{robot_id}_' if robot_id else ''
    share = get_package_share_directory('ranger_xarm6_manipulation')

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ranger_xarm6_moveit_config'), 'launch', 'move_group.launch.py')),
        launch_arguments={
            'robot_id': robot_id,
            'use_sim_time': str(use_sim_time).lower(),
            'use_rviz': LaunchConfiguration('use_rviz').perform(context),
        }.items(),
    )
    base_server = Node(
        package='ranger_xarm6_manipulation',
        executable='base_trajectory_server.py',
        namespace=robot_id,
        parameters=[
            os.path.join(share, 'config', 'base_trajectory_server.yaml'),
            {'odom_frame': f'{prefix}odom', 'base_frame': f'{prefix}base_link', 'use_sim_time': use_sim_time},
        ],
        output='screen',
    )
    coordinator = Node(
        package='ranger_xarm6_manipulation',
        executable='mobile_manipulation_coordinator.py',
        namespace=robot_id,
        parameters=[{'prefix': prefix, 'use_sim_time': use_sim_time}],
        output='screen',
    )
    return [move_group, base_server, coordinator]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot_a', description='ROS namespace + joint/frame prefix; must match ranger_xarm6.launch.py'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='true with Gazebo, false on real hardware'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='RViz with the MotionPlanning panel'),
        OpaqueFunction(function=launch_setup),
    ])
