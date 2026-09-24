#!/usr/bin/env python3
"""Bring up one controller on top of the running robot.

The robot itself (sim or real) comes from ranger_xarm6_description, in
another terminal, with the same robot_id; its own RViz off, since the
MoveIt controllers bring one with the MotionPlanning panel:

    ros2 launch ranger_xarm6_description ranger_xarm6.launch.py run_rviz:=false \\
        world:=artc_lab.world x:=0.94 y:=4.35 yaw:=-1.5708
    ros2 launch ranger_xarm6_manipulation control.launch.py controller:=moveit_sequential
    ros2 launch ranger_xarm6_manipulation control.launch.py controller:=moveit_whole_body

List the choices with:

    ros2 launch ranger_xarm6_manipulation control.launch.py --show-args

controller:
  moveit_sequential  MoveIt move_group + base_trajectory_server.py +
                     mobile_manipulation_coordinator.py, SEQUENTIAL by
                     default (arm stows, base crabs, base spins, arm moves).
  moveit_whole_body  The same nodes, WHOLE_BODY by default (base crab and
                     arm in one plan). Either mode can still be asked for
                     per goal (MoveToGoal.mode); this picks mode 0's.

MoveIt's planning scene gets the Gazebo world's static models from the
robot bringup itself (ranger_xarm6_description's world_collision_objects.py),
plus a floor from the coordinator; on real hardware, only the floor.

Both MoveIt controllers take the arm off arm_velocity_controller (switching
in arm_trajectory_controller) and publish cmd_vel for the base: stop
wbc.py or anything else driving the robot first.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONTROLLERS = {
    'moveit_sequential': 'sequential',
    'moveit_whole_body': 'whole_body',
}


def launch_setup(context, *args, **kwargs):
    controller = LaunchConfiguration('controller').perform(context)
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
        parameters=[{
            'prefix': prefix,
            'default_mode': CONTROLLERS[controller],
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )
    return [move_group, base_server, coordinator]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='moveit_sequential', choices=list(CONTROLLERS),
                              description='moveit_sequential: arm stows, base crabs, base spins, arm moves; '
                                          'moveit_whole_body: base crab + arm in one plan'),
        DeclareLaunchArgument('robot_id', default_value='robot_a', description='ROS namespace + joint/frame prefix; must match ranger_xarm6.launch.py'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='true with Gazebo, false on real hardware'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='RViz with the MotionPlanning panel'),
        OpaqueFunction(function=launch_setup),
    ])
