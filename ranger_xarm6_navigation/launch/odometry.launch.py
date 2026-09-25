#!/usr/bin/env python3
"""odom -> base_link from the EKF (wheel odometry + Mid-360 IMU).

The robot (sim or real) comes from ranger_xarm6_description, in another
terminal, with its own odom -> base_link TF off, so the EKF is the only
publisher of it:

    ros2 launch ranger_xarm6_description ranger_xarm6.launch.py publish_odom_tf:=false \\
        world:=artc_lab.world x:=0.94 y:=4.35 yaw:=-1.5708
    ros2 launch ranger_xarm6_navigation odometry.launch.py x:=0.94 y:=4.35 yaw:=-1.5708

Topics (in the robot's namespace): in livox/imu, odom (the drivers', or
the sim's); out odometry/filtered, and ekf/livox_imu, ekf/wheel_odom (the
EKF's inputs, see scripts/ekf_inputs.py). In sim, ground_truth/odom is
the truth to compare odometry/filtered against.

x/y/yaw: where odometry starts. In sim, odom is Gazebo's world frame
(world models, RViz markers and MoveIt's planning scene are placed in it),
so pass the robot's spawn pose; on hardware odom starts wherever the
robot is, 0 0 0.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1', 'yes')
    prefix = f'{robot_id}_' if robot_id else ''
    x, y, yaw = (float(LaunchConfiguration(a).perform(context)) for a in ('x', 'y', 'yaw'))
    config = os.path.join(get_package_share_directory('ranger_xarm6_navigation'), 'config', 'ekf.yaml')

    ekf_inputs = Node(
        package='ranger_xarm6_navigation',
        executable='ekf_inputs.py',
        namespace=robot_id,
        parameters=[config, {'imu_frame': f'{prefix}livox_imu_frame', 'use_sim_time': use_sim_time}],
        output='screen',
    )
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace=robot_id,
        parameters=[config, {
            'map_frame': f'{prefix}map',
            'odom_frame': f'{prefix}odom',
            'base_link_frame': f'{prefix}base_link',
            'world_frame': f'{prefix}odom',
            # x y z roll pitch yaw, then velocities and accelerations.
            'initial_state': [x, y, 0.0, 0.0, 0.0, yaw] + [0.0] * 9,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )
    return [ekf_inputs, ekf]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot_a', description='ROS namespace + frame prefix; must match ranger_xarm6.launch.py'),
        DeclareLaunchArgument('x', default_value='0.0', description='Initial x in odom [m]; the spawn x in sim'),
        DeclareLaunchArgument('y', default_value='0.0', description='Initial y in odom [m]; the spawn y in sim'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial yaw in odom [rad]; the spawn yaw in sim'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='true with Gazebo, false on real hardware'),
        OpaqueFunction(function=launch_setup),
    ])
