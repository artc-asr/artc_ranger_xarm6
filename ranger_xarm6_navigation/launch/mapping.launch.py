#!/usr/bin/env python3
"""Mapping session: FAST-LIO2 on the Livox Mid-360 and its built-in IMU.

The robot (sim or real) comes from ranger_xarm6_description in another
terminal. Drive it around (teleop, or MoveIt base goals), then save:

    ros2 launch ranger_xarm6_navigation mapping.launch.py map:=~/ranger_xarm6_maps/lab.pcd
    ros2 service call /robot_a/map_save std_srvs/srv/Trigger

The saved .pcd is the 3D map, in FAST-LIO's 'camera_init' frame: the IMU's
pose when mapping started, gravity-aligned.

Nodes (in the robot's namespace):
  livox_cloud_to_custom: livox/lidar (PointCloud2, the driver's / sim's
    xfer_format 0) -> livox/lidar_custom (CustomMsg, what FAST-LIO reads).
  fastlio_mapping: config/fast_lio.yaml. Its absolutely-named outputs are
    remapped into the namespace, under fast_lio/: odometry, path,
    cloud_registered (each scan, world frame), laser_map (the map so far).
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
    map_file = os.path.abspath(os.path.expanduser(LaunchConfiguration('map').perform(context)))
    os.makedirs(os.path.dirname(map_file), exist_ok=True)
    ns = f'/{robot_id}' if robot_id else ''
    config = os.path.join(get_package_share_directory('ranger_xarm6_navigation'), 'config', 'fast_lio.yaml')

    converter = Node(
        package='ranger_xarm6_navigation',
        executable='livox_cloud_to_custom',
        namespace=robot_id,
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        namespace=robot_id,
        parameters=[config, {
            'common.lid_topic': f'{ns}/livox/lidar_custom',
            'common.imu_topic': f'{ns}/livox/imu',
            'map_file_path': map_file,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/Odometry', f'{ns}/fast_lio/odometry'),
            ('/path', f'{ns}/fast_lio/path'),
            ('/cloud_registered', f'{ns}/fast_lio/cloud_registered'),
            ('/cloud_registered_body', f'{ns}/fast_lio/cloud_registered_body'),
            ('/cloud_effected', f'{ns}/fast_lio/cloud_effected'),
            ('/Laser_map', f'{ns}/fast_lio/laser_map'),
        ],
        output='screen',
    )
    return [converter, fast_lio]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot_a', description='ROS namespace + frame prefix; must match ranger_xarm6.launch.py'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='true with Gazebo, false on real hardware'),
        DeclareLaunchArgument('map', default_value='~/ranger_xarm6_maps/map.pcd', description='Where map_save writes the 3D map (.pcd)'),
        OpaqueFunction(function=launch_setup),
    ])
