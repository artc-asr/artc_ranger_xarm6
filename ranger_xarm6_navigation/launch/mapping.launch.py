#!/usr/bin/env python3
"""Mapping session: FAST-LIO2 on the Livox Mid-360 and its built-in IMU.

The robot (sim or real) comes from ranger_xarm6_description in another
terminal. Drive it around (teleop, or MoveIt base goals), then save:

    ros2 launch ranger_xarm6_navigation mapping.launch.py map:=ranger_xarm6_navigation/maps/lab.pcd
    ros2 service call /robot_a/map_save std_srvs/srv/Trigger

The saved .pcd is the 3D map, in FAST-LIO's 'camera_init' frame: the IMU's
pose when mapping started, gravity-aligned. Next to it, <map>_depth.pcd:
the front D435i's low obstacles in the same frame (depth_mapper.py, written
every 10 s and on exit); pcd_to_grid.py merges both.

Nodes (in the robot's namespace):
  livox_cloud_to_custom: livox/lidar (PointCloud2, the driver's / sim's
    xfer_format 0) -> livox/lidar_custom (CustomMsg, what FAST-LIO reads).
  fastlio_mapping: config/fast_lio.yaml. Its absolutely-named outputs are
    remapped into the namespace, under fast_lio/: odometry, path,
    cloud_registered (each scan, world frame), laser_map (the map so far).
  front_depth_to_cloud (front_depth.launch.py) + depth_mapper: with
    front_depth:=true (default).
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
    nodes = [converter, fast_lio]
    if LaunchConfiguration('front_depth').perform(context).lower() in ('true', '1', 'yes'):
        prefix = f'{robot_id}_' if robot_id else ''
        share = get_package_share_directory('ranger_xarm6_navigation')
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'front_depth.launch.py')),
            launch_arguments={'robot_id': robot_id, 'use_sim_time': str(use_sim_time).lower()}.items(),
        ))
        nodes.append(Node(
            package='ranger_xarm6_navigation',
            executable='depth_mapper.py',
            namespace=robot_id,
            parameters=[{
                'output': os.path.splitext(map_file)[0] + '_depth.pcd',
                'body_frame': f'{prefix}livox_imu_frame',
                'base_frame': f'{prefix}base_link',
                'base_height': float(LaunchConfiguration('base_height').perform(context)),
                'use_sim_time': use_sim_time,
            }],
            remappings=[('points', f'{ns}/fixed_cam1_camera/depth/points_nav')],
            output='screen',
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot_a', description='ROS namespace + frame prefix; must match ranger_xarm6.launch.py'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='true with Gazebo, false on real hardware'),
        DeclareLaunchArgument('map', default_value='ranger_xarm6_navigation/maps/map.pcd',
                              description="Where map_save writes the 3D map (.pcd); relative paths from where it's run (the repo root, README section 0)"),
        DeclareLaunchArgument('front_depth', default_value='true', description="Also record the front D435i's low obstacles (<map>_depth.pcd)"),
        DeclareLaunchArgument('base_height', default_value='0.315', description='base_link above the floor [m]'),
        OpaqueFunction(function=launch_setup),
    ])
