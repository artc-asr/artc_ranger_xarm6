#!/usr/bin/env python3
"""The front D435i (fixed_cam1) as a small cloud: depth/points_nav.

depth_to_cloud (src/depth_to_cloud.cpp) turns its depth image into 5 cm
voxels within 3 m, in the camera's optical frame, each released once TF
can place it in odom (see the source), for Nav2's costmaps
(navigation.launch.py) and depth_mapper.py (mapping.launch.py); both
include this. Published on /<robot_id>/fixed_cam1_camera/depth/points_nav.

Input: sim, the gz depth image (labelled with the camera's body frame but
in its optical axes, so the frame is overridden). Real, realsense2_camera
(ranger_xarm6.launch.py), which nests its topics under
camera_namespace/camera_name. front_depth_image/front_depth_info override
either.

memory:=true (navigation.launch.py) adds obstacle_memory: what the camera
has seen, kept after it's out of view, on depth/obstacle_memory (odom), for
collision_monitor (see src/obstacle_memory.cpp).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1', 'yes')
    prefix = f'{robot_id}_' if robot_id else ''
    cam = f'/{robot_id}/fixed_cam1_camera' if robot_id else f'/{prefix}fixed_cam1_camera'
    if use_sim_time:
        depth, info = f'{cam}/depth/depth_image', f'{cam}/depth/camera_info'
        frame = f'{prefix}fixed_cam1_camera_depth_optical_frame'
    else:
        rs = f'{cam}/{prefix}fixed_cam1_camera'
        depth, info, frame = f'{rs}/depth/image_rect_raw', f'{rs}/depth/camera_info', ''
    depth = LaunchConfiguration('front_depth_image').perform(context) or depth
    info = LaunchConfiguration('front_depth_info').perform(context) or info
    nodes = [Node(
        package='ranger_xarm6_navigation', executable='depth_to_cloud', name='front_depth_to_cloud',
        namespace=robot_id,
        parameters=[{'use_sim_time': use_sim_time, 'voxel_size': 0.05, 'min_range': 0.2, 'max_range': 3.0,
                     'frame_id': frame, 'wait_for_frame': f'{prefix}odom'}],
        remappings=[('depth', depth), ('camera_info', info), ('points', f'{cam}/depth/points_nav')],
        output='screen',
    )]
    if LaunchConfiguration('memory').perform(context).lower() in ('true', '1', 'yes'):
        # The footprint box is cloud_self_filter's (navigation.launch.py).
        nodes.append(Node(
            package='ranger_xarm6_navigation', executable='obstacle_memory', name='front_obstacle_memory',
            namespace=robot_id,
            parameters=[{'use_sim_time': use_sim_time, 'frame_id': frame, 'odom_frame': f'{prefix}odom',
                         'base_frame': f'{prefix}base_link', 'voxel_size': 0.05, 'min_range': 0.2,
                         'max_range': 3.0, 'z_min': 0.05, 'z_max': 1.5, 'clear_margin': 0.1, 'max_age': 30.0,
                         'keep_radius': 3.0, 'footprint_min_x': -0.57, 'footprint_max_x': 0.42,
                         'footprint_min_y': -0.31, 'footprint_max_y': 0.31}],
            remappings=[('depth', depth), ('camera_info', info), ('points', f'{cam}/depth/obstacle_memory')],
            output='screen',
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot_a', description='ROS namespace + frame prefix; must match ranger_xarm6.launch.py'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='true with Gazebo, false on real hardware'),
        DeclareLaunchArgument('front_depth_image', default_value='',
                              description="Front D435i depth image topic; '' = the sim's, or realsense2_camera's with use_sim_time:=false"),
        DeclareLaunchArgument('front_depth_info', default_value='', description='Its camera_info topic; same defaults'),
        DeclareLaunchArgument('memory', default_value='false',
                              description='Also obstacle_memory (depth/obstacle_memory), for collision_monitor'),
        OpaqueFunction(function=launch_setup),
    ])
