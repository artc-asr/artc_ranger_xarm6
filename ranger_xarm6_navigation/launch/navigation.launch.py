#!/usr/bin/env python3
"""Nav2 for the base, on a 2D grid exported from the FAST-LIO map.

The robot (sim or real) comes from ranger_xarm6_description in another
terminal; odom -> base_link from it (sim: ground truth) or from
odometry.launch.py's EKF. Sim, the artc_lab grid from pcd_to_grid.py:

    ros2 launch ranger_xarm6_description ranger_xarm6.launch.py run_rviz:=false \\
        world:=artc_lab.world x:=0.94 y:=4.35 yaw:=-1.5708
    ros2 launch ranger_xarm6_navigation navigation.launch.py        # map: maps/artc_lab.yaml by default

Then '2D Goal Pose' in RViz, or the navigate_to_pose action.

map -> odom, localization:=
  static (default): an identity map -> odom, right in sim, where odom is
    Gazebo's world frame and the grid is exported in it; odometry drift
    isn't corrected. (static_map_to_odom:=false: none at all.)
  ndt: lidar_localization_ros2 (submodule) matches the Mid-360's cloud
    (without the arm) against the FAST-LIO map in the map frame
    (localization_map, default <map>_map_frame.pcd next to the grid, from
    scripts/pcd_to_map_frame.py) and publishes map -> odom (x, y, yaw:
    scripts/planar_map_to_odom.py), correcting
    the EKF's drift (run odometry.launch.py, and the robot with
    publish_odom_tf:=false). Where it starts: initial_pose:="x y yaw_deg"
    in the map frame (in sim, the spawn pose), else RViz's 2D Pose
    Estimate. config/localization.yaml.

Nodes (in the robot's namespace): front_depth_to_cloud + front_obstacle_memory
(front_depth.launch.py: the front D435i's depth image -> a 5 cm-voxel cloud
for the costmaps, and what it has seen for collision_monitor), map_server, planner_server (NavFn),
controller_server (RotationShim + Regulated Pure Pursuit), behavior_server,
bt_navigator, velocity_smoother (controller -> cmd_vel_nav -> ramped ->
cmd_vel_smoothed), collision_monitor (-> cmd_vel; stops/slows on the raw
lidar + front depth), lidar_self_filter (the lidar without the arm, for
collision_monitor), two lifecycle managers. config/nav2.yaml is written for
robot_id robot_a; its "robot_a" is replaced with the robot_id given here.
Not the nav2_bringup launch: it remaps /tf into the namespace, and this
robot's TF is global.
"""
import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV_NODES = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'velocity_smoother',
             'collision_monitor']


def _for_robot(path, robot_id, suffix):
    """A copy of path with robot_a's names replaced by robot_id's.

    A params file's node keys must match the node's full name, so the
    .yaml's are nested under the namespace (as nav2_bringup's
    RewrittenYaml does).
    """
    with open(path) as f:
        text = f.read()
    text = text.replace('/robot_a/', f'/{robot_id}/' if robot_id else '/')
    text = text.replace('robot_a_', f'{robot_id}_' if robot_id else '')
    if suffix == '.yaml' and robot_id:
        text = yaml.safe_dump({robot_id: yaml.safe_load(text)})
    out = tempfile.NamedTemporaryFile('w', prefix='ranger_xarm6_nav_', suffix=suffix, delete=False)
    out.write(text)
    out.close()
    return out.name


def launch_setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1', 'yes')
    static_map_to_odom = LaunchConfiguration('static_map_to_odom').perform(context).lower() in ('true', '1', 'yes')
    localization = LaunchConfiguration('localization').perform(context)
    if localization not in ('static', 'ndt'):
        raise ValueError(f"localization must be 'static' or 'ndt', got {localization!r}")
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() in ('true', '1', 'yes')
    map_yaml = os.path.abspath(os.path.expanduser(LaunchConfiguration('map').perform(context)))
    prefix = f'{robot_id}_' if robot_id else ''
    share = get_package_share_directory('ranger_xarm6_navigation')
    params_file = LaunchConfiguration('params_file').perform(context) or os.path.join(share, 'config', 'nav2.yaml')
    params = _for_robot(os.path.expanduser(params_file), robot_id, '.yaml')
    common = {'use_sim_time': use_sim_time}

    def nav2_node(package, executable, remappings=()):
        return Node(package=package, executable=executable, name=executable, namespace=robot_id,
                    parameters=[params, common], remappings=list(remappings), output='screen')

    nodes = [
        Node(package='nav2_map_server', executable='map_server', name='map_server', namespace=robot_id,
             parameters=[params, common, {'yaml_filename': map_yaml}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
             namespace=robot_id, parameters=[common, {'autostart': True, 'node_names': ['map_server']}],
             output='screen'),
        nav2_node('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_planner', 'planner_server'),
        nav2_node('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_bt_navigator', 'bt_navigator'),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother', [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_collision_monitor', 'collision_monitor'),
        # The Mid-360 without the robot (the arm), for collision_monitor
        # (see src/cloud_self_filter.cpp). The box is the costmaps'
        # published footprint (footprint + padding) and 1 cm more.
        Node(package='ranger_xarm6_navigation', executable='cloud_self_filter', name='lidar_self_filter',
             namespace=robot_id,
             parameters=[common, {'target_frame': f'{prefix}base_link', 'min_x': -0.57, 'max_x': 0.42,
                                  'min_y': -0.31, 'max_y': 0.31, 'voxel_size': 0.05}],
             remappings=[('in', 'livox/lidar'), ('out', 'livox/lidar_self_filtered')], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             namespace=robot_id, parameters=[common, {'autostart': True, 'node_names': NAV_NODES}],
             output='screen'),
    ]
    # The front D435i as a small cloud (depth/points_nav), for the costmaps.
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'front_depth.launch.py')),
        launch_arguments={
            'robot_id': robot_id,
            'use_sim_time': str(use_sim_time).lower(),
            'front_depth_image': LaunchConfiguration('front_depth_image').perform(context),
            'front_depth_info': LaunchConfiguration('front_depth_info').perform(context),
            'memory': 'true',
        }.items(),
    ))
    if localization == 'ndt':
        pcd = LaunchConfiguration('localization_map').perform(context) or os.path.splitext(map_yaml)[0] + '_map_frame.pcd'
        pcd = os.path.abspath(os.path.expanduser(pcd))
        if not os.path.exists(pcd):
            raise FileNotFoundError(f'{pcd}: make it with scripts/pcd_to_map_frame.py (the .pcd and pcd_to_grid.py\'s --origin)')
        loc_params = {'map_path': pcd}
        initial_pose = LaunchConfiguration('initial_pose').perform(context).split()
        if initial_pose:
            x, y, yaw_deg = (float(v) for v in initial_pose)
            yaw = math.radians(yaw_deg)
            loc_params.update(set_initial_pose=True, initial_pose_x=x, initial_pose_y=y, initial_pose_z=0.0,
                              initial_pose_qx=0.0, initial_pose_qy=0.0,
                              initial_pose_qz=math.sin(yaw / 2), initial_pose_qw=math.cos(yaw / 2))
        nodes += [
            Node(package='lidar_localization_ros2', executable='lidar_localization_node', name='lidar_localization',
                 namespace=robot_id,
                 parameters=[_for_robot(os.path.join(share, 'config', 'localization.yaml'), robot_id, '.yaml'),
                             common, loc_params],
                 # Its 'map' is a PointCloud2 topic (unused with a .pcd):
                 # not map_server's grid.
                 remappings=[('cloud', 'livox/lidar_self_filtered'), ('map', 'localization/map_cloud'),
                             ('initial_map', 'localization/initial_map'), ('odom', 'odometry/filtered'),
                             ('initialpose', 'localization/initialpose')],
                 output='screen'),
            # map -> odom from it, planar (see the script), and RViz's 2D
            # Pose Estimate relayed to it.
            Node(package='ranger_xarm6_navigation', executable='planar_map_to_odom.py', namespace=robot_id,
                 parameters=[common, {'map_frame': f'{prefix}map', 'ndt_frame': f'{prefix}map_3d',
                                      'odom_frame': f'{prefix}odom'}],
                 output='screen'),
            # It's a lifecycle node without Nav2's bond: its own manager,
            # bond off.
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', namespace=robot_id,
                 parameters=[common, {'autostart': True, 'node_names': ['lidar_localization'],
                                      'bond_timeout': 0.0}],
                 output='screen'),
        ]
    elif static_map_to_odom:
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name='map_to_odom', namespace=robot_id,
            arguments=['--frame-id', f'{prefix}map', '--child-frame-id', f'{prefix}odom'],
            parameters=[common], output='screen',
        ))
    if use_rviz:
        nodes.append(Node(
            package='rviz2', executable='rviz2', namespace=robot_id,
            arguments=['-d', _for_robot(os.path.join(share, 'config', 'navigation.rviz'), robot_id, '.rviz')],
            parameters=[common], output='screen',
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(get_package_share_directory('ranger_xarm6_navigation'),
                                                                 'maps', 'artc_lab.yaml'),
                              description="map_server .yaml of the 2D grid (scripts/pcd_to_grid.py); default: the sim's artc_lab (maps/)"),
        DeclareLaunchArgument('robot_id', default_value='robot_a', description='ROS namespace + frame prefix; must match ranger_xarm6.launch.py'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='true with Gazebo, false on real hardware'),
        DeclareLaunchArgument('localization', default_value='static',
                              description="map -> odom: 'static' (identity, right in sim) or 'ndt' (lidar_localization_ros2 against the FAST-LIO map)"),
        DeclareLaunchArgument('localization_map', default_value='',
                              description="ndt: the map-frame .pcd (scripts/pcd_to_map_frame.py); '' = <map>_map_frame.pcd"),
        DeclareLaunchArgument('initial_pose', default_value='',
                              description="ndt: where the robot starts, 'x y yaw_deg' in the map frame; '' = RViz 2D Pose Estimate"),
        DeclareLaunchArgument('static_map_to_odom', default_value='true',
                              description='localization:=static only: false = no map -> odom at all'),
        DeclareLaunchArgument('params_file', default_value='', description="Nav2 parameters, written for robot_a; '' = config/nav2.yaml"),
        DeclareLaunchArgument('front_depth_image', default_value='',
                              description="Front D435i depth image topic; '' = the sim's, or realsense2_camera's with use_sim_time:=false"),
        DeclareLaunchArgument('front_depth_info', default_value='', description='Its camera_info topic; same defaults'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='RViz with the map, costmaps, plan and 2D Goal Pose'),
        OpaqueFunction(function=launch_setup),
    ])
