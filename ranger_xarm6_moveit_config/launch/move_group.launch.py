#!/usr/bin/env python3
"""MoveIt 2 move_group (+ optional RViz MotionPlanning) for the Ranger + xArm6.

Runs alongside ranger_xarm6_description's ranger_xarm6.launch.py (sim or
real), in the same robot_id namespace:

    ros2 launch ranger_xarm6_description ranger_xarm6.launch.py run_rviz:=false
    ros2 launch ranger_xarm6_moveit_config move_group.launch.py

MoveIt executes arm plans on arm_trajectory_controller, which is loaded
inactive; ranger_xarm6_manipulation's coordinator switches it in. To plan
and execute from RViz alone, switch first:

    ros2 control switch_controllers -c /robot_a/controller_manager \\
        --activate arm_trajectory_controller --deactivate arm_velocity_controller
"""
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(pkg_share, name):
    with open(os.path.join(pkg_share, 'config', name)) as f:
        return yaml.safe_load(f)


def _prefix_joint_keys(limits, prefix):
    limits['joint_limits'] = {f'{prefix}{j}': v for j, v in limits['joint_limits'].items()}
    return limits


def launch_setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1', 'yes')
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() in ('true', '1', 'yes')
    prefix = f'{robot_id}_' if robot_id else ''

    description_share = get_package_share_directory('ranger_xarm6_description')
    moveit_share = get_package_share_directory('ranger_xarm6_moveit_config')

    # Same xacro as the running robot, so collision geometry matches.
    # ros2_control_plugin='none' only skips the Gazebo/hardware blocks,
    # which MoveIt doesn't read.
    robot_description = xacro.process_file(
        os.path.join(description_share, 'robots', 'ranger_xarm6.urdf.xacro'),
        mappings={'prefix': prefix, 'robot_namespace': robot_id, 'ros2_control_plugin': 'none'},
    ).toxml()
    robot_description_semantic = xacro.process_file(
        os.path.join(moveit_share, 'srdf', 'ranger_xarm6.srdf.xacro'),
        mappings={'prefix': prefix},
    ).toxml()

    controllers = _load_yaml(moveit_share, 'moveit_controllers.yaml')
    scm = controllers['moveit_simple_controller_manager']
    for name in scm['controller_names']:
        scm[name]['joints'] = [f'{prefix}{j}' for j in scm[name]['joints']]

    ompl = _load_yaml(moveit_share, 'ompl_planning.yaml')
    ompl.update({
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': ' '.join([
            'default_planner_request_adapters/AddTimeOptimalParameterization',
            'default_planner_request_adapters/FixWorkspaceBounds',
            'default_planner_request_adapters/FixStartStateBounds',
            'default_planner_request_adapters/FixStartStateCollision',
            'default_planner_request_adapters/FixStartStatePathConstraints',
        ]),
        'start_state_max_bounds_error': 0.1,
    })

    moveit_params = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': _load_yaml(moveit_share, 'kinematics.yaml'),
        'robot_description_planning': _prefix_joint_keys(_load_yaml(moveit_share, 'joint_limits.yaml'), prefix),
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'use_sim_time': use_sim_time,
    }

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=robot_id,
        output='screen',
        parameters=[moveit_params, controllers],
    )

    nodes = [move_group]
    if use_rviz:
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            namespace=robot_id,
            arguments=['-d', os.path.join(moveit_share, 'config', 'moveit.rviz'), '-f', f'{prefix}odom'],
            parameters=[{
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': moveit_params['robot_description_kinematics'],
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot_a', description='ROS namespace + joint/frame prefix; must match ranger_xarm6.launch.py'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='true with Gazebo, false on real hardware'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='RViz with the MotionPlanning panel'),
        OpaqueFunction(function=launch_setup),
    ])
