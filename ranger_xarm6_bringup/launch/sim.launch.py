#!/usr/bin/env python3
"""The whole sim stack in one launch, viewed in Foxglove and klein-bt.

    ros2 launch ranger_xarm6_bringup sim.launch.py
    ros2 launch ranger_xarm6_bringup sim.launch.py localization:=ndt     # EKF + NDT, as on hardware

Replaces the four terminals of the repo README's usage guide (robot, Nav2,
MoveIt, tasks) and their RViz windows. Stages start when the one before is
ready (scripts/wait_for.py), not after fixed delays:

  1. the robot in Gazebo (ranger_xarm6_description), foxglove_bridge,
     klein-bt, the Foxglove app opened on the bridge, and (artc_lab) the
     obstacle buttons' server (spawn_obstacles.py --serve);
  2. its controllers active -> Nav2 (+ with localization:=ndt the EKF and
     the NDT localizer, the robot's own odom TF off);
  3. bt_navigator active -> MoveIt (ranger_xarm6_manipulation);
  4. MoveIt's move_to_goal up -> the task layer (ranger_xarm6_tasks).

Then run tasks as usual (run_task.py), watch them in klein-bt, and see
everything else in Foxglove (config/foxglove_layout.json, imported once).
rviz:=true also opens the stages' own RViz windows.
"""
import os
import shutil
import urllib.parse

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, launch_file, **arguments):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments={k: str(v) for k, v in arguments.items()}.items())


def after(waiter, actions, label):
    """Run actions when waiter (a wait_for.py process) exits 0; log if it timed out."""
    def on_exit(event, context):
        if event.returncode == 0:
            return actions
        return [LogInfo(msg=f'[sim.launch] {label} never became ready: later stages not started')]
    return RegisterEventHandler(OnProcessExit(target_action=waiter, on_exit=on_exit))


def launch_setup(context, *args, **kwargs):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    flag = lambda name: arg(name).lower() in ('true', '1', 'yes')  # noqa: E731
    robot_id = arg('robot_id')
    ns = f'/{robot_id}' if robot_id else ''
    x, y, yaw = arg('x'), arg('y'), arg('yaw')
    localization = arg('localization')
    if localization not in ('static', 'ndt'):
        raise ValueError(f"localization must be 'static' or 'ndt', got {localization!r}")
    ndt = localization == 'ndt'
    rviz = flag('rviz')

    def wait_for(label, *conditions):
        return Node(package='ranger_xarm6_bringup', executable='wait_for.py', name=f'wait_for_{label}',
                    arguments=['--label', label, *conditions], parameters=[{'use_sim_time': True}],
                    output='screen')

    # Stage 1: the robot, and the viewers.
    robot = include('ranger_xarm6_description', 'ranger_xarm6.launch.py',
                    robot_id=robot_id, world=arg('world'), x=x, y=y, yaw=yaw, run_rviz=rviz,
                    publish_odom_tf=not ndt, base_drive=arg('base_drive'),
                    random_obstacles=arg('random_obstacles'), obstacle_seed=arg('obstacle_seed'))
    stage1 = [robot]
    # Buttons for random boxes (Foxglove layout): obstacles/respawn, obstacles/clear.
    if os.path.splitext(os.path.basename(arg('world')))[0] == 'artc_lab':
        count = int(arg('random_obstacles')) or 5
        stage1.append(Node(package='ranger_xarm6_description', executable='spawn_obstacles.py',
                           name='obstacle_spawner', namespace=robot_id,
                           arguments=['--serve', '--count', str(count), '--seed', arg('obstacle_seed')],
                           output='screen'))
    port = arg('foxglove_port')
    if flag('foxglove'):
        stage1.append(Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
                           parameters=[{'port': int(port), 'use_sim_time': True,
                                        # package:// meshes for the robot model
                                        'capabilities': ['clientPublish', 'parameters', 'parametersSubscribe',
                                                         'services', 'connectionGraph', 'assets']}],
                           output='screen'))
        app = shutil.which('foxglove-studio')
        if flag('open_foxglove') and app:
            url = 'foxglove://open?' + urllib.parse.urlencode(
                {'ds': 'foxglove-websocket', 'ds.url': f'ws://localhost:{port}'})
            stage1.append(ExecuteProcess(cmd=[app, url], output='log'))
        elif flag('open_foxglove'):
            stage1.append(LogInfo(msg='[sim.launch] no foxglove-studio on PATH: open Foxglove yourself, '
                                      f'Open connection -> Foxglove WebSocket -> ws://localhost:{port}'))
    if flag('klein'):
        klein = shutil.which('klein-bt') or os.path.expanduser('~/.local/bin/klein-bt')
        if os.path.exists(klein):
            stage1.append(ExecuteProcess(cmd=[klein, '--port', arg('klein_port'), '--robot-port', arg('groot2_port'),
                                              *([] if flag('open_klein') else ['--no-browser'])],
                                         output='screen'))
        else:
            stage1.append(LogInfo(msg='[sim.launch] klein-bt not installed (ranger_xarm6_tasks README): '
                                      'the live task view is off'))

    # Stage 2: controllers active -> Nav2 (+ EKF and localizer).
    controllers = ['joint_state_broadcaster', 'arm_velocity_controller', 'gripper_position_controller']
    if arg('base_drive') == 'physics':
        controllers += ['steering_position_controller', 'wheel_velocity_controller']
    robot_ready = wait_for('robot', '--controllers', f'{ns}/controller_manager', ','.join(controllers))
    stage2 = [include('ranger_xarm6_navigation', 'navigation.launch.py',
                      robot_id=robot_id, map=arg('map'), use_rviz=rviz, localization=localization,
                      **({'initial_pose': f'{x} {y} {float(yaw) * 57.29577951308232}'} if ndt else {}))]
    if ndt:
        stage2.insert(0, include('ranger_xarm6_navigation', 'odometry.launch.py', robot_id=robot_id, x=x, y=y, yaw=yaw))

    # Stage 3: Nav2 active -> MoveIt.
    nav_ready = wait_for('nav2', '--lifecycle', f'{ns}/bt_navigator')
    stage3 = [include('ranger_xarm6_manipulation', 'control.launch.py',
                      robot_id=robot_id, use_rviz=rviz, controller=arg('controller'))]

    # Stage 4: MoveIt up -> tasks.
    moveit_ready = wait_for('moveit', '--action', f'{ns}/mobile_manipulation/move_to_goal')
    stage4 = [include('ranger_xarm6_tasks', 'tasks.launch.py', robot_id=robot_id, groot2_port=arg('groot2_port'))]

    return [
        *stage1,
        robot_ready,
        after(robot_ready, [*stage2, nav_ready], 'the robot'),
        after(nav_ready, [*stage3, moveit_ready], 'Nav2'),
        after(moveit_ready, stage4, 'MoveIt'),
    ]


def generate_launch_description():
    maps = os.path.join(get_package_share_directory('ranger_xarm6_navigation'), 'maps', 'artc_lab.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot_a'),
        DeclareLaunchArgument('world', default_value='artc_lab.world'),
        DeclareLaunchArgument('x', default_value='0.94', description='spawn pose (the home spot in artc_lab)'),
        DeclareLaunchArgument('y', default_value='4.35'),
        DeclareLaunchArgument('yaw', default_value='-1.5708', description='rad'),
        DeclareLaunchArgument('map', default_value=maps, description='the 2D grid .yaml (ranger_xarm6_navigation)'),
        DeclareLaunchArgument('localization', default_value='static',
                              description="'static': ground-truth odom, identity map -> odom; "
                                          "'ndt': EKF + NDT scan-to-map, as on hardware"),
        DeclareLaunchArgument('base_drive', default_value='physics', description="'physics' or 'kinematic'"),
        DeclareLaunchArgument('controller', default_value='moveit_sequential',
                              description='MoveIt default mode: moveit_sequential or moveit_whole_body'),
        DeclareLaunchArgument('random_obstacles', default_value='0'),
        DeclareLaunchArgument('obstacle_seed', default_value='-1'),
        DeclareLaunchArgument('rviz', default_value='false', description="also each stage's own RViz"),
        DeclareLaunchArgument('foxglove', default_value='true', description='foxglove_bridge'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('open_foxglove', default_value='true', description='open the Foxglove app on the bridge'),
        DeclareLaunchArgument('klein', default_value='true', description='klein-bt, the live task view'),
        DeclareLaunchArgument('klein_port', default_value='8080'),
        DeclareLaunchArgument('groot2_port', default_value='1667', description="the task server's live view (klein-bt reads it)"),
        DeclareLaunchArgument('open_klein', default_value='true', description="open klein-bt's page in the browser"),
        OpaqueFunction(function=launch_setup),
    ])
