#!/usr/bin/env python3
"""Bring up the Ranger Mini 3.0 + xArm6 mobile manipulator.

Single entry point for simulation and real hardware, selected with the
'sim'/'gazebo' arguments, so wbcc_controller/wbc.py (which expects a
namespaced 'arm_velocity_controller/commands' topic and
'{ns}_joint1'..'{ns}_joint6' joint names) sees an identical interface
regardless of which mode is active:

    ros2 launch ranger_xarm6_description ranger_xarm6.launch.py                    # full physics, RViz for viz, gz headless (default)
    ros2 launch ranger_xarm6_description ranger_xarm6.launch.py gz_gui:=true       # also pop the Gazebo GUI window
    ros2 launch ranger_xarm6_description ranger_xarm6.launch.py gazebo:=false      # skip physics entirely -- structural/visual check only
    ros2 launch ranger_xarm6_description ranger_xarm6.launch.py sim:=false \\
        robot_ip:=192.168.1.231 can_device:=can0                                   # real hardware

sim:=true, gazebo:=true (default) -> Gazebo Harmonic (gz_ros2_control/GazeboSimSystem)
              running headless by default (server only, no GUI window -- RViz is
              the visualization; gz_gui:=true also launches Gazebo's own GUI).
              Base fixed (no drive plugin wired yet -- separate task).
sim:=true, gazebo:=false -> robot_state_publisher + joint_state_publisher + static
              base TF + RViz only. No controller_manager, no gz_sim at all --
              arm_velocity_controller commands go nowhere in this mode (nothing
              claims that interface). Only for when you don't want Gazebo running
              at all, not even headless.
sim:=false -> real xArm6 over the network (uf_robot_hardware/UFRobotSystemHardware)
              + real Ranger Mini 3 over CAN (westonrobot_ranger_ros2). UNTESTED
              against physical hardware -- see wbcc_mm README / handover notes
              before running this against a real robot.
"""
import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from uf_ros_lib.uf_robot_utils import get_xacro_content, generate_robot_api_params


# Base controller set (see config/ros2_controllers.yaml). Joint names there
# are unprefixed; this prefixes only the *joints*, never the controller
# names themselves, so 'arm_velocity_controller' stays exactly that
# regardless of robot_id -- wbc.py's topic name doesn't get a robot_id
# baked into it, only its node namespace does.
#
# When a namespace is set, the whole params tree also needs nesting one
# level under that namespace key (ROS2 param-YAML nodes only match plain
# top-level keys like 'controller_manager' against an UNnamespaced node of
# that exact name; a namespaced node needs 'robot_a: {controller_manager:
# ...}'). uf_ros_lib.generate_ros2_control_params_temp_file does the same
# thing for xarm's own default controller yaml -- confirmed by reading its
# source (add_prefix_to_ros2_control_params + the ros_namespace wrap).
def _prefix_controller_joints(yaml_path, prefix, robot_id, use_sim_time):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    if prefix:
        for name, cfg in data.items():
            if name == 'controller_manager':
                continue
            joints = (cfg or {}).get('ros__parameters', {}).get('joints')
            if joints:
                cfg['ros__parameters']['joints'] = [f'{prefix}{j}' for j in joints]
    if use_sim_time:
        data['controller_manager']['ros__parameters']['use_sim_time'] = True
    if robot_id:
        data = {robot_id: data}
    with tempfile.NamedTemporaryFile(mode='w', prefix='ranger_xarm6_controllers_', suffix='.yaml', delete=False) as f:
        yaml.dump(data, f)
        return f.name


def launch_setup(context, *args, **kwargs):
    # gz-sim doesn't resolve 'package://' mesh URIs against AMENT_PREFIX_PATH
    # the way robot_state_publisher/RViz do -- it needs GZ_SIM_RESOURCE_PATH
    # explicitly, which is empty by default. Only matters for meshes
    # referenced with bare 'package://' (e.g. ros-humble-realsense2-
    # description's sensor_d435i, used for the two frame-mounted cameras);
    # this repo's own meshes all resolve fine already because
    # xarm_device_macro.xacro deliberately uses absolute file://$(find ...)
    # paths instead, specifically for the gz-sim case (see that file's own
    # mesh_path property) -- so this is filling the same gap
    # realsense2_description didn't work around itself, for every ROS
    # package's share dir at once (not just this one), so it doesn't need
    # revisiting the next time some other vendored package's mesh doesn't
    # show up in Gazebo. Setting os.environ directly (not a launch
    # SetEnvironmentVariable action) so it's in effect for every
    # ExecuteProcess/Node constructed below in this same call, including
    # gz_sim.launch.py further down.
    resource_paths = [
        os.path.join(p, 'share')
        for p in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
        if p
    ]
    existing_gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_gz_resource_path:
        resource_paths.append(existing_gz_resource_path)
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(resource_paths)

    sim = LaunchConfiguration('sim').perform(context).lower() in ('true', '1', 'yes')
    # gazebo:=false -- sim only: skip physics entirely (no gz_sim, no
    # ros2_control/controller_manager). Just robot_state_publisher + a
    # static base TF + joint_state_publisher (so RViz/wbc.py have SOMETHING
    # to read) + RViz. Fast, no physics, for structural/visual checks --
    # commanding arm_velocity_controller does nothing in this mode since
    # there's no controller_manager to claim it.
    gazebo = LaunchConfiguration('gazebo').perform(context).lower() in ('true', '1', 'yes')
    launch_gazebo = LaunchConfiguration('launch_gazebo').perform(context).lower() in ('true', '1', 'yes')
    # gazebo:=true only: whether Gazebo's own GUI window also opens.
    # Headless by default -- RViz already provides visualization, so the
    # Gazebo window is redundant unless you specifically want it (e.g. to
    # see contact forces / physics debug visuals Gazebo renders that RViz
    # doesn't).
    gz_gui = LaunchConfiguration('gz_gui').perform(context).lower() in ('true', '1', 'yes')
    robot_id = LaunchConfiguration('robot_id').perform(context)
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    can_device = LaunchConfiguration('can_device').perform(context)
    prefix = f'{robot_id}_' if robot_id else ''

    ros2_control_params = _prefix_controller_joints(
        os.path.join(get_package_share_directory('ranger_xarm6_description'), 'config', 'ros2_controllers.yaml'),
        prefix=prefix,
        robot_id=robot_id,
        use_sim_time=sim,
    )

    xacro_kwargs = dict(
        prefix=prefix,
        robot_namespace=robot_id,
        ros2_control_params=ros2_control_params,
    )
    if sim and gazebo:
        xacro_kwargs.update(ros2_control_plugin='gz_ros2_control/GazeboSimSystem')
    elif sim:
        # No physics running (see 'gazebo' arg) -- explicitly override to
        # something other than the xacro's own default (which IS
        # 'gz_ros2_control/GazeboSimSystem'), so its _is_gz_sim check is
        # false and it skips both the gz_ros2_control plugin block and the
        # F/T sensor block (neither means anything without a
        # controller_manager/Gazebo to back them).
        xacro_kwargs.update(ros2_control_plugin='none')
    else:
        xacro_kwargs.update(
            ros2_control_plugin='uf_robot_hardware/UFRobotSystemHardware',
            robot_ip=robot_ip,
        )

    robot_description = {
        'robot_description': get_xacro_content(
            context,
            xacro_file=Path(get_package_share_directory('ranger_xarm6_description')) / 'robots' / 'ranger_xarm6.urdf.xacro',
            **xacro_kwargs,
        )
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_id,
        output='screen',
        # sim time only makes sense when a '/clock' publisher actually
        # exists (Gazebo); with gazebo:=false there's none, and use_sim_time
        # would stall this node waiting for clock messages that never come.
        parameters=[{'use_sim_time': sim and gazebo}, robot_description],
    )

    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=robot_id,
            output='screen',
            arguments=['joint_state_broadcaster'],
            parameters=[{'use_sim_time': sim}],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=robot_id,
            output='screen',
            arguments=['arm_velocity_controller'],
            parameters=[{'use_sim_time': sim}],
        ),
        # G2 gripper's drive_joint (config/ros2_controllers.yaml); separate
        # ros2_control resource ('XArmGripperSystem', see
        # xarm_gripper.ros2_control.xacro) from the arm's, so its own
        # spawner call, same pattern as arm_velocity_controller above.
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=robot_id,
            output='screen',
            arguments=['gripper_position_controller'],
            parameters=[{'use_sim_time': sim}],
        ),
    ]

    if sim:
        run_rviz = LaunchConfiguration('run_rviz').perform(context).lower() in ('true', '1', 'yes')

        # The base has no ros2_control-driven wheels or real drive plugin
        # (wheels are passive -- see ranger_mini_v3_description), so
        # nothing else publishes '{ns}_odom' -> '{ns}_base_link' or moves
        # the Gazebo entity in response to wbc.py's cmd_vel. This node is
        # both: it's a kinematic (not physics-based) simulation-only
        # stand-in for a real drive -- integrates cmd_vel and teleports
        # the entity to match via gz-transport -- and the sole owner of
        # the odom->base_link TF (also updated by the "Initial Position"
        # button via the 'set_base_pose' topic; see base_pose_publisher.py
        # for why that's a single node rather than wbc_visualize.py
        # broadcasting TF itself). wbc.py's control loop looks this TF up
        # every tick via update_robot_state() and silently no-ops if it's
        # missing -- without this node, bringup.launch.py would spin but
        # the WBC state machine would never advance.
        odom_tf_publisher = Node(
            package='ranger_xarm6_description',
            executable='base_pose_publisher.py',
            namespace=robot_id,
            parameters=[{
                'x': float(LaunchConfiguration('x').perform(context)),
                'y': float(LaunchConfiguration('y').perform(context)),
                'z': float(LaunchConfiguration('z').perform(context)),
                'yaw': float(LaunchConfiguration('yaw').perform(context)),
                'frame_id': f'{prefix}odom',
                'child_frame_id': f'{prefix}base_link',
                # Must match spawn_entity_node's '-name' below so this
                # node's gz-transport teleport calls (see
                # base_pose_publisher.py) hit the right entity.
                'gz_entity_name': robot_id or 'ranger_xarm6',
                'use_sim_time': gazebo,
            }],
            output='screen',
        )

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            # Namespaced like robot_state_publisher_node, so the config's
            # relative 'robot_description' topic resolves under this robot's
            # namespace without hardcoding robot_id into the .rviz file.
            namespace=robot_id,
            arguments=[
                '-f', f'{prefix}odom',
                '-d', PathJoinSubstitution(
                    [FindPackageShare('ranger_xarm6_description'), 'config', 'ranger_xarm6.rviz']
                ),
            ],
            parameters=[{'use_sim_time': gazebo}],
            output='screen',
        )

        if not gazebo:
            # No physics: just publish a static robot -- no controller_manager,
            # no gz_sim, no F/T bridge. joint_state_publisher feeds default
            # joint positions so RobotModel/TF have something to show and
            # wbc.py's joint_states subscription isn't simply empty (it'll
            # stay frozen at those defaults since nothing drives them here --
            # this is a structural/visual check, not a motion test).
            joint_state_publisher_node = Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                namespace=robot_id,
                output='screen',
                parameters=[{'use_sim_time': False}, robot_description],
            )
            return [
                robot_state_publisher_node,
                joint_state_publisher_node,
                odom_tf_publisher,
                *([rviz_node] if run_rviz else []),
            ]

        # When embedded under wbcc_bringup/simulation.launch.py, that file
        # already starts Gazebo (with the wall-bearing 'tested_world.world',
        # copied into this package's worlds/ dir) and its own '/clock'
        # bridge -- launch_gazebo:=false skips duplicating both here. The
        # force_torque bridge is still ours to provide either way, since
        # nothing else knows about our sensor.
        startup_actions = []

        if launch_gazebo:
            world_path = PathJoinSubstitution(
                [FindPackageShare('ranger_xarm6_description'), 'worlds', 'tested_world.world']
            )
            # Server always (headless simulation needs this); GUI window
            # only when gz_gui:=true. Two separate gz_sim processes, same
            # pattern robotnik_gazebo_ignition's spawn_world.launch.py used
            # (its 'gui' arg, default false) before that package was removed.
            gazebo_server_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
                ),
                launch_arguments={'gz_args': [world_path, ' -s -r -v 3']}.items(),
            )
            startup_actions.append(gazebo_server_launch)
            if gz_gui:
                gazebo_gui_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
                    ),
                    launch_arguments={'gz_args': '-g'}.items(),
                )
                startup_actions.append(gazebo_gui_launch)
            clock_bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen',
            )
            startup_actions.append(clock_bridge)

        # F/T sensor bridge (see the sensor block in ranger_xarm6.urdf.xacro
        # for the sim-vs-hardware caveat). gz-sim publishes it on the flat
        # 'force_torque' topic (same convention robotnik_description uses
        # for Theron/Kairos); remap it under this robot's namespace so
        # wbc.py's plain 'force_torque' subscription (relative to its own
        # namespace) picks it up.
        ft_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/force_torque@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench'],
            remappings=[('/force_torque', f'/{robot_id}/force_torque')] if robot_id else [],
            output='screen',
        )
        startup_actions.append(ft_bridge)

        spawn_entity_node = Node(
            package='ros_gz_sim',
            executable='create',
            namespace=robot_id,
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-name', robot_id or 'ranger_xarm6',
                # gz_ros2_control's internal controller_manager inherits this
                # namespace.
                '-robot_namespace', robot_id,
                '-x', LaunchConfiguration('x').perform(context),
                '-y', LaunchConfiguration('y').perform(context),
                '-z', LaunchConfiguration('z').perform(context),
                '-Y', LaunchConfiguration('yaw').perform(context),
            ],
            parameters=[{'use_sim_time': True}],
        )

        post_spawn_actions = list(controller_spawners)
        if run_rviz:
            post_spawn_actions.append(rviz_node)

        return [
            robot_state_publisher_node,
            odom_tf_publisher,
            *[RegisterEventHandler(OnProcessStart(target_action=robot_state_publisher_node, on_start=a)) for a in startup_actions],
            RegisterEventHandler(OnProcessStart(target_action=robot_state_publisher_node, on_start=spawn_entity_node)),
            RegisterEventHandler(OnProcessExit(target_action=spawn_entity_node, on_exit=post_spawn_actions)),
        ]

    # ---- Real hardware ----
    # xarm_api's default connection/baud params; harmless to include even
    # though the F/T-sensor-bearing fields only matter if this unit actually
    # has UFACTORY's optional F/T sensor accessory installed (UNCONFIRMED --
    # the ARTC handover note lists only DC control box + arm + G2 gripper for
    # this unit, no F/T sensor line item).
    robot_api_params = generate_robot_api_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
        ros_namespace=robot_id,
        node_name='ufactory_driver',
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=robot_id,
        parameters=[robot_description, ros2_control_params, robot_api_params],
        remappings=[
            # Speculative: only present if this unit has the optional F/T
            # sensor accessory. wbc.py subscribes to 'force_torque' unchanged
            # across sim/real -- confirm this topic actually publishes
            # (`ros2 topic echo <ns>/force_torque`) before trusting force
            # control against real hardware.
            ('uf_ftsensor_ext_states', 'force_torque'),
        ],
        output='screen',
    )

    ranger_driver_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ranger_bringup'), 'launch', 'ranger_mini_v3.launch.xml'])
        ),
        launch_arguments={
            'port_name': can_device,
            'odom_frame': f'{prefix}odom',
            'base_frame': f'{prefix}base_link',
            'publish_odom_tf': 'true',
        }.items(),
    )

    return [
        robot_state_publisher_node,
        ros2_control_node,
        ranger_driver_launch,
        RegisterEventHandler(OnProcessStart(target_action=ros2_control_node, on_start=controller_spawners)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true', description='true = simulation (see gazebo arg), false = real hardware'),
        DeclareLaunchArgument('gazebo', default_value='true', description='sim only: true (default) = full Gazebo Harmonic physics (headless -- see gz_gui), false = RViz-only structural view (robot_state_publisher + joint_state_publisher + static TF, no controller_manager)'),
        DeclareLaunchArgument('gz_gui', default_value='false', description="sim+gazebo only: also launch Gazebo's own GUI window (default false -- RViz already visualizes)"),
        DeclareLaunchArgument('launch_gazebo', default_value='true', description='sim+gazebo only: false when an external launcher (e.g. wbcc_bringup/simulation.launch.py) already started Gazebo + its /clock bridge'),
        DeclareLaunchArgument('robot_id', default_value='robot_a', description='ROS namespace + joint/frame prefix'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.231', description='xArm6 IP (real hardware only; from ARTC handover note, confirm against your unit)'),
        DeclareLaunchArgument('can_device', default_value='can0', description='Ranger Mini 3 CAN interface (real hardware only)'),
        DeclareLaunchArgument('x', default_value='0', description='Spawn pose (sim only)'),
        DeclareLaunchArgument('y', default_value='0', description='Spawn pose (sim only)'),
        DeclareLaunchArgument('z', default_value='0.15', description='Spawn pose (sim only)'),
        DeclareLaunchArgument('yaw', default_value='0', description='Spawn pose (sim only)'),
        DeclareLaunchArgument('run_rviz', default_value='true', description='sim only: launch RViz (RobotModel + TF + wall marker)'),
        OpaqueFunction(function=launch_setup),
    ])
