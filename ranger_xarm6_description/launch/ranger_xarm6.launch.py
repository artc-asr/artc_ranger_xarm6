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
              + real Ranger Mini 3 over CAN (westonrobot_ranger_ros2) + the real
              realsense2_camera driver for the two fixed D435i cameras. UNTESTED
              against physical hardware -- see wbcc_mm README / handover notes
              before running this against a real robot.
              No Gazebo here -- deliberately: the standard ROS/Nav2 pattern for
              a "digital twin" of a real robot is robot_state_publisher + TF +
              RViz (same URDF as sim, real /joint_states driving it), not a
              second live Gazebo instance kept state-synced to the real one.
              RViz (run_rviz, on by default) is that digital twin here, same
              as sim: real xArm6/Ranger joint states -> TF -> RobotModel, real
              realsense2_camera topics -> the same point-cloud/image displays
              ranger_xarm6.rviz already uses in sim.
"""
import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
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


# Leaf topics gz-sim publishes for each fixed_cam{1,2} rgbd_camera/camera
# sensor (see fixed_cam_sensor_tags in ranger_xarm6.urdf.xacro), as (leaf,
# ROS type, gz type). depth/camera_info has no explicit <camera_info_topic>
# in that macro, but gz-sim's rgbd_camera sensor still auto-publishes it
# under '{topic}/camera_info' -- bridged here for completeness the same way
# xarm_gazebo's own '_robot_beside_table_gazebo.launch.py' bridges it for
# the (unrelated) wrist camera.
# depth/image vs depth/depth_image: gz-sim's rgbd_camera sensor publishes
# BOTH, and they are NOT the same data -- 'depth/image' is an rgb8 preview
# render (from this sensor's <camera><image><format>L8</format> config,
# same as any plain camera's "image" stream), NOT metric depth (confirmed
# by echoing its header: encoding=rgb8). 'depth/depth_image' is the actual
# gz.msgs.Image depth buffer the point cloud itself is generated from --
# that's the one to view as a real 2D depth map (e.g. an RViz Image
# display), not 'depth/image' despite the name suggesting otherwise.
_FIXED_CAMERA_LEAVES = [
    ('color/image_raw', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
    ('color/camera_info', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo'),
    ('infra1/image_raw', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
    ('infra1/camera_info', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo'),
    ('infra2/image_raw', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
    ('infra2/camera_info', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo'),
    ('depth/image', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
    ('depth/depth_image', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
    ('depth/camera_info', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo'),
    ('depth/points', 'sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked'),
]


def _fixed_camera_bridge_args(cam_id, prefix, robot_id):
    """ros_gz_bridge args + remaps for one fixed_cam{1,2} camera (gz -> ROS only).

    gz-sim publishes on '{prefix}{cam_id}_camera/...' -- 'prefix' here is the
    same f'{robot_id}_'-or-'' string used for the URDF's 'prefix' xacro arg,
    since fixed_cam_sensor_tags is called with prefix="$(arg prefix)fixed_cam{1,2}_".
    Remapped onto a clean '/{robot_id}/{cam_id}_camera/...' ROS topic, same
    pattern the force_torque bridge above uses; with no robot_id, the gz-side
    topic is already that clean, so no remap is needed.
    """
    gz_base = f'{prefix}{cam_id}_camera'
    ros_base = f'/{robot_id}/{cam_id}_camera' if robot_id else f'/{gz_base}'
    args = []
    remaps = []
    for leaf, ros_type, gz_type in _FIXED_CAMERA_LEAVES:
        args.append(f'/{gz_base}/{leaf}@{ros_type}[{gz_type}')
        if robot_id:
            remaps.append((f'/{gz_base}/{leaf}', f'{ros_base}/{leaf}'))
    return args, remaps


# Plain single-image cameras (left_camera/right_camera, see
# usb_camera_sensor_tags in ranger_xarm6.urdf.xacro): just image_raw +
# camera_info, unlike the D435i's multi-stream _FIXED_CAMERA_LEAVES. gz-side
# topic base is '{prefix}{cam_name}' directly (usb_camera_macro's own links
# are named "${name}_link"/"${name}_optical_frame", not
# "${prefix}fixed_cam{1,2}_camera_..." like sensor_d435i's convention), so
# no '_camera' suffix gets inserted the way _fixed_camera_bridge_args adds
# one -- cam_name already IS 'left_camera'/'right_camera'.
_USB_CAMERA_LEAVES = [
    ('image_raw', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
    ('camera_info', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo'),
]


def _usb_camera_bridge_args(cam_name, prefix, robot_id):
    """ros_gz_bridge args + remaps for one left_camera/right_camera (gz -> ROS only)."""
    gz_base = f'{prefix}{cam_name}'
    ros_base = f'/{robot_id}/{cam_name}' if robot_id else f'/{gz_base}'
    args = []
    remaps = []
    for leaf, ros_type, gz_type in _USB_CAMERA_LEAVES:
        args.append(f'/{gz_base}/{leaf}@{ros_type}[{gz_type}')
        if robot_id:
            remaps.append((f'/{gz_base}/{leaf}', f'{ros_base}/{leaf}'))
    return args, remaps


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
    # Two frame-mounted D435i cameras (fixed_cam1/fixed_cam2). Sim: bridges
    # their gz-sim sensor topics to ROS2. Real hardware: launches the actual
    # realsense2_camera driver for each. Either way, ignores the wrist
    # camera on purpose -- still the D435i mesh software-side for now, but
    # slated to become an Orbbec Gemini 2, wired up separately once that
    # camera's own ROS packages are in (see ranger_xarm6.urdf.xacro).
    enable_fixed_cameras = LaunchConfiguration('enable_fixed_cameras').perform(context).lower() in ('true', '1', 'yes')
    # Two SincereFirst USB camera modules (left_camera/right_camera, see
    # usb_camera_macro/usb_camera_sensor_tags in ranger_xarm6.urdf.xacro).
    # Sim: bridges their gz-sim camera sensor topics. Real hardware: launches
    # v4l2_camera_node per camera (ros-humble-v4l2-camera; NOT installed by
    # this repo, install separately -- 'sudo apt install ros-humble-v4l2-
    # camera' -- before running sim:=false with this on). UNTESTED against
    # physical hardware, same status as ranger_bringup/xarm_api elsewhere in
    # this launch file; left_camera_video_device/right_camera_video_device
    # below default to /dev/video0 and /dev/video2 as placeholders only --
    # confirm against the real unit's actual device nodes (`v4l2-ctl
    # --list-devices`) before trusting them.
    enable_usb_cameras = LaunchConfiguration('enable_usb_cameras').perform(context).lower() in ('true', '1', 'yes')
    # Wrist camera (Orbbec Gemini 2, see wrist_camera_sensor_tags in
    # ranger_xarm6.urdf.xacro; mesh stays the existing D435i+stand
    # placeholder, deliberately not swapped -- no combined Gemini2+xArm-
    # mount mesh exists, see this session's own mesh-recommendation
    # discussion). Sim: bridges its gz-sim sensor topics, same leaf set as
    # the fixed cameras (reuses _fixed_camera_bridge_args with cam_id
    # 'wrist', since wrist_camera_sensor_tags' topics were deliberately
    # shaped to match: '{prefix}wrist_camera/...'). Real hardware: includes
    # orbbec_camera's own gemini2.launch.py (ros-humble-orbbec-camera,
    # already installed here, unlike v4l2-camera above). UNTESTED against
    # physical hardware; gemini2.launch.py exposes 'cloud_frame_id' to
    # override the point cloud's frame_id (set below to match this URDF's
    # camera_depth_frame) but no equivalent override for color/infra1/
    # infra2's own frame_ids was found in its argument list -- those may
    # not line up with this URDF's frame names without further work once
    # real hardware is available to check against.
    enable_wrist_camera = LaunchConfiguration('enable_wrist_camera').perform(context).lower() in ('true', '1', 'yes')
    # HiPNUC HI14R3-232-000 IMU (see hipnuc_imu_link/hipnuc_imu_data_frame
    # in ranger_xarm6.urdf.xacro), on top of extras_link. Sim only for now:
    # bridges the gz IMU sensor to a plain sensor_msgs/Imu topic. Real
    # hardware isn't wired here yet -- the official HiPNUC ROS2 driver
    # (hipnuc/products, ros/ros2/hipnuc_imu) needs vendoring as a new git
    # submodule (its own CMakeLists requires the full repo checkout, not
    # just the ROS package folder), held pending confirmation rather than
    # added unprompted.
    enable_hipnuc_imu = LaunchConfiguration('enable_hipnuc_imu').perform(context).lower() in ('true', '1', 'yes')
    # Shared between sim and real hardware: RViz (RobotModel + TF + the two
    # fixed cameras' point clouds) is the "digital twin" viewer either way,
    # same URDF, same topics either way -- following the standard ROS/Nav2
    # pattern (robot_state_publisher + TF + RViz IS the digital twin; no
    # second Gazebo instance kept alive alongside real hardware, since
    # there's no first-class way to state-sync a live physics sim to a real
    # robot without building that bridge ourselves, and RViz+TF already
    # gives the same visualization for free).
    run_rviz = LaunchConfiguration('run_rviz').perform(context).lower() in ('true', '1', 'yes')
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

    # Shared digital-twin viewer (see the run_rviz comment above): namespaced
    # like robot_state_publisher_node, so the config's relative topics
    # (robot_description, fixed_cam{1,2}_camera/depth/points, ...) resolve
    # under this robot's namespace without hardcoding robot_id into the
    # .rviz file, whether that data came from Gazebo (sim) or the real
    # realsense2_camera driver (real hardware, sim:=false) -- both publish
    # under the same '/{robot_id}/fixed_cam{1,2}_camera/...' topic layout.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=robot_id,
        arguments=[
            '-f', f'{prefix}odom',
            '-d', PathJoinSubstitution(
                [FindPackageShare('ranger_xarm6_description'), 'config', 'ranger_xarm6.rviz']
            ),
        ],
        # Same use_sim_time as robot_state_publisher_node above: only true
        # when a '/clock' publisher actually exists (Gazebo, sim:=true).
        # Real hardware (sim:=false) has no '/clock', so this is False there
        # -- same reasoning as robot_state_publisher_node's own comment.
        parameters=[{'use_sim_time': sim and gazebo}],
        output='screen',
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

        if enable_fixed_cameras:
            camera_args = []
            camera_remaps = []
            for cam_id in ('fixed_cam1', 'fixed_cam2'):
                cam_args, cam_remaps = _fixed_camera_bridge_args(cam_id, prefix, robot_id)
                camera_args.extend(cam_args)
                camera_remaps.extend(cam_remaps)
            camera_bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=camera_args,
                remappings=camera_remaps,
                output='screen',
            )
            startup_actions.append(camera_bridge)

        if enable_usb_cameras:
            usb_args = []
            usb_remaps = []
            for cam_name in ('left_camera', 'right_camera'):
                cam_args, cam_remaps = _usb_camera_bridge_args(cam_name, prefix, robot_id)
                usb_args.extend(cam_args)
                usb_remaps.extend(cam_remaps)
            usb_camera_bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=usb_args,
                remappings=usb_remaps,
                output='screen',
            )
            startup_actions.append(usb_camera_bridge)

        if enable_wrist_camera:
            wrist_args, wrist_remaps = _fixed_camera_bridge_args('wrist', prefix, robot_id)
            wrist_camera_bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=wrist_args,
                remappings=wrist_remaps,
                output='screen',
            )
            startup_actions.append(wrist_camera_bridge)

        if enable_hipnuc_imu:
            # gz-sim's IMU sensor has no optical_frame_id-style override (see
            # fix_imu_frame_id.py's own docstring), so it always publishes
            # with an auto-generated, unresolvable scoped entity path as
            # header.frame_id. Bridge to a "_raw" topic first, then a small
            # republisher node fixes up the frame_id to the real TF frame
            # before anything (RViz's Imu display, etc.) consumes it.
            imu_gz_topic = f'{prefix}hipnuc_imu/data'
            imu_raw_topic = f'/{robot_id}/hipnuc_imu/data_raw' if robot_id else f'/{imu_gz_topic}_raw'
            imu_ros_topic = f'/{robot_id}/hipnuc_imu/data' if robot_id else f'/{imu_gz_topic}'
            imu_frame_id = f'{prefix}hipnuc_imu_data_frame'
            hipnuc_imu_bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[f'/{imu_gz_topic}@sensor_msgs/msg/Imu[gz.msgs.IMU'],
                remappings=[(f'/{imu_gz_topic}', imu_raw_topic)],
                output='screen',
            )
            hipnuc_imu_frame_fixup = Node(
                package='ranger_xarm6_description',
                executable='fix_imu_frame_id.py',
                arguments=[imu_raw_topic, imu_ros_topic, imu_frame_id],
                output='screen',
            )
            startup_actions.append(hipnuc_imu_bridge)
            startup_actions.append(hipnuc_imu_frame_fixup)

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

    fixed_camera_launches = []
    if enable_fixed_cameras:
        # Real hardware: the two frame-mounted D435i units, via the official
        # realsense2_camera driver (ros-humble-realsense2-camera). camera_name
        # matches the URDF's sensor_d435i 'name' param (${prefix}fixed_cam{1,2}_camera)
        # so its frame ids line up with robot_state_publisher's -- which is
        # exactly why publish_tf is turned off here: the URDF already
        # broadcasts that whole static tree from the same baked-in
        # extrinsics realsense2_description's sensor_d435i uses, so the
        # driver's own (duplicate) static TF would just fight it.
        # Distinguishing the two physical units needs each one's serial
        # number -- fixed_cam1_serial/fixed_cam2_serial launch args, REQUIRED
        # once both are plugged in simultaneously (unset, "first device
        # found" is a race between them).
        rs_launch_path = PathJoinSubstitution(
            [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
        )
        for cam_id, serial_arg in (('fixed_cam1', 'fixed_cam1_serial'), ('fixed_cam2', 'fixed_cam2_serial')):
            fixed_camera_launches.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rs_launch_path),
                launch_arguments={
                    # camera_namespace is what actually prefixes this node's
                    # (relative) topics -- camera_name does NOT additionally
                    # nest them. Bug caught before hardware ever saw it: an
                    # earlier version set camera_namespace to plain robot_id
                    # for BOTH cameras, so fixed_cam1 and fixed_cam2 would
                    # have collided on the same '/{robot_id}/color/image_raw'
                    # etc topics. This lands them on
                    # '/{robot_id}/{cam_id}_camera/...', matching exactly
                    # what the sim-side camera_bridge remaps gz's topics onto
                    # above, so RViz's config (relative topics, same either
                    # way) and any other consumer see identical topic names
                    # regardless of sim vs real.
                    'camera_namespace': f'{robot_id}/{cam_id}_camera' if robot_id else f'{cam_id}_camera',
                    'camera_name': f'{prefix}{cam_id}_camera',
                    'serial_no': LaunchConfiguration(serial_arg),
                    'enable_color': 'true',
                    'enable_depth': 'true',
                    'enable_infra1': 'true',
                    'enable_infra2': 'true',
                    'pointcloud.enable': 'true',
                    'publish_tf': 'false',
                }.items(),
            ))

    usb_camera_launches = []
    if enable_usb_cameras:
        # Real hardware: the two SincereFirst USB (UVC) modules, via
        # v4l2_camera_node (ros-humble-v4l2-camera -- NOT installed by this
        # repo, see enable_usb_cameras' own comment above). camera_frame_id
        # matches usb_camera_macro's own "${name}_optical_frame" link name
        # so the driver's published image/camera_info headers line up with
        # robot_state_publisher's TF, same reasoning as publish_tf:false on
        # the D435i's above -- except v4l2_camera_node never publishes TF of
        # its own to begin with (no depth/extrinsics to derive one from),
        # so there's no equivalent flag to turn off here.
        # namespace is what actually prefixes this node's (relative)
        # image_raw/camera_info topics, matching camera_namespace's role for
        # the D435i's above -- lands these on '/{robot_id}/{cam_name}/...',
        # exactly what the sim-side usb_camera_bridge remaps gz's topics
        # onto, so RViz's config and any other consumer see identical topic
        # names regardless of sim vs real.
        for cam_name, device_arg in (('left_camera', 'left_camera_video_device'), ('right_camera', 'right_camera_video_device')):
            usb_camera_launches.append(Node(
                package='v4l2_camera',
                executable='v4l2_camera_node',
                name=f'{prefix}{cam_name}',
                namespace=f'{robot_id}/{cam_name}' if robot_id else cam_name,
                parameters=[{
                    'video_device': LaunchConfiguration(device_arg),
                    'camera_frame_id': f'{prefix}{cam_name}_optical_frame',
                    'output_encoding': 'rgb8',
                }],
                output='screen',
            ))

    wrist_camera_launches = []
    if enable_wrist_camera:
        # Real hardware: the wrist-mounted Orbbec Gemini 2, via
        # orbbec_camera's own gemini2.launch.py (ros-humble-orbbec-camera,
        # already installed on this dev machine). That launch file uses its
        # own 'camera_name' arg as BOTH a PushRosNamespace value AND a
        # composable-node 'name=' field directly -- the latter can't
        # contain '/', so camera_name is kept a simple 'wrist_camera' (no
        # robot_id baked in) and the robot_id level is added by wrapping
        # the include in our own outer PushRosNamespace instead. Combined:
        # '/{robot_id}/wrist_camera/...', matching exactly what the sim-side
        # wrist_camera_bridge remaps gz's topics onto above.
        # publish_tf:false: same reasoning as the D435i's/'s above -- this
        # URDF's existing wrist camera frame chain (camera_link, from
        # xarm_device's add_realsense_d435i, plus the gemini_* extrinsics
        # frames orbbec_gemini2_extrinsics adds on top of it) already gets
        # broadcast by robot_state_publisher.
        # cloud_frame_id: pointed at gemini_depth_frame, NOT xarm's own
        # camera_depth_frame -- since this repo started using Orbbec's real
        # measured extrinsics for the wrist camera (see
        # orbbec_gemini2_extrinsics in ranger_xarm6.urdf.xacro) rather than
        # xarm_description's borrowed D435i ones, gemini_depth_frame is the
        # one that's actually correct for a real Gemini 2's point cloud.
        # Same "physical, not optical" reasoning as fixed_cam_sensor_tags'
        # own depth sensor for why *sim* uses the physical frame there;
        # this is real hardware though, where the real orbbec_camera driver
        # should already publish in the proper optical convention -- so if
        # this needs revisiting, it's specifically because that assumption
        # turned out wrong on the real device, not the sim-side quirk.
        # This is the one frame override gemini2.launch.py exposes; see the
        # enable_wrist_camera comment above for the color/infra1/infra2
        # caveat this doesn't cover.
        gemini2_launch_path = PathJoinSubstitution(
            [FindPackageShare('orbbec_camera'), 'launch', 'gemini2.launch.py']
        )
        gemini2_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gemini2_launch_path),
            launch_arguments={
                'camera_name': 'wrist_camera',
                'serial_number': LaunchConfiguration('wrist_camera_serial'),
                'enable_point_cloud': 'true',
                'publish_tf': 'false',
                'cloud_frame_id': f'{prefix}gemini_depth_frame',
            }.items(),
        )
        wrist_camera_launches.append(
            GroupAction([PushRosNamespace(robot_id), gemini2_include]) if robot_id else gemini2_include
        )

    return [
        robot_state_publisher_node,
        ros2_control_node,
        ranger_driver_launch,
        *fixed_camera_launches,
        *usb_camera_launches,
        *wrist_camera_launches,
        *([rviz_node] if run_rviz else []),
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
        DeclareLaunchArgument('run_rviz', default_value='true', description='sim and real hardware: launch RViz, the digital-twin viewer (RobotModel + TF + the two fixed cameras\' point clouds/images) either way'),
        DeclareLaunchArgument('enable_fixed_cameras', default_value='true', description='Bridge (sim) / launch realsense2_camera drivers (real) for the two frame-mounted D435i cameras (fixed_cam1/fixed_cam2). Wrist camera (Gemini 2, eventually) not covered by this flag.'),
        DeclareLaunchArgument('fixed_cam1_serial', default_value="''", description="real hardware only: fixed_cam1's D435i serial number (REQUIRED once both fixed cameras are plugged in together)"),
        DeclareLaunchArgument('fixed_cam2_serial', default_value="''", description="real hardware only: fixed_cam2's D435i serial number (REQUIRED once both fixed cameras are plugged in together)"),
        DeclareLaunchArgument('enable_usb_cameras', default_value='true', description='Bridge (sim) / launch v4l2_camera_node (real, needs ros-humble-v4l2-camera installed separately) for the two SincereFirst USB modules (left_camera/right_camera).'),
        DeclareLaunchArgument('left_camera_video_device', default_value='/dev/video0', description="real hardware only: left_camera's V4L2 device node -- confirm with `v4l2-ctl --list-devices` on the real unit, this default is a placeholder"),
        DeclareLaunchArgument('right_camera_video_device', default_value='/dev/video2', description="real hardware only: right_camera's V4L2 device node -- confirm with `v4l2-ctl --list-devices` on the real unit, this default is a placeholder"),
        DeclareLaunchArgument('enable_wrist_camera', default_value='true', description='Bridge (sim) / launch orbbec_camera (real, ros-humble-orbbec-camera) for the wrist-mounted Orbbec Gemini 2. Mesh stays the D435i+stand placeholder.'),
        DeclareLaunchArgument('wrist_camera_serial', default_value='', description='real hardware only: wrist camera Gemini 2 serial number (only needed if multiple Orbbec devices are ever present at once)'),
        DeclareLaunchArgument('enable_hipnuc_imu', default_value='true', description='Bridge the gz-sim IMU sensor for the HiPNUC HI14R3-232-000 (mounted on extras_link). Sim only for now -- real hardware driver not wired yet.'),
        OpaqueFunction(function=launch_setup),
    ])
