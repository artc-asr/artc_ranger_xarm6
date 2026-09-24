# artc_ranger_xarm6

Robot-platform packages for the **Ranger Mini 3.0 (AgileX / Weston Robot) + xArm6** mobile manipulator, built for ARTC. This repo holds the combined URDF/xacro description, Gazebo simulation bringup, and real-hardware bringup — everything needed to get the robot moving in sim or on real hardware, independent of any particular control algorithm.

Application code (e.g. [wbcc_mm](https://github.com/artc-asr/whole_body_compliance_control_mm), the whole-body compliance controller) lives in a separate repo and pulls this one in as a dependency, the same way this repo pulls in the vendor description/driver packages below.

## Contents

| Package | Description |
|---|---|
| `ranger_xarm6_description` | Combined description + bringup launch. Joins the base and arm through a fixed mount transform; `ranger_xarm6.launch.py` supports both Gazebo Harmonic simulation and real hardware (`sim:=true/false`). |
| `ranger_mini_v3_description` | Vendored — missing from upstream `ranger_ros2` for ROS 2 Humble at the time this was ported. |
| `xarm_ros2` (submodule) | UFACTORY xArm6 description, ros2_control, and driver packages. |
| `westonrobot_ranger_ros2` (submodule) | Ranger Mini 3.0 real-hardware bringup/driver. |
| `ugv_sdk` (submodule) | Weston Robot UGV SDK — `ranger_ros2`'s driver dependency. |
| `gz_ros2_control` (submodule, `humble` branch) | Built from source with `GZ_VERSION=harmonic` (see below) — the `ros-humble-gz-ros2-control` **apt** package is built against Fortress (`libignition-gazebo6`) regardless of what's installed locally, so on a Harmonic system its plugin exports the wrong ABI symbol (`IgnitionPluginHook` instead of `GzPluginHook`) and Gazebo silently fails to load it, which cascades into `controller_manager` never starting and `joint_state_broadcaster`/`arm_velocity_controller` never spawning. Building this submodule locally (the main `colcon build` below already does, since `GZ_VERSION=harmonic` is exported first) overlays a correctly-linked version. |
| `Livox-SDK2` (submodule) | Livox's low-level lidar SDK (Mid-360 support). No apt package exists; built from source to a **user-writable prefix** (`$HOME/.local`, not `/usr/local` via `sudo make install` as Livox's own README suggests) — see below. |
| `livox_ros_driver2` (submodule) | ROS 2 driver for the Mid-360, built against `Livox-SDK2` above. Its `package.xml` is intentionally **not** committed upstream (gitignored in that submodule, since the same source tree serves both ROS1 and ROS2 via templated `package_ROS1.xml`/`package_ROS2.xml`) — regenerate it after every fresh clone, see below. |

## Installation

> **Note:** targets **ROS 2 Humble / Ubuntu 22.04**, paired with **Gazebo Harmonic** (not the default Fortress pairing for Humble).

```bash
# Clone
cd ~/Worksplace
git clone --recurse-submodules git@github.com:artc-asr/artc_ranger_xarm6.git
cd artc_ranger_xarm6

# Gazebo Harmonic (OSRF apt repo — Harmonic isn't in Ubuntu 22.04's default archives)
sudo apt-get update && sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update && sudo apt-get install gz-harmonic

# ROS dependencies
# WARNING: ros-humble-ros-gz* conflicts with ros-humble-ros-gzharmonic — remove it first if installed.
sudo apt-get remove -y 'ros-humble-ros-gz*' || true
sudo apt install ros-humble-ros-gzharmonic

# Camera drivers + description packages (RealSense D435i x2, Orbbec Gemini 2 on the wrist)
sudo apt-get install -y \
  ros-humble-realsense2-description ros-humble-realsense2-camera \
  ros-humble-orbbec-description ros-humble-orbbec-camera

rosdep install --from-paths . --ignore-src -r -y \
  --skip-keys "gz_sim_vendor sdformat_vendor gz_transport_vendor gz_msgs_vendor"

# Livox-SDK2 (Mid-360 lidar): no apt package, build from source. Installed to
# $HOME/.local (NOT /usr/local via sudo make install, as Livox's own README
# suggests) so the rest of this setup needs no sudo; livox_ros_driver2's
# CMakeLists.txt hardcodes /usr/local/lib as a find_library() HINT but still
# also searches CMAKE_PREFIX_PATH (passed to colcon build below), which is
# how it's actually found.
cmake -S Livox-SDK2 -B Livox-SDK2/build -DCMAKE_INSTALL_PREFIX=$HOME/.local -DCMAKE_BUILD_TYPE=Release
cmake --build Livox-SDK2/build -j$(nproc)
cmake --install Livox-SDK2/build

# livox_ros_driver2's package.xml is gitignored upstream (see Contents table
# above) -- regenerate it from the ROS2 template before every fresh clone's
# first build.
cp livox_ros_driver2/package_ROS2.xml livox_ros_driver2/package.xml

# Build
export GZ_VERSION=harmonic
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  -DCMAKE_PREFIX_PATH=$HOME/.local \
  --packages-skip xarm_moveit_servo xarm_planner \
  gz_ros2_control_demos gz_ros2_control_tests ign_ros2_control ign_ros2_control_demos \
  --allow-overriding gz_ros2_control

# Source
source install/setup.bash
```

> **Note:** `xarm_moveit_servo`/`xarm_planner` are skipped — `xarm_moveit_servo` fails to build against Humble's older `moveit_msgs` (missing `servo_command_type.hpp`, a newer MoveIt Servo API only present from Jazzy+).
>
> **Note:** don't run `livox_ros_driver2/build.sh` directly — it assumes the
> `ws_livox/src/livox_ros_driver2` layout from Livox's own clone
> instructions and does `rm -rf ../../build/` etc. as a "clean slate" step,
> which in this flat-submodule layout resolves to this **entire repo's**
> `build/` directory. The steps above do the same substitution
> (`package_ROS2.xml` -> `package.xml`) safely, without that.
>
> **Note:** `gz_ros2_control_demos`/`gz_ros2_control_tests`/`ign_ros2_control`/`ign_ros2_control_demos` (siblings of `gz_ros2_control` inside that same submodule) are skipped too — only `gz_ros2_control` itself is needed here (see the Contents table above for why it's built from source at all), and the demo/test packages pull in extra controller deps (`control_toolbox`, `ackermann_steering_controller`, `mecanum_drive_controller`, `tricycle_controller`, ...) this repo doesn't otherwise need.

## Quick Start

```bash
# Simulation (Gazebo Harmonic + robot)
ros2 launch ranger_xarm6_description ranger_xarm6.launch.py sim:=true

# Real hardware
ros2 launch ranger_xarm6_description ranger_xarm6.launch.py sim:=false
```

Then, optionally, a controller on top (MoveIt 2 planning for base + arm, sequential or whole-body; see [`ranger_xarm6_manipulation`](ranger_xarm6_manipulation/README.md)):

```bash
ros2 launch ranger_xarm6_description ranger_xarm6.launch.py run_rviz:=false world:=artc_lab.world x:=0.94 y:=4.35 yaw:=-1.5708
ros2 launch ranger_xarm6_manipulation control.launch.py controller:=moveit_sequential   # or moveit_whole_body; --show-args lists them
```

Either way the arm is exposed as an `arm_velocity_controller` joint group, so downstream control code drives it identically in sim or on hardware. Base drive on real hardware isn't wired up yet (`ranger_bringup`/`ranger_ros2`, vendored, untested); in simulation it's driven by `ranger_xarm6_description/scripts/base_pose_publisher.py`, a kinematic (not physics-based) stand-in that integrates `cmd_vel` and teleports the Gazebo entity, since Ranger's wheels have no `ros2_control` command interface.

## Using this repo from another workspace

Add it as a submodule (or a vcstool `.repos` entry) under your workspace's `src/`:

```bash
git submodule add git@github.com:artc-asr/artc_ranger_xarm6.git src/artc_ranger_xarm6
git submodule update --init --recursive
```

`colcon build` discovers all packages recursively, so no further wiring is needed.
