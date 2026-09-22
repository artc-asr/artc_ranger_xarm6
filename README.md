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
rosdep install --from-paths . --ignore-src -r -y \
  --skip-keys "gz_sim_vendor sdformat_vendor gz_transport_vendor gz_msgs_vendor"

# Build
export GZ_VERSION=harmonic
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip xarm_moveit_servo xarm_planner

# Source
source install/setup.bash
```

> **Note:** `xarm_moveit_servo`/`xarm_planner` are skipped — `xarm_moveit_servo` fails to build against Humble's older `moveit_msgs` (missing `servo_command_type.hpp`, a newer MoveIt Servo API only present from Jazzy+).

## Quick Start

```bash
# Simulation (Gazebo Harmonic + robot)
ros2 launch ranger_xarm6_description ranger_xarm6.launch.py sim:=true

# Real hardware
ros2 launch ranger_xarm6_description ranger_xarm6.launch.py sim:=false
```

Either way the arm is exposed as an `arm_velocity_controller` joint group, so downstream control code drives it identically in sim or on hardware. Base drive on real hardware isn't wired up yet (`ranger_bringup`/`ranger_ros2`, vendored, untested); in simulation it's driven by `ranger_xarm6_description/scripts/base_pose_publisher.py`, a kinematic (not physics-based) stand-in that integrates `cmd_vel` and teleports the Gazebo entity, since Ranger's wheels have no `ros2_control` command interface.

## Using this repo from another workspace

Add it as a submodule (or a vcstool `.repos` entry) under your workspace's `src/`:

```bash
git submodule add git@github.com:artc-asr/artc_ranger_xarm6.git src/artc_ranger_xarm6
git submodule update --init --recursive
```

`colcon build` discovers all packages recursively, so no further wiring is needed.
