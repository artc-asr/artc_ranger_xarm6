# ranger_xarm6_navigation

Odometry, mapping, localization and navigation for the Ranger Mini 3.0 +
xArm6. Built so far: the EKF (`odom -> base_link`), FAST-LIO2 3D
mapping plus the front D435i's low obstacles, a 2D grid exported from
both, and Nav2 with the lidar and the front D435i in its costmaps (tested
in sim, with a static `map -> odom`). To come: 3D NDT scan-to-map
localization (`map -> odom`).

| Piece | What it does |
|---|---|
| `launch/odometry.launch.py` | robot_localization EKF over the Ranger's wheel odometry and the Mid-360's built-in IMU; publishes `odometry/filtered` and `odom -> base_footprint -> base_link`. |
| `scripts/ekf_inputs.py` | Makes the raw driver messages fusable (see below). |
| `launch/mapping.launch.py` | FAST-LIO2 mapping session on the Mid-360 + its IMU; `map_save` writes the 3D map (`.pcd`), `depth_mapper.py` the front D435i's low obstacles (`<map>_depth.pcd`). |
| `scripts/depth_mapper.py` | The front depth cloud, placed with FAST-LIO's pose, cut to a height band against the floor under the robot. |
| `scripts/pcd_to_grid.py` | The `.pcd` map (+ `<map>_depth.pcd`) -> a 2D occupancy grid (map_server `.pgm` + `.yaml`). |
| `launch/navigation.launch.py` | Nav2 on that grid: NavFn, RotationShim + Regulated Pure Pursuit, lidar + front depth voxel costmaps, velocity smoother, collision monitor. |
| `src/cloud_self_filter.cpp` | The lidar cloud without the robot (arm), for the collision monitor. |
| `src/obstacle_memory.cpp` | What the front camera has seen, kept after it's out of view, for the collision monitor. |
| `launch/front_depth.launch.py` | `depth_to_cloud` for the front D435i; included by mapping and navigation. |
| `src/depth_to_cloud.cpp` | A depth image -> a small cloud (5 cm voxels within 3 m), released once TF can place it. |
| `src/livox_cloud_to_custom.cpp` | The Livox driver's `PointCloud2` -> the Livox `CustomMsg` FAST-LIO reads. |

## Quick start

Build (once, and after pulling):

```bash
cd ~/Worksplace/artc_ranger_xarm6
colcon build --symlink-install --packages-select ranger_xarm6_description ranger_xarm6_navigation
source install/setup.bash
```

Every terminal below: `source install/setup.bash`. Recommended, in every
terminal of a session (sim, Nav2, tools alike):
`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. With the default FastDDS,
Nav2 hung at startup in 2 of 8 sim launches (a lifecycle `change_state`
response got lost); with Cyclone 8 of 8 came up. If a FastDDS session
misbehaves after crashes, `fastdds shm clean` removes its stale shared
memory.

**1. Map a room (sim).**

```bash
# T1: the robot in the lab (RViz off; Gazebo headless)
ros2 launch ranger_xarm6_description ranger_xarm6.launch.py run_rviz:=false \
  world:=artc_lab.world x:=0.94 y:=4.35 yaw:=-1.5708
# T2: FAST-LIO2 + the front camera's low obstacles
ros2 launch ranger_xarm6_navigation mapping.launch.py map:=~/ranger_xarm6_maps/lab.pcd
# T3: drive, smoothly, arm parked, camera facing what's low (teleop, or Nav2 on an old map)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot_a/cmd_vel
# when done: save the 3D map, then Ctrl-C T2 (writes lab_depth.pcd)
ros2 service call /robot_a/map_save std_srvs/srv/Trigger
```

**2. Make the 2D grid.** `--origin` is the IMU's pose when mapping
started, in the map frame (see "2D grid" below; this one is for the
spawn above):

```bash
ros2 run ranger_xarm6_navigation pcd_to_grid.py ~/ranger_xarm6_maps/lab.pcd \
  --origin 0.917 4.177 0.793 1.5708
```

**3. Navigate (sim).**

```bash
# T1 as above; T2:
ros2 launch ranger_xarm6_navigation navigation.launch.py map:=~/ranger_xarm6_maps/lab.yaml
```

Wait for `Managed nodes are active` twice, then **2D Goal Pose** in RViz
(the red/orange boxes in front of the robot are the collision monitor's
stop and slowdown zones), or `ros2 action send_goal` (see "Navigation").
Obstacles to try: spawn a box in front of the robot in Gazebo, e.g.

```bash
gz service -s /world/artc_lab/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'sdf: "<sdf version=\"1.9\"><model name=\"box\"><static>true</static><pose>3 3 0.075 0 0 0</pose><link name=\"l\"><collision name=\"c\"><geometry><box><size>0.3 0.3 0.15</size></box></geometry></collision><visual name=\"v\"><geometry><box><size>0.3 0.3 0.15</size></box></geometry></visual></link></model></sdf>"'
```

**4. Real robot** (untested on hardware). Wheel odometry + IMU through the
EKF (so heights are right), `use_sim_time:=false`, no static map -> odom
until the NDT localizer exists (odometry drift isn't corrected):

```bash
ros2 launch ranger_xarm6_description ranger_xarm6.launch.py sim:=false publish_odom_tf:=false \
  robot_ip:=192.168.1.231 can_device:=can0 fixed_cam1_serial:=<front D435i serial>
ros2 launch ranger_xarm6_navigation odometry.launch.py use_sim_time:=false
ros2 launch ranger_xarm6_navigation mapping.launch.py use_sim_time:=false map:=~/ranger_xarm6_maps/lab.pcd
```

Check before driving: the Mid-360 is PTP-synced (or `imu_restamp`), the
front camera's depth topic matches `front_depth.launch.py`'s default
(`ros2 topic list | grep image_rect_raw`; override with
`front_depth_image:=`/`front_depth_info:=`), the Ranger driver stops the
base when `cmd_vel` stops coming (the collision monitor's last line of
defence when Nav2 shuts itself down), and the first obstacle tests are
slow, with a hand on the e-stop.

## Sensors

| Sensor | Topic | Used for |
|---|---|---|
| Livox Mid-360 | `livox/lidar` (`PointCloud2`, the driver's `xfer_format: 0` layout, per-point `timestamp`) | mapping, localization |
| Mid-360 built-in IMU (ICM-40609) | `livox/imu` (acceleration in g, frame `livox_frame`, as the driver publishes) | EKF (gyro), mapping |
| Ranger wheel odometry | `odom` (`ranger_base`) | EKF |
| Front D435i (`fixed_cam1`) | depth image + `camera_info` (real: `fixed_cam1_camera/robot_a_fixed_cam1_camera/depth/image_rect_raw`, 848x480 decimated to 424x240, 15 Hz; sim: `fixed_cam1_camera/depth/depth_image`) | Nav2 costmaps, low obstacles in the map |

Not used:
- **Rear D435i** (`fixed_cam2`): the arm sits inside its minimum range,
  so part of every image is noise. Nav2 never reverses (it spins in place
  instead), so it adds little; if it's wanted, the fix is a fixed pixel
  mask for the parked arm's patch in `depth_to_cloud` (recorded on the
  real camera: the sim doesn't reproduce stereo noise).
- **The arm's Gemini**: it moves with the arm.
- **HiPNUC IMU**: no real driver wired yet (and its sim is a Gazebo IMU on
  the teleported base, the flaw `sim_livox_imu.py` avoids). It's the
  better gyro; once its driver is in, it can replace or join the
  Mid-360's in the EKF. The D435i's IMUs add nothing over these.

In sim, `ranger_xarm6_description` publishes the same topics in the same
formats (`sim_livox_imu.py`, `base_pose_publisher.py`,
`gz_lidar_to_pointcloud.py`, the depth camera), plus `ground_truth/odom`.

## Odometry (EKF)

```bash
ros2 launch ranger_xarm6_description ranger_xarm6.launch.py publish_odom_tf:=false \
  world:=artc_lab.world x:=0.94 y:=4.35 yaw:=-1.5708
ros2 launch ranger_xarm6_navigation odometry.launch.py x:=0.94 y:=4.35 yaw:=-1.5708
```

`publish_odom_tf:=false` stops the base's own `odom -> base_link` (sim:
ground truth; real: the Ranger driver's) so the EKF owns it. `x/y/yaw` is
where odometry starts: the spawn pose in sim (odom is Gazebo's world
frame), `0 0 0` on hardware.

The EKF estimates `odom -> base_footprint` (base_link on the floor, z 0
in 2D mode), and a static `base_footprint -> base_link` lifts it by
`base_height` (0.315 m). Before, the 2D EKF put base_link itself at z 0,
0.315 m lower than the sim's ground truth: Nav2 measures obstacle heights
in odom (every height band would have been off by 0.315 m, the floor-level
ones lost), and MoveIt places the world's objects in it.

- **Fused:** wheels vx, vy, vyaw; gyro vyaw with a much lower variance, so
  it carries the heading (a swerve base's wheels scrub when it spins).
  2D, 50 Hz. The accelerometer isn't fused.
- **`ekf_inputs.py`:** IMU acceleration g -> m/s^2; its unprefixed
  `livox_frame` -> `<robot_id>_livox_imu_frame`; real covariances instead of
  the drivers' all-zero ones (which robot_localization treats as
  near-certain); gyro bias estimated while the wheels read still;
  `imu_restamp` for a Mid-360 that isn't PTP-synced.
- **Sim result** (4 m square of crabs and 90 deg spins, an arc, a sideways
  crab): EKF 3.6 cm / 0.25 deg from ground truth; wheel odometry alone
  14 cm / 13.4 deg.

## Mapping (FAST-LIO2)

```bash
ros2 launch ranger_xarm6_navigation mapping.launch.py map:=~/ranger_xarm6_maps/lab.pcd
# drive around with the arm stowed (or at home), then:
ros2 service call /robot_a/map_save std_srvs/srv/Trigger
```

The map is in FAST-LIO's `camera_init` frame (the IMU's pose at start,
gravity-aligned). Outputs are under `fast_lio/` in the robot's namespace:
`odometry`, `path`, `cloud_registered`, `laser_map`.

- **Why the converter:** FAST-LIO reads per-point times only from the
  Livox `CustomMsg`, but the driver publishes one format, and RViz/Nav2
  want `PointCloud2`. So `livox/lidar` stays `PointCloud2` and
  `livox_cloud_to_custom` feeds FAST-LIO.
- **Config** (`config/fast_lio.yaml`): IMU-lidar extrinsic from the Livox
  manual (not estimated), 0.25 m voxels, 40 m range, `blind: 0.7` m to drop
  the arm (0.27-0.67 m from the lidar at home/stow). An arm reaching
  elsewhere during mapping gets mapped.
- **Drive smoothly.** In sim the base follows `cmd_vel` instantly; an
  instant 0 -> 0.3 m/s step spiked FAST-LIO's pose by 16 cm and doubled
  walls. Ramp commands (a joystick does); the real base ramps anyway.
- **Low obstacles** (`front_depth:=true`, the default): `depth_mapper.py`
  places the front D435i's cloud (`depth_to_cloud`: 5 cm voxels within
  3 m) in `camera_init` with FAST-LIO's pose at each depth frame, keeps
  points 0.05-1.5 m above the floor *under the robot* (base_link's height,
  not FAST-LIO's drifting z), skips frames while turning faster than 0.6
  rad/s, and writes the voxels seen in >= 3 frames to `<map>_depth.pcd`
  every 10 s and on exit. Drive so the camera faces what's low: it sees
  3 m ahead, 87 deg wide.

### How good is the map (sim, artc_lab)

26.6 m loop, crabs and spins, arm stowed, scored against the world file:

| | |
|---|---|
| Trajectory | 2.0 cm RMSE (max 9.6 cm) |
| Map points to true surfaces | median 1.6 cm, 95% 4.7 cm, none > 10 cm |
| 2D projection, z 0.15-1.5 m | 88-100% of each obstacle's footprint (cupboard 76%), 1.6% spurious cells |
| Surface coverage by height | 0-0.3 m: 53%, 0.3-0.6: 73%, 0.6-0.9: 83%, 0.9-2: 83% |

The lidar is at ~0.84 m, above the table tops (0.71-0.75 m); with the
Mid-360's -7 deg lowest beam it sees the floor only beyond 6.8 m and a
0.15 m-tall object only beyond 5.6 m. The map still has every obstacle's
footprint, because the lab's obstacles have tops the lidar sees; what it
can't provide is low, free-standing obstacles near the robot: that's the
front D435i's job in Nav2.

The 2D projection has to start at ~0.15 m (or remove the ground first):
FAST-LIO's height drifts ~6 cm, which lifts the far floor ring into a
0.05 m band as a false wall.

Sim caveats: the simulated lidar scans a regular grid and captures a scan
at one instant (no motion distortion); the real Mid-360's non-repetitive
pattern is denser over time and distorted by motion (FAST-LIO undistorts
it). On hardware the Mid-360 needs PTP time sync with the host.

## 2D grid

```bash
ros2 run ranger_xarm6_navigation pcd_to_grid.py ~/ranger_xarm6_maps/artc_lab.pcd \
  --origin 0.917 4.177 0.793 1.5708
```

`--origin` is FAST-LIO's `camera_init` (the Mid-360 IMU's pose when
mapping started) in the map frame: x y z yaw, z its height above the
floor. In sim the map frame is Gazebo's world, and this is the spawn pose
above (0.94 4.35, yaw -90 deg) composed with `base_link ->
robot_a_livox_imu_frame` (0.173, -0.023, 0.478, yaw 180 deg); base_link
is 0.315 m above the floor. Writes `artc_lab.pgm` + `artc_lab.yaml` next
to the `.pcd`.

A cell is occupied when >= 2 map points lie 0.15-1.5 m above the floor
in it (the robot's height band; above the floor ring FAST-LIO's height
drift lifts), or when `<map>_depth.pcd` (picked up automatically; `--depth
''` to leave it out) has a voxel in it: those were cut to 0.05-1.5 m when
recorded, so the lidar's 0.15 m floor doesn't apply to them. The artc_lab grid (5 cm cells): every wall, table, the
cupboard and the plant, median 0 cm from the world file's surfaces; 5% of
cells fatten edges by 10-20 cm, none stray into the free space.

## Navigation (Nav2)

```bash
ros2 launch ranger_xarm6_description ranger_xarm6.launch.py run_rviz:=false \
  world:=artc_lab.world x:=0.94 y:=4.35 yaw:=-1.5708
ros2 launch ranger_xarm6_navigation navigation.launch.py map:=~/ranger_xarm6_maps/artc_lab.yaml
```

Then **2D Goal Pose** in its RViz, or:

```bash
ros2 action send_goal /robot_a/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: robot_a_map}, pose: {position: {x: 7.5, y: 2.5}, orientation: {w: 1.0}}}}"
```

Park the arm (`home` or `stow`) first: the footprint covers the arm only
there (x -0.53..0.38, y +-0.27 m around base_link; the gripper reaches
0.49 m behind it at home). Real hardware: `use_sim_time:=false`, and the
front camera's topics default to realsense2_camera's
(`front_depth_image`/`front_depth_info` override them).

- **map -> odom**: no localizer yet; `static_map_to_odom:=true` (default)
  publishes an identity, right in sim (odom = Gazebo world = the grid's
  frame). Odometry drift isn't corrected, so not for real hardware.
- **Base motion**: RotationShim spins in place to within 45 deg of the
  path, Regulated Pure Pursuit follows it forward (no reversing), then a
  spin to the goal heading. vy is always 0: Nav2 never crabs; the final
  crab/arm approach is `ranger_xarm6_manipulation`'s job. 0.3 m/s, 0.4
  rad/s, ramped by the velocity smoother. The real driver turns v/w <
  0.476 m into a spin in place; the sim base follows any twist exactly.
- **Costmaps**: the static grid plus, in a VoxelLayer (a 2D ObstacleLayer
  clears an obstacle with every ray that passes over it):
  - the Mid-360 cloud, 0.15-1.5 m, out to 6 m;
  - the front D435i (`depth_to_cloud`: 5 cm voxels within 3 m), marking
    0.05-1.5 m; and the same cloud floor included as a clearing-only
    source (a source's height band filters its clearing rays too, and
    it's the rays to the floor that clear an obstacle that has gone).

  Local costmap 8x8 m around the robot.
- **Collision monitor** (`collision_monitor`, last in the chain:
  velocity_smoother -> `cmd_vel_smoothed` -> collision_monitor ->
  `cmd_vel`): stops or slows the base from the raw sensors on its own,
  whatever the planner, controller and costmaps think. The backstop for
  a controller that has lost the robot's pose and keeps sending its last
  command. Zones (base_link, heights 0.05-1.5 m above the floor):
  - stop: a strip 0.42-0.55 m ahead (the bumper is at 0.38; a stop zone
    all round would also stop the base turning away from something beside
    it);
  - slowdown to 30%: 0.42-0.95 m ahead, 0.8 m wide;
  - approach: the footprint moved along the current command, any
    direction, spins included; the command is scaled so that a hit is
    never less than 1.2 s away.

  Sources: the Mid-360 through `cloud_self_filter` (it sees the arm,
  which would sit in every zone: points inside the published footprint +
  1 cm are dropped), the front depth cloud, and `obstacle_memory`: what
  the front camera has seen, kept in odom after it's out of view. The
  monitor itself only looks at the latest data, and a low obstacle is
  below the camera's view by the time it's in a zone. A remembered spot
  is forgotten when the camera sees past it (obstacle gone), after 30 s,
  beyond 3 m, or inside the footprint. The strip right in front of the
  bumper is below the camera's view: something remembered there is only
  forgotten by age (the base waits up to 30 s) or once the base backs
  off and the camera sees the spot again. It publishes only while
  commands come in, so teleop and manipulation's `cmd_vel` are untouched.
- **Why `depth_to_cloud` and not the camera's cloud**: a 424x240 cloud is
  ~100k points (2-3 MB, 15 Hz); 5 cm voxels within 3 m are ~5k. And it
  releases each cloud only once TF can place it in odom (+50 ms): the
  costmaps' tf2 MessageFilter otherwise hands a cloud that's ahead of TF
  to the costmap from inside the TF buffer's callback, and in sim that
  froze the local costmap's TF within a few runs (the controller lost the
  robot's pose, and the base drove on its last command).
- **Tolerance**: 10 cm / 0.1 rad at the goal.
- `config/nav2.yaml` is written for `robot_id` robot_a; the launch
  replaces `robot_a` and nests the params under the namespace. Not
  nav2_bringup's launch: it remaps `/tf` into the namespace, and this
  robot's TF is global.

### Sim results (artc_lab, ground truth)

Lidar only (before the front camera):

| Goal (x, y, yaw) | Result | Sim time | Final error | Min clearance |
|---|---|---|---|---|
| spawn -> 7.5, 2.5, 0 | reached | 27 s | 9.8 cm / 4.9 deg | 32 cm |
| -> 3.2, 1.6, 180 | reached | 25 s | 9.8 cm / 4.5 deg | 54 cm |
| -> 0.94, 4.35, -90 | reached | 21 s | 9.9 cm / 4.1 deg | 34 cm |
| 0.3 x 0.3 x 0.4 m box, not in the map, seen from 3.3 m | reached | 29 s | 9.7 cm / 5.1 deg | 38 cm |
| same box, 2.9 m away when Nav2 started | reached, **through the box** | 40 s | | 0 cm |

Lidar + front D435i, spawn -> 7.5, 2.5, 0, boxes not in the map placed on
the planned path (each on a fresh sim):

| Box | Result | Sim time | Final error | Min clearance |
|---|---|---|---|---|
| none | reached | 28 s | 9.6 cm / 4.5 deg | |
| 0.3 x 0.3 x 0.4 m, 2.9 m along the path | reached | 28 s | 9.7 cm / 5.2 deg | 39 cm |
| 0.3 x 0.3 x 0.15 m, 2.0 m along the path (x5) | reached | 29 s | 9.5-9.9 cm / 3.7-5.3 deg | 35-39 cm |
| 0.3 x 0.3 x 0.3 m, dropped 1.5 m ahead while driving | reached | 30 s | 9.6 cm / 4.8 deg | 33 cm |
| 0.3 x 0.3 x 0.15 m, dropped 1.8 m ahead while driving | reached | 29 s | 9.9 cm / 4.7 deg | 36 cm |

Collision monitor (fixed command published straight into it, i.e. no
planner or controller; `stop`/`slowdown`/`approach` as configured above):

| Test | Result |
|---|---|
| 0.3 m/s straight at a wall | slowed, stopped 23 cm from it |
| 0.15 m/s straight at a wall | stopped 25 cm from it |
| 0.3 m/s at a table's side (tabletop 0.73 m, legs only at the corners) | stopped 16 cm from the tabletop edge (camera) |
| spin 0.4 rad/s, side 18 cm from a wall | approach cut the spin to 0.12 rad/s, stopped after 12 deg, 11 cm from the wall |
| Nav2 goal, controller frozen, its last command (0.3 m/s) kept coming | stopped 24 cm from the wall |
| 0.3 m/s at a 0.15 m box (lidar can't see it; the camera only from 1.25 m) | stopped 17 cm from it (`obstacle_memory`; without it, drove into it) |
| same, box removed once the base had stopped | went on 21 s later (the memory aged out), stopped at the wall |
| Nav2 box runs above, with the monitor in the chain | all reached, 28-29 s, 34-40 cm; no monitor action |

A frozen (not just stuck) Nav2 node trips the lifecycle manager's bond
(~4 s): it deactivates the whole stack, collision monitor included, and
nothing publishes `cmd_vel` any more. The sim base stops after 0.5 s
without commands; the real base's own command timeout is what stops it
then.

**Remaining gap: the camera's near blind zone.** The front D435i is 0.69
m up and level, 87 x 58 deg: it sees the floor from ~1 m ahead of the
lens, a 0.15 m-tall obstacle from 1.0 m, a 0.4 m one from 0.55 m.
What it marked on the way in stays marked; an obstacle that appears
closer than that (or beside the robot while it spins in place) isn't
seen. Tilting the camera down ~15 deg would bring the floor in to ~0.6 m.
