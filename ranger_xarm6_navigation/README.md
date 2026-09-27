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
| `src/obstacle_memory.cpp` | What the front camera has seen, kept after it's out of view, for the collision monitor and the costmaps. |
| `launch/front_depth.launch.py` | `depth_to_cloud` for the front D435i; included by mapping and navigation. |
| `src/depth_to_cloud.cpp` | A depth image -> a small cloud (5 cm voxels within 3 m), released once TF can place it. |
| `src/livox_cloud_to_custom.cpp` | The Livox driver's `PointCloud2` -> the Livox `CustomMsg` FAST-LIO reads. |

## How to run

Step by step, with mapping, navigation, MoveIt and the gripper: the
[usage guide in the repo's README](../README.md#usage-guide). The
sections below are the reference: what each piece does and how it was
tested.

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
  `cmd_vel`): slows or stops the base from the raw sensors on its own,
  whatever the planner, controller and costmaps think. The backstop for
  a controller that has lost the robot's pose and keeps sending its last
  command. One zone, **approach**: the local costmap's footprint (arm
  included, + 3 cm) moved along the current command, in any direction,
  spins included; the command is scaled so that a hit is never less than
  1.2 s away. No stop or slowdown zone: in this Nav2 both act on every
  command whatever its direction, so a stop zone kept the base from
  backing away from what stopped it, and a slowdown zone held turns at
  ~0.05 rad/s (RotationShim accelerates from the measured rate).

  Sources (0.05-1.5 m above the floor): the Mid-360 through
  `cloud_self_filter` (it sees the arm, which would always be "in
  collision"), the front depth cloud, and `obstacle_memory`.
- **`obstacle_memory`**: what the front camera has seen, kept in odom
  after it's out of view (a low obstacle is below the camera's view by
  the time it's close). Also a marking source in both costmaps: Nav2's
  recoveries clear the costmaps, and without it the planner then routed
  through a box the camera could no longer see. A remembered spot is
  forgotten when the camera sees past it (obstacle gone), after 60 s if
  more than 1 m from the robot, beyond 3 m, or well inside the body. Near
  the robot it's never forgotten by age: a false one there doesn't trap
  the base (approach still lets it turn or back away; backing off lets
  the camera look again).
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
planner or controller):

| Test | Result |
|---|---|
| 0.3 m/s straight at a wall | stopped 14 cm from it |
| 0.3 m/s at a 0.15 m box (lidar can't see it, the camera only from 1.25 m) | stopped 6 cm from it (`obstacle_memory`; without it: drove into it) |
| 0.3 m/s at a table's side (tabletop 0.73 m) | stopped 9 cm from the tabletop edge (camera) |
| spin 0.4 rad/s, side 18 cm from a wall | stopped 2 cm from it |
| same box, removed after the stop; back off 1 m; forward again | backed off, then drove on through the empty spot |
| Nav2 goal, controller frozen, its last command (0.3 m/s) kept coming | stopped (earlier zone setup: 24 cm) |

Nav2 through random obstacles (`random_obstacles:=5`, seeds 7/11/23;
spawn -> 8.2,2.4 -> 2.4,1.4 -> 5.5,3.4 -> home): no contact in any run,
closest 4 cm. Seed 11: all four goals. Seed 7: the turn-around at 8.2,2.4
next to a box is blocked (the turning footprint would hit it), the goal
aborts. Seed 23: goal 5.5,3.4 overlaps a box and aborts; the base then
stays wedged beside it. Nav2 here only turns in place and never
reverses: with no room to turn, it aborts rather than hits.

A frozen (not just stuck) Nav2 node trips the lifecycle manager's bond
(~4 s): it deactivates the whole stack, collision monitor included, and
nothing publishes `cmd_vel` any more. The sim base stops after 0.5 s
without commands; the real base's own command timeout is what stops it
then.

**Remaining gaps.** Turning room: a goal beside an obstacle may be
unreachable (see above); allowing reversing (RPP `allow_reversing`) or a
planner that knows the footprint (Smac) is the next step. Tabletops
beside the robot: the lidar can't see a tabletop edge within ~0.7 m and
the camera only looks forward, so a spin next to a table relies on the
costmap and memory having seen it earlier. And the camera's near blind
zone: The front D435i is 0.69
m up and level, 87 x 58 deg: it sees the floor from ~1 m ahead of the
lens, a 0.15 m-tall obstacle from 1.0 m, a 0.4 m one from 0.55 m.
What it marked on the way in stays marked; an obstacle that appears
closer than that (or beside the robot while it spins in place) isn't
seen. Tilting the camera down ~15 deg would bring the floor in to ~0.6 m.
