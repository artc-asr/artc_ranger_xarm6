# ranger_xarm6_navigation

Odometry, mapping, localization and navigation for the Ranger Mini 3.0 +
xArm6. Built so far: the EKF (`odom -> base_link`) and FAST-LIO2 3D
mapping. To come: a 2D grid exported from the 3D map, 3D NDT scan-to-map
localization (`map -> odom`), and Nav2.

| Piece | What it does |
|---|---|
| `launch/odometry.launch.py` | robot_localization EKF over the Ranger's wheel odometry and the Mid-360's built-in IMU; publishes `odometry/filtered` and `odom -> base_link`. |
| `scripts/ekf_inputs.py` | Makes the raw driver messages fusable (see below). |
| `launch/mapping.launch.py` | FAST-LIO2 mapping session on the Mid-360 + its IMU; `map_save` writes the 3D map (`.pcd`). |
| `src/livox_cloud_to_custom.cpp` | The Livox driver's `PointCloud2` -> the Livox `CustomMsg` FAST-LIO reads. |

## Sensors

| Sensor | Topic | Used for |
|---|---|---|
| Livox Mid-360 | `livox/lidar` (`PointCloud2`, the driver's `xfer_format: 0` layout, per-point `timestamp`) | mapping, localization |
| Mid-360 built-in IMU (ICM-40609) | `livox/imu` (acceleration in g, frame `livox_frame`, as the driver publishes) | EKF (gyro), mapping |
| Ranger wheel odometry | `odom` (`ranger_base`) | EKF |
| Front D435i (`fixed_cam1`) | depth | planned: Nav2 near-field obstacles |

Not used: the rear D435i (unreliable up close), the arm's Gemini (moves),
the HiPNUC IMU. In sim, `ranger_xarm6_description` publishes the same
topics in the same formats (`sim_livox_imu.py`, `base_pose_publisher.py`,
`gz_lidar_to_pointcloud.py`), plus `ground_truth/odom`.

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
