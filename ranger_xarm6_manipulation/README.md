# ranger_xarm6_manipulation

Mobile manipulation for the Ranger Mini 3.0 + xArm6: one action,
`MoveToGoal`, that moves the base and/or the arm. Every motion, base
included, is planned with MoveIt 2 / OMPL (`ranger_xarm6_moveit_config`)
against the planning scene.

| Node | What it does |
|---|---|
| `mobile_manipulation_coordinator.py` | `MoveToGoal` action server. Runs the stages, switches the arm to `arm_trajectory_controller`, fills MoveIt's planning scene (floor + the Gazebo world's static models). |
| `base_trajectory_server.py` | `FollowJointTrajectory` server for the base (`base_trajectory_controller/follow_joint_trajectory`). Tracks MoveIt's base trajectories in time on `cmd_vel`. |

## The base never translates and rotates at once

The Ranger either **crabs** (parallel mode: translates in any direction
at a fixed heading) or **spins** in place; its dual-Ackermann mode does
both but on an arc, and isn't used. So MoveIt's model has the base as
three joints, `base_x_joint`, `base_y_joint`, `base_theta_joint`
(`ranger_xarm6_moveit_config/urdf/ranger_xarm6_moveit.urdf.xacro`), and
no planning group mixes them:

| Group | Joints | Ranger mode |
|---|---|---|
| `base_translation` | x, y | crab |
| `base_rotation` | theta | spin |
| `whole_body` | x, y + arm | crab, arm moving at the same time |
| `arm` | joint1..6 | base still |

`base_trajectory_server.py` rejects any trajectory that does both, and
sends only commands the driver maps to one mode (crab: `angular.z = 0`,
`linear.y` never exactly 0; spin: `linear = 0`), stopping for 0.5s after
each so the wheels can re-steer.

## Modes

- **SEQUENTIAL**: arm to `transit_state` (default `stow`) -> base crabs to
  `base_goal` x/y -> base spins to `base_goal` theta -> arm to
  `arm_named_goal` or `arm_pose_goal`. Stages run only when needed.
- **WHOLE_BODY**: base spins to `base_goal` theta (if needed) -> base crab
  and arm in one plan (`whole_body` group), moving together. With an arm
  pose goal and `move_base: false`, the coordinator picks the base
  position: the nearest one, at the current heading, from which the arm
  has a collision-free IK solution (if that's where the base already is,
  only the arm moves).
- **DEFAULT** (`mode: 0`): whichever `control.launch.py` was started with.

## Run (sim)

Terminal 1, the robot (its own RViz off, MoveIt brings one):

    ros2 launch ranger_xarm6_description ranger_xarm6.launch.py run_rviz:=false \
      world:=artc_lab.world x:=0.94 y:=4.35 yaw:=-1.5708

Terminal 2, pick a controller (MoveIt + base executor + coordinator + RViz
MotionPlanning):

    ros2 launch ranger_xarm6_manipulation control.launch.py controller:=moveit_sequential
    ros2 launch ranger_xarm6_manipulation control.launch.py controller:=moveit_whole_body

`ros2 launch ranger_xarm6_manipulation control.launch.py --show-args` lists
the choices. `controller` only sets the default mode; a goal can still ask
for either. The world's tables/walls/cupboard reach MoveIt's planning
scene from the robot bringup (`world_collision_objects.py`), nothing to
pass here.
Stop `wbc.py` first if it's running: the coordinator takes the arm off
`arm_velocity_controller`, and the base executor publishes `cmd_vel`.

## Example goals

Sequential: drive to the middle of the lab (arm stowed on the way),
facing +y, then arm to `home`:

    ros2 action send_goal -f /robot_a/mobile_manipulation/move_to_goal \
      ranger_xarm6_manipulation/action/MoveToGoal \
      "{mode: 1, move_base: true, base_goal: {x: 3.2, y: 2.6, theta: 1.5708},
        move_arm: true, arm_named_goal: home}"

Whole body, base position chosen for you: gripper pointing down 5cm above
cube_4 on the north-east table (odom = Gazebo world frame in sim):

    ros2 action send_goal -f /robot_a/mobile_manipulation/move_to_goal \
      ranger_xarm6_manipulation/action/MoveToGoal \
      "{mode: 2, move_arm: true, arm_pose_goal: {header: {frame_id: robot_a_odom},
        pose: {position: {x: 7.4, y: 4.12, z: 0.85}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}}"

Whole body with a given base pose; a pose in `base_link` is relative to
where the base ends up (here: cube_3, 0.52m behind the base):

    ros2 action send_goal -f /robot_a/mobile_manipulation/move_to_goal \
      ranger_xarm6_manipulation/action/MoveToGoal \
      "{mode: 2, move_base: true, base_goal: {x: 6.5, y: 3.58, theta: -1.5708},
        move_arm: true, arm_pose_goal: {header: {frame_id: robot_a_base_link},
        pose: {position: {x: -0.52, y: 0.0, z: 0.535}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}}"

Named arm states (SRDF): `home`, `stow`. The xArm is mounted at the rear of
the base facing backwards (-x of base_link), so reachable poses are
behind the robot; with the gripper pointing down 5cm above the lab's
tables, it reaches only ~0.38m horizontally from `link_base`.

From RViz alone: pick a group in the MotionPlanning panel (`arm`,
`base_translation`, `base_rotation` or `whole_body`), set a goal, Plan &
Execute. `arm_trajectory_controller` must be active (the coordinator
switches it in at startup).

## Behaviour and limits

- **Pose goals** are solved with MoveIt's IK service seeded from the
  arm's current joints (plus perturbed seeds, since KDL stalls at the
  wrist singularity near the gripper-down poses), then planned as a joint
  goal: short, predictable motions instead of whichever IK branch OMPL's
  goal sampler picks. Goal states in collision fail up front, naming the
  colliding pair.
- **Speed**: `velocity_scaling` 0 means the coordinator's
  `default_velocity_scaling` (0.3). Base limits (`joint_limits.yaml`) are
  per axis: 0.3 m/s along x and y, 0.23 rad/s spinning at 0.3.
- **Base tracking** follows the plan's timing (so it stays in step with
  the arm) with position feedback from TF; it can't correct heading drift
  while crabbing or position drift while spinning. Tuning:
  `config/base_trajectory_server.yaml`.
- **Planning scene**: a floor (from the coordinator), plus in sim the
  Gazebo world's static models' box/cylinder/sphere collisions, published
  by `ranger_xarm6_description`'s `world_collision_objects.py` as part of
  the robot bringup (the plant's leaves use their collision cylinder; the
  cubes are left out, they move). On real hardware there's only the floor
  until perception is in.
- **Base pose** comes from TF (`odom -> base_link`), published to MoveIt
  as the base joints' states (x, y, z, theta) by
  `ranger_xarm6_moveit_config`'s `base_joint_state_publisher.py`, so
  MoveIt's model matches TF and odom-frame goals need no correction.
- **Real hardware is untested.** In particular: whether the Ranger
  re-steers smoothly when a crab path changes direction mid-motion, and
  how much heading it drifts while crabbing.
