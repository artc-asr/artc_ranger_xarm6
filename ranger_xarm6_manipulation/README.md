# ranger_xarm6_manipulation

Mobile manipulation for the Ranger Mini 3.0 + xArm6: one action,
`MoveToGoal`, that moves the base and/or the arm, planned with MoveIt 2
(`ranger_xarm6_moveit_config`).

| Node | What it does |
|---|---|
| `mobile_manipulation_coordinator.py` | `MoveToGoal` action server. Runs the stages, switches the arm to `arm_trajectory_controller`, adds a floor to MoveIt's planning scene. |
| `base_trajectory_server.py` | `FollowJointTrajectory` server for the base (`base_trajectory_controller/follow_joint_trajectory`). Drives through odom waypoints on `cmd_vel`. |

## Modes

- **SEQUENTIAL** (`mode: 0`): arm to `transit_state` (default `stow`), then
  base to `base_goal`, then arm to `arm_named_goal` or `arm_pose_goal`.
  Stages run only when needed (`move_base`, `move_arm`).
- **WHOLE_BODY** (`mode: 1`): not implemented yet; rejected.

## Run (sim)

Terminal 1, the robot (its own RViz off, MoveIt brings one):

    ros2 launch ranger_xarm6_description ranger_xarm6.launch.py run_rviz:=false

Terminal 2, MoveIt + base executor + coordinator + RViz (MotionPlanning):

    ros2 launch ranger_xarm6_manipulation manipulation.launch.py

Stop `wbc.py` first if it's running: the coordinator takes the arm off
`arm_velocity_controller`, and the base executor publishes `cmd_vel`.

## Example goals

Drive the base (arm stowed on the way), then arm to `home`:

    ros2 action send_goal -f /robot_a/mobile_manipulation/move_to_goal \
      ranger_xarm6_manipulation/action/MoveToGoal \
      "{mode: 0, move_base: true, base_goal: {x: 0.5, y: 0.2, theta: 0.6},
        move_arm: true, arm_named_goal: home}"

Arm only, to a gripper (`link_tcp`) pose relative to the base (gripper
pointing down, 0.42m behind base_link, 0.15m above the deck):

    ros2 action send_goal -f /robot_a/mobile_manipulation/move_to_goal \
      ranger_xarm6_manipulation/action/MoveToGoal \
      "{mode: 0, move_arm: true, arm_pose_goal: {header: {frame_id: robot_a_base_link},
        pose: {position: {x: -0.42, y: 0.0, z: 0.15}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}}"

Named arm states (SRDF): `home`, `stow`. The xArm is mounted at the rear of
the base facing backwards (-x of base_link), so reachable poses are
mostly behind the robot.

From RViz alone: pick group `arm` in the MotionPlanning panel, drag the
marker, Plan & Execute. `arm_trajectory_controller` must be active (the
coordinator switches it in at startup).

## Behaviour and limits

- **Pose goals** are solved with MoveIt's IK service seeded from the
  arm's current joints (plus perturbed seeds, since KDL stalls at the
  wrist singularity near the gripper-down poses), then planned as a joint
  goal: short, predictable motions instead of whichever IK branch OMPL's
  goal sampler picks.
- **Base motion** only uses command shapes the Ranger driver maps to a
  single steering mode: straight-line crab translation (heading fixed),
  then spin in place, with a 0.5s stop between for the wheels to re-steer.
  Tracks geometry, not timing. Tuning: `config/base_trajectory_server.yaml`.
- **The base isn't collision-checked** in sequential mode: nothing stops it
  driving into things. Only use clear paths until navigation or
  whole-body planning is in.
- **Planning scene** holds the robot and a floor only; no perception yet.
- **Base pose** comes from TF (`odom -> base_link`); MoveIt's planar base
  joint ignores its z, and so do pose goals (see the coordinator's
  docstring).
