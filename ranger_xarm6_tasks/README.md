# ranger_xarm6_tasks

Tasks for the Ranger Mini 3.0 + xArm6 as **behavior trees**: drag-and-drop
steps in **Groot2** ("drive to this waypoint, turn the arm side to the
table, pick, carry, place"), saved as XML, run by name. Every step is a
client of an action the robot already serves (Nav2, MoveIt's `MoveToGoal`,
the gripper), so no low-level code changes to make a new task. Waypoints
and arm poses are **taught** by putting the robot there and saving.

Built on [BehaviorTree.CPP v4](https://www.behaviortree.dev/) (apt) and
[BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2)
(submodule) for the task server; tested in sim (2026-09-28), untested on
hardware.

```
ranger_xarm6_tasks/
  trees/*.xml                 the tasks (behavior trees): edit in Groot2
  config/waypoints.yaml       base waypoints (taught)
  config/arm_poses.yaml       gripper poses (taught)
  ranger_xarm6_tasks.btproj   the Groot2 project (rewritten by the task server at startup)
  src/                        task server + the task nodes
  scripts/                    save_waypoint.py, save_arm_pose.py, run_task.py
  launch/tasks.launch.py
```

The task server reads `trees/` and `config/` **from this source folder**
at the start of every task: a tree saved in Groot2 or a waypoint just
taught is used by the next run, no rebuild or restart.

## 1. Setup (once)

```bash
sudo apt install ros-humble-behaviortree-cpp          # BT.CPP v4 (next to Nav2's v3)
cd ~/Worksplace/artc_ranger_xarm6
git submodule update --init BehaviorTree.ROS2
colcon build --packages-select btcpp_ros2_interfaces behaviortree_ros2 ranger_xarm6_manipulation ranger_xarm6_tasks
```

**Groot2** (the editor/monitor, no install needed): download
`Groot2-v1.9.0-x86_64.AppImage` from https://www.behaviortree.dev/groot/,
then

```bash
mkdir -p ~/Applications && mv ~/Downloads/Groot2-*-x86_64.AppImage ~/Applications/
chmod +x ~/Applications/Groot2-*-x86_64.AppImage
~/Applications/Groot2-*-x86_64.AppImage
```

(Needs `libfuse2`, already installed here.) The free version edits
without limits; **live monitoring shows trees of up to 20 nodes** (PRO,
€590/year, lifts that).

## 2. Start everything

Four terminals, each starting with `cd ~/Worksplace/artc_ranger_xarm6 &&
source install/setup.bash` (and the same `RMW_IMPLEMENTATION` in all, see
the repo README):

```bash
# 1. the robot (sim)
ros2 launch ranger_xarm6_description ranger_xarm6.launch.py run_rviz:=false \
  world:=artc_lab.world x:=0.94 y:=4.35 yaw:=-1.5708
# 2. Nav2 (wait for "Managed nodes are active" twice)
ros2 launch ranger_xarm6_navigation navigation.launch.py map:=~/ranger_xarm6_maps/artc_lab.yaml
# 3. MoveIt (wait for "Arm named states")
ros2 launch ranger_xarm6_manipulation control.launch.py
# 4. the task layer (task server + gripper action server)
ros2 launch ranger_xarm6_tasks tasks.launch.py
```

Terminal 4 prints `Groot2 project: .../ranger_xarm6_tasks.btproj` when
it's ready, and every step of a running task.

## 3. Run a task

```bash
ros2 run ranger_xarm6_tasks run_task.py --list          # Tasks: DemoPick DemoPickPlace DemoTurn GoHome GripperTest
ros2 run ranger_xarm6_tasks run_task.py DemoPickPlace
```

It prints `SUCCESS` or `FAILED` with the reason. **Ctrl-C cancels**: the
tree is halted and the robot's current action (Nav2 goal, MoveIt motion)
is cancelled; the robot stops. One task at a time: a second one is
refused while one runs. The same from anything that speaks ROS:

```bash
ros2 action send_goal /robot_a/execute_task btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: DemoTurn}"
```

The example tasks (`trees/Basics.xml`, `trees/PickPlace.xml`, for the sim's
`artc_lab`):

| Task | What it does |
|---|---|
| `GoHome` | arm to `stow`, Nav2 to `home` |
| `GripperTest` | close, open |
| `DemoTurn` | Nav2 to the north-east table's staging spot, turn 90 deg, turn the arm side (the back) towards `cube_4` |
| `DemoPick` | pick `cube_4` from the north-east table (subtree `PickAt`) |
| `DemoPickPlace` | pick `cube_4`, carry it to the south table, put it down (`PlaceAt`), go home |

Tested in sim: all of these run to SUCCESS; `DemoPickPlace` is 18 steps
(~3 min sim time, 4-5 min wall), ending 9 cm / 1.4 deg from home. Five
runs of it on fresh stacks: 4 SUCCESS (FastDDS 3/3, Cyclone 1/2). The
failure logged Nav2's intermittent controller TF freeze (see
`ranger_xarm6_navigation`'s README); one earlier run also saw MoveIt return a
plan through the tabletop once, which is why `PickAt`/`PlaceAt` retry
their arm moves. A failed task stops where it is: re-run it, or run
`GoHome`. **In sim the cube
doesn't come along**: the fingers close on it (the gripper reports
`holding something`, stopped at 0.41-0.47 of 0.85) but Gazebo's contact
grasp doesn't hold it when the arm lifts. The task logic is unaffected;
a sim grasp that holds (finger friction, or attaching the cube on grasp)
is separate work.

## 4. Edit tasks in Groot2

1. Start Groot2 (above). **File -> Open Project** (or the "Open Project"
   button) -> `~/Worksplace/artc_ranger_xarm6/ranger_xarm6_tasks/ranger_xarm6_tasks.btproj`.
2. The left panel lists the **trees** (the tasks, and the subtrees
   `PickAt`, `PlaceAt`) and the **models**: every node below, with its
   ports and their descriptions. Double-click a tree to open it.
3. **Edit**: drag nodes from the models list onto the canvas, connect
   them under a `Sequence` (steps in order: stops at the first failure),
   `Fallback` (tries children until one succeeds), `RetryUntilSuccessful`,
   `Parallel`, ... Click a node to fill in its ports (e.g. `waypoint` =
   `table_ne_stage`). A value in braces, `{stage}`, reads a blackboard
   entry: that's how subtrees take parameters.
4. **New task**: add a tree (the "+" next to the trees list), name it (the
   name is what `run_task.py` runs), build it, **save** (Ctrl-S). A new
   tree must be saved in a file under `trees/` (any `*.xml` there is
   loaded; one file can hold several trees).
5. Run it: `run_task.py <Name>`. No restart needed.

The project file is regenerated when the task server starts (the tree
files under `trees/` and the nodes' models), so the palette always
matches the code; restart terminal 4 after adding a *file* so the project
lists it (Groot2 can also add it: "Add existing file").

**Watch a task live**: while a task runs, Groot2's **Monitor** mode ->
connect to `localhost`, port `1667`. Nodes light up as they run
(running / success / failure). The live view is created per run: connect
after starting the task. Free Groot2: trees of up to 20 nodes (`DemoPick`
is 11; `DemoPickPlace` with its subtrees is over 20). Not checked here:
Groot2's GUI couldn't run in this session; the server's side (port 1667
open while a task runs) was.

## 5. The nodes

Angles in degrees. `pose`/`waypoint` names come from `config/`.

| Node | Uses | Ports |
|---|---|---|
| **NavigateToWaypoint** | Nav2 | `waypoint`: a taught waypoint |
| **NavigateToPose** | Nav2 | `x`, `y`, `yaw_deg`, `frame` (default map) |
| **BaseToPose** | MoveIt (crab/spin, precise; for docking) | `waypoint`, or `x`, `y`, `yaw_deg`, `frame`; `mode`; `collision_monitor` (auto) |
| **TurnBase** | MoveIt spin in place | one of: `back_towards` (a waypoint or arm pose name: the arm side faces it), `yaw_deg` (absolute, map frame), `angle_deg` (relative, + = left); `mode`; `collision_monitor` (auto) |
| **ArmToNamedPose** | MoveIt, arm only | `pose`: an SRDF named pose (`home`, `stow`, yours) |
| **ArmToPose** | MoveIt, arm only | `pose` (taught), or `x`, `y`, `z`, `frame` (default odom; `robot_a_base_link` = relative to the base), `orientation` (`down`, `down:<deg>`, or `qx qy qz qw`); `offset_z` (e.g. `-0.07` to go down) |
| **WholeBodyToPose** | MoveIt, base + arm together | the `ArmToPose` ports + `base_waypoint` (where the base goes; empty = chosen at the current heading); `collision_monitor` (never) |
| **Gripper** | `gripper_command` | `command`: `open`, `close`, or rad (0..0.85); `expect`: `any`, `holding` (fail if it closed on nothing), `empty` |
| **Say** | log | `message` |

All MoveIt nodes also take `velocity_scaling` (0..1, 0 = default 0.3);
`mode` is `sequential` (arm stows before the base moves) or `whole_body`
(it stays). Before each MoveIt goal the node waits until the base is still
(Nav2 reports arrival while the base is still settling, and MoveIt aborts
a plan whose start moved).

**`collision_monitor`**: while Nav2 runs, its collision monitor also
guards MoveIt's base motions (slows/stops them when the lidar or the
front camera sees something in the way). Docking at a table has to bypass
it: the base goes partly under the tabletop, which the monitor's 2D
footprint calls a collision; so `WholeBodyToPose` defaults to `never`,
and a `BaseToPose` that leaves a dock needs `collision_monitor="never"`
(as in `PickAt`/`PlaceAt`). With `never`, only MoveIt's 3D check of the
room (the world's static models) applies: nothing the sensors see.

## 6. Teach waypoints and arm poses

**Waypoint** (where the base goes and which way it faces): drive the robot
there (teleop, `NavigateToPose`, RViz 2D Goal Pose), then

```bash
ros2 run ranger_xarm6_tasks save_waypoint.py kitchen_counter
ros2 run ranger_xarm6_tasks save_waypoint.py kitchen_counter --yaw-deg -90   # store another heading
```

The arm is at the back: for a pick/place spot, save the heading with the
**back to the table**. Use two waypoints per table, as the examples do: a
`*_stage` ~0.6 m out (Nav2 drives there: its map treats the table's
outline as an obstacle), and a `*_dock` close in (MoveIt docks there,
base partly under the tabletop).

**Arm pose** (where the gripper goes): move the arm there (a MoveIt pose
goal, or RViz's MotionPlanning drag + Execute; see the repo README), then

```bash
ros2 run ranger_xarm6_tasks save_arm_pose.py above_bin                           # a place in the room (odom)
ros2 run ranger_xarm6_tasks save_arm_pose.py carry --frame robot_a_base_link     # relative to the base
```

Both write one line to `config/*.yaml` (replacing a line of that name,
keeping the rest and comments); you can also edit those files by hand.
Named arm *joint* poses (`home`, `stow`) are MoveIt's: see the repo
README, "Saving a new named pose".

## 7. How it works

- `task_server` (`src/task_server.cpp`, BehaviorTree.ROS2's
  `TreeExecutionServer`): action `/robot_a/execute_task`, service
  `/robot_a/get_loaded_trees`, Groot2 on port 1667. Re-reads `trees/` and
  `config/` per task; refuses a task while one runs.
- The nodes (`src/nodes.cpp`) are clients of `/robot_a/navigate_to_pose`,
  `/robot_a/mobile_manipulation/move_to_goal`, `/robot_a/gripper_command`
  (`src/action_node.hpp`: one client per action for the whole server,
  served by its executor; a goal the tree stops following is cancelled).
  BehaviorTree.ROS2's own `RosActionNode` isn't used: on Humble its
  per-client executors stopped delivering Nav2's goal replies after a
  MoveIt goal (Nav2 drove while the task had failed).
- `gripper_action_server.py` (`ranger_xarm6_manipulation`):
  `control_msgs/GripperCommand` on the gripper controller; `position` is
  drive_joint in rad (0 open .. 0.85 closed), `stalled` = stopped short
  while closing (holding something).
- `base_trajectory_server.py` (MoveIt's base executor) sends through the
  collision monitor while Nav2 runs (parameter `collision_monitor`,
  auto/never/always), waiting until Nav2's velocity smoother has gone
  quiet.

A new kind of step (a new node) means C++ in `src/nodes.cpp` + registering
it in `registerTaskNodes`; everything else (new tasks, waypoints, poses)
is data.
