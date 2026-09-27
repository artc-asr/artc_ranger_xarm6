#!/usr/bin/env python3
"""Mobile manipulation coordinator: one action for base + arm goals.

Action: <ns>/mobile_manipulation/move_to_goal
(ranger_xarm6_manipulation/action/MoveToGoal).

Every motion is planned by MoveIt (OMPL) against the planning scene, base
included, using the base's x/y/theta joints in MoveIt's model
(ranger_xarm6_moveit_config/urdf/ranger_xarm6_moveit.urdf.xacro). No plan
ever translates and rotates the base at once: the Ranger only crabs at a
fixed heading or spins in place (see base_trajectory_server.py).

SEQUENTIAL mode, each stage only if needed:
  1. transit:   arm to an SRDF named state (default "stow"), group 'arm'.
                Only when the base moves.
  2. translate: base crabs to base_goal x/y, group 'base_translation'.
  3. rotate:    base spins to base_goal theta, group 'base_rotation'.
  4. arm:       arm to arm_named_goal or arm_pose_goal, group 'arm'.
WHOLE_BODY mode:
  1. rotate:     base spins to base_goal theta first (arm held as is), so
                 the rest can run at a fixed heading.
  2. whole_body: base crab + arm in ONE plan, group 'whole_body'
                 (x, y, joint1..6), both moving at once. The base's x/y
                 target is base_goal's; without move_base and with a pose
                 goal it is chosen here (placement): the nearest base
                 position, at the current heading, from which the arm has
                 a collision-free IK solution. If that's where the base
                 already is, only the arm moves.

Arm pose goals are solved with MoveIt's IK service seeded from the arm's
current joints (plus perturbed seeds: KDL stalls at the wrist singularity
next to the gripper-down poses), then planned as a joint goal: the
nearest configuration, not whichever of the many joint1/4/6 (+-2pi)
solutions OMPL's goal sampler picks. The pose is handed to IK in odom
(MoveIt's model matches TF, base height included): a goal in base_link
means relative to where the base ends up; any other frame is resolved
through TF now.

At startup it adds a floor to MoveIt's planning scene: base_height below
base_link, minus a 1cm gap so the wheels don't touch it. In sim, the
Gazebo world's static models come from ranger_xarm6_description's
world_collision_objects.py (started by the robot bringup), not from here.

MoveIt plans are executed on arm_trajectory_controller, which this node
switches in (deactivating arm_velocity_controller, wbc.py's controller) at
startup and before every stage: the two can't both drive the arm.
"""
import math
import time
import xml.etree.ElementTree as ET

import numpy as np

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, MoveItErrorCodes, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK, GetStateValidity
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener

from ranger_xarm6_manipulation.action import MoveToGoal

MOVEIT_ERRORS = {v: k for k, v in vars(MoveItErrorCodes).items() if k.isupper() and isinstance(v, int)}
MODES = {'sequential': MoveToGoal.Goal.SEQUENTIAL, 'whole_body': MoveToGoal.Goal.WHOLE_BODY}


class StageFailed(Exception):
    pass


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Coordinator(Node):
    def __init__(self):
        super().__init__('mobile_manipulation_coordinator')
        p = self.declare_parameter
        prefix = p('prefix', '').value
        default_mode = p('default_mode', 'sequential').value
        if default_mode not in MODES:
            raise ValueError(f"default_mode must be one of {sorted(MODES)}, got {default_mode!r}")
        self.default_mode = MODES[default_mode]
        self.arm_group = p('arm_group', 'arm').value
        self.translation_group = p('translation_group', 'base_translation').value
        self.rotation_group = p('rotation_group', 'base_rotation').value
        self.whole_body_group = p('whole_body_group', 'whole_body').value
        self.tcp_link = p('tcp_link', f'{prefix}link_tcp').value
        self.odom_frame = p('odom_frame', f'{prefix}odom').value
        self.base_frame = p('base_frame', f'{prefix}base_link').value
        self.arm_base_frame = p('arm_base_frame', f'{prefix}link_base').value
        self.base_x, self.base_y, self.base_theta = (
            f'{prefix}base_x_joint', f'{prefix}base_y_joint', f'{prefix}base_theta_joint')
        self.arm_joints = [f'{prefix}joint{i}' for i in range(1, 7)]
        self.default_transit = p('default_transit_state', 'stow').value
        self.planning_time = p('planning_time', 5.0).value
        self.planning_attempts = p('planning_attempts', 5).value
        # Used when a goal's velocity_scaling is 0. move_group itself
        # treats 0 as 1.0 (full joint_limits.yaml speed); the
        # default_*_scaling_factor there is only read by MoveGroupInterface.
        self.default_scaling = p('default_velocity_scaling', 0.3).value
        self.state_tolerance = p('named_state_tolerance', 0.01).value  # rad
        # Before each plan, wait (up to settle_timeout, ROS time) for the
        # arm to stop: MoveIt rejects a trajectory whose start is more than
        # 0.02 rad from the current state, and the arm may still be
        # finishing the last motion (its controller reports success within
        # a 0.02 rad goal tolerance).
        self.settle_velocity = p('settle_velocity', 0.005).value  # rad/s
        self.settle_timeout = p('settle_timeout', 3.0).value      # s
        self.base_xy_tolerance = p('base_xy_tolerance', 0.02).value    # m: closer = don't translate
        self.base_yaw_tolerance = p('base_yaw_tolerance', 0.02).value  # rad: closer = don't rotate
        self.ik_timeout = p('ik_timeout', 0.2).value  # s, per seed
        self.ik_perturbed_seeds = p('ik_perturbed_seeds', 10).value
        self.ik_seed_noise = p('ik_seed_noise', 0.2).value  # rad, std dev
        # Base placement search (WHOLE_BODY pose goals without a base goal):
        # grid of base positions around the target, nearest to the base
        # first, kept if the arm's base (link_base) is within reach.
        self.placement_step = p('placement_step', 0.1).value            # m
        # Horizontal link_base -> target distance: an upper bound only (the
        # arm's reach shrinks with height, ~0.38m at 5cm above the lab's
        # tables), IK decides. Failing IK calls return in a few ms, so the
        # whole ring (~130 candidates at these defaults) is affordable.
        self.placement_reach = p('placement_reach', 0.65).value         # m
        self.placement_min_reach = p('placement_min_reach', 0.15).value  # m
        self.placement_candidates = p('placement_candidates', 200).value
        self.placement_ik_timeout = p('placement_ik_timeout', 0.05).value  # s, per seed
        self.trajectory_controller = p('trajectory_controller', 'arm_trajectory_controller').value
        self.velocity_controller = p('velocity_controller', 'arm_velocity_controller').value
        activate_on_start = p('activate_trajectory_controller_on_start', True).value
        # base_link height above the floor (ranger_mini_v3's wheels bottom
        # out 0.315m below it; ranger_xarm6.launch.py's z default). <= 0
        # disables the floor.
        self.base_height = p('base_height', 0.315).value

        group = ReentrantCallbackGroup()
        self.move_group = ActionClient(self, MoveGroup, 'move_action', callback_group=group)
        self.switch_srv = self.create_client(SwitchController, 'controller_manager/switch_controller', callback_group=group)
        self.list_srv = self.create_client(ListControllers, 'controller_manager/list_controllers', callback_group=group)
        self.params_srv = self.create_client(GetParameters, 'move_group/get_parameters', callback_group=group)
        self.ik_srv = self.create_client(GetPositionIK, 'compute_ik', callback_group=group)
        self.scene_srv = self.create_client(ApplyPlanningScene, 'apply_planning_scene', callback_group=group)
        self.validity_srv = self.create_client(GetStateValidity, 'check_state_validity', callback_group=group)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.joint_positions = {}
        self.joint_velocities = {}
        self.create_subscription(JointState, 'joint_states', self._joint_states, 10, callback_group=group)

        self.named_states = {}
        self._child = None  # in-flight child goal handle, for cancellation
        self._busy = False
        self._server = ActionServer(
            self, MoveToGoal, 'mobile_manipulation/move_to_goal',
            execute_callback=self._execute, goal_callback=self._on_goal,
            cancel_callback=self._on_cancel, callback_group=group)
        self._startup = self.create_timer(1.0, self._on_startup, callback_group=group)
        self._activate_on_start = activate_on_start
        self.get_logger().info(f'Waiting for move_group... (default mode: {default_mode})')

    # ---- startup ------------------------------------------------------------
    async def _on_startup(self):
        if not self.params_srv.service_is_ready():
            return
        self._startup.cancel()
        res = await self.params_srv.call_async(GetParameters.Request(names=['robot_description_semantic']))
        srdf = ET.fromstring(res.values[0].string_value)
        self.named_states = {
            s.get('name'): {j.get('name'): float(j.get('value')) for j in s.findall('joint')}
            for s in srdf.findall('group_state') if s.get('group') == self.arm_group
        }
        self.get_logger().info(f'Arm named states: {sorted(self.named_states)}')
        await self._add_floor()
        if self._activate_on_start:
            try:
                await self._ensure_trajectory_controller()
            except StageFailed as e:
                self.get_logger().error(str(e))
        self.get_logger().info('Ready: mobile_manipulation/move_to_goal')

    async def _add_floor(self):
        if self.base_height <= 0:
            return
        try:
            base_z = self._base_transform().transform.translation.z
        except StageFailed as e:
            self.get_logger().error(f'No floor in the planning scene: {e}')
            return
        # 1cm under the wheels, so the robot's own start state isn't in collision.
        thickness = 0.02
        floor = CollisionObject(id='floor', operation=CollisionObject.ADD)
        floor.header.frame_id = self.odom_frame
        floor.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[200.0, 200.0, thickness])]
        pose = Pose()
        pose.position.z = base_z - self.base_height - 0.01 - thickness / 2
        pose.orientation.w = 1.0
        floor.primitive_poses = [pose]
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [floor]
        if not self.scene_srv.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('apply_planning_scene not available: no floor in the planning scene')
            return
        res = await self.scene_srv.call_async(ApplyPlanningScene.Request(scene=scene))
        if res.success:
            self.get_logger().info(f'Added a floor at z={pose.position.z + thickness / 2:.3f} to the planning scene')
        else:
            self.get_logger().error('Failed to add the floor to the planning scene')

    def _joint_states(self, msg):
        self.joint_positions.update(zip(msg.name, msg.position))
        self.joint_velocities.update(zip(msg.name, msg.velocity))

    def _wait_arm_settled(self):
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds * 1e-9 < self.settle_timeout:
            if all(abs(self.joint_velocities.get(j, 0.0)) < self.settle_velocity for j in self.arm_joints):
                return
            time.sleep(0.05)
        self.get_logger().warn(f'Arm still moving after {self.settle_timeout}s; planning anyway')

    # ---- action callbacks ---------------------------------------------------
    def _on_goal(self, goal):
        if self._busy:
            self.get_logger().warn('Rejecting goal: already executing one')
            return GoalResponse.REJECT
        if goal.mode not in (MoveToGoal.Goal.DEFAULT, *MODES.values()):
            self.get_logger().warn(f'Rejecting goal: unknown mode {goal.mode}')
            return GoalResponse.REJECT
        if not self.named_states:
            self.get_logger().warn('Rejecting goal: not connected to move_group yet')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _):
        if self._child is not None:
            self._child.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle):
        self._busy = True
        goal = goal_handle.request
        result = MoveToGoal.Result()
        self._stage = ''
        try:
            mode = goal.mode or self.default_mode
            await self._ensure_trajectory_controller()
            if mode == MoveToGoal.Goal.WHOLE_BODY:
                await self._whole_body(goal_handle, goal)
            else:
                await self._sequential(goal_handle, goal)
            result.success = True
            result.message = 'done'
            goal_handle.succeed()
        except StageFailed as e:
            result.success = False
            result.message = f'{self._stage}: {e}'
            self.get_logger().error(result.message)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        finally:
            self._child = None
            self._busy = False
        return result

    # ---- modes ------------------------------------------------------------------
    async def _sequential(self, goal_handle, goal):
        scaling = goal.velocity_scaling
        if goal.move_base:
            b = goal.base_goal
            x, y, yaw = self._base_pose()
            translate = math.hypot(b.x - x, b.y - y) > self.base_xy_tolerance
            rotate = abs(wrap(b.theta - yaw)) > self.base_yaw_tolerance

            if translate or rotate:
                transit = goal.transit_state or self.default_transit
                self._begin(goal_handle, 'transit', f'arm to {transit!r}')
                if self._at_named_state(transit):
                    self._feedback(goal_handle, f'arm already at {transit!r}')
                else:
                    await self._plan_execute(self.arm_group, self._named_goal(transit), scaling)
                self._check_cancel(goal_handle)

            if translate:
                self._begin(goal_handle, 'translate', f'base crab to x={b.x:.3f} y={b.y:.3f}')
                await self._plan_execute(self.translation_group, {self.base_x: b.x, self.base_y: b.y}, scaling)
                self._check_cancel(goal_handle)
            if rotate:
                self._begin(goal_handle, 'rotate', f'base spin to theta={b.theta:.3f}')
                await self._plan_execute(self.rotation_group, {self.base_theta: self._near_yaw(b.theta)}, scaling)
                self._check_cancel(goal_handle)

        if goal.move_arm:
            if goal.arm_named_goal:
                self._begin(goal_handle, 'arm', f'arm to {goal.arm_named_goal!r}')
                target = self._named_goal(goal.arm_named_goal)
            else:
                self._begin(goal_handle, 'arm', self._describe_pose(goal.arm_pose_goal))
                pose = self._pose_in_odom(goal.arm_pose_goal, self._base_pose())
                target = await self._ik(pose, self._base_pose())
            await self._plan_execute(self.arm_group, target, scaling)

    async def _whole_body(self, goal_handle, goal):
        scaling = goal.velocity_scaling
        x, y, yaw = self._base_pose()
        heading = goal.base_goal.theta if goal.move_base else yaw

        if abs(wrap(heading - yaw)) > self.base_yaw_tolerance:
            self._begin(goal_handle, 'rotate', f'base spin to theta={heading:.3f}')
            await self._plan_execute(self.rotation_group, {self.base_theta: self._near_yaw(heading)}, scaling)
            self._check_cancel(goal_handle)
            x, y, yaw = self._base_pose()

        arm_target = {j: self.joint_positions[j] for j in self.arm_joints if j in self.joint_positions}
        base_xy = (goal.base_goal.x, goal.base_goal.y) if goal.move_base else (x, y)
        if goal.move_arm and goal.arm_named_goal:
            arm_target = self._named_goal(goal.arm_named_goal)
        elif goal.move_arm:
            final_base = (*base_xy, yaw)
            if goal.move_base:
                self._begin(goal_handle, 'placement', f'IK with the base at x={base_xy[0]:.3f} y={base_xy[1]:.3f}')
                pose = self._pose_in_odom(goal.arm_pose_goal, final_base)
                arm_target = await self._ik(pose, final_base)
            else:
                self._begin(goal_handle, 'placement', f'base position for {self._describe_pose(goal.arm_pose_goal)}')
                pose = self._pose_in_odom(goal.arm_pose_goal, (x, y, yaw))
                base_xy, arm_target = await self._place_base(pose, (x, y, yaw))
            self._check_cancel(goal_handle)

        moves_base = math.hypot(base_xy[0] - x, base_xy[1] - y) > self.base_xy_tolerance
        if moves_base:
            self._begin(goal_handle, 'whole_body',
                        f'base crab to x={base_xy[0]:.3f} y={base_xy[1]:.3f} + arm, together')
            target = {self.base_x: base_xy[0], self.base_y: base_xy[1], **arm_target}
            await self._plan_execute(self.whole_body_group, target, scaling)
        elif goal.move_arm:
            self._begin(goal_handle, 'arm', 'base already in place, arm only')
            await self._plan_execute(self.arm_group, arm_target, scaling)

    # ---- stages ---------------------------------------------------------------
    def _begin(self, goal_handle, stage, status):
        self._stage = stage
        self._feedback(goal_handle, status)

    def _check_cancel(self, goal_handle):
        if goal_handle.is_cancel_requested:
            raise StageFailed('canceled')

    def _feedback(self, goal_handle, status):
        self.get_logger().info(f'[{self._stage}] {status}')
        goal_handle.publish_feedback(MoveToGoal.Feedback(stage=self._stage, status=status))

    async def _plan_execute(self, group, joint_goal, velocity_scaling):
        """Plan group to a joint-space goal with MoveIt and execute it."""
        await self._ensure_trajectory_controller()
        await self._check_goal_valid(group, joint_goal)
        self._wait_arm_settled()
        if not self.move_group.wait_for_server(timeout_sec=5.0):
            raise StageFailed('move_group action server not available')
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group
        req.num_planning_attempts = self.planning_attempts
        req.allowed_planning_time = self.planning_time
        scaling = velocity_scaling if velocity_scaling > 0 else self.default_scaling
        req.max_velocity_scaling_factor = scaling
        req.max_acceleration_scaling_factor = scaling
        c = Constraints()
        for joint, value in joint_goal.items():
            c.joint_constraints.append(JointConstraint(
                joint_name=joint, position=float(value), tolerance_above=1e-3, tolerance_below=1e-3, weight=1.0))
        req.goal_constraints = [c]
        goal.planning_options.plan_only = False
        result = await self._run_child(self.move_group, goal)
        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            raise StageFailed(f'MoveIt {MOVEIT_ERRORS.get(code, code)} ({group})')

    async def _check_goal_valid(self, group, joint_goal):
        """Fail with the colliding pairs if the goal state is in collision.

        OMPL would only say it couldn't sample the goal ("FAILURE").
        """
        if not self.validity_srv.wait_for_service(timeout_sec=5.0):
            return  # let the planner report it
        req = GetStateValidity.Request(group_name=group)
        req.robot_state.is_diff = True
        req.robot_state.joint_state = JointState(name=list(joint_goal), position=[float(v) for v in joint_goal.values()])
        res = await self.validity_srv.call_async(req)
        if res.valid:
            return
        pairs = sorted({' <-> '.join(sorted((c.contact_body_1, c.contact_body_2))) for c in res.contacts})
        detail = ', '.join(pairs[:4]) + (f' (+{len(pairs) - 4} more)' if len(pairs) > 4 else '')
        raise StageFailed(f'goal state is invalid ({group}): {detail or "violates constraints or bounds"}')

    async def _run_child(self, client, goal):
        handle = await client.send_goal_async(goal)
        if not handle.accepted:
            raise StageFailed('goal rejected')
        self._child = handle
        response = await handle.get_result_async()
        self._child = None
        return response.result

    # ---- goals -----------------------------------------------------------------
    def _named_goal(self, name):
        if name not in self.named_states:
            raise StageFailed(f'unknown named state {name!r} (have {sorted(self.named_states)})')
        return dict(self.named_states[name])

    def _near_yaw(self, theta):
        """theta, unwrapped to within pi of the base's current yaw joint value."""
        current = self.joint_positions.get(self.base_theta, 0.0)
        return current + wrap(theta - current)

    async def _ik(self, pose, base, seeds=None, timeout=None):
        """Arm joints reaching pose (odom, model frame) with the base at base (x, y, yaw).

        Seeds, in order: the current joints, small random perturbations of
        them (KDL stalls at the wrist singularity, joint5 ~ 0, which the
        gripper-pointing-down poses sit right next to), then the SRDF
        named states. The first collision-free solution wins, so the
        result stays as close to the current configuration as possible.
        """
        if not self.ik_srv.wait_for_service(timeout_sec=5.0):
            raise StageFailed('compute_ik service not available')
        if seeds is None:
            seeds = self._ik_seeds(self.ik_perturbed_seeds)
        timeout = self.ik_timeout if timeout is None else timeout
        code = None
        for seed in seeds:
            req = GetPositionIK.Request()
            ik = req.ik_request
            ik.group_name = self.arm_group
            ik.ik_link_name = self.tcp_link
            ik.pose_stamped = pose
            ik.avoid_collisions = True
            ik.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
            # is_diff: only the arm and base joints are set; everything else
            # stays at move_group's current state. The base joints put the
            # base where it will be, so IK's collision check sees it there.
            ik.robot_state.is_diff = True
            ik.robot_state.joint_state = JointState(
                name=[*self.arm_joints, self.base_x, self.base_y, self.base_theta],
                position=[float(v) for v in seed] + [float(base[0]), float(base[1]), float(self._near_yaw(base[2]))])
            res = await self.ik_srv.call_async(req)
            code = res.error_code.val
            if code == MoveItErrorCodes.SUCCESS:
                solution = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return {j: solution[j] for j in self.arm_joints}
        raise StageFailed(f'no collision-free IK solution for the pose from {len(seeds)} seeds '
                          f'({MOVEIT_ERRORS.get(code, code)})')

    def _ik_seeds(self, perturbed):
        current = [self.joint_positions.get(j, 0.0) for j in self.arm_joints]
        rng = np.random.default_rng(0)
        seeds = [current]
        seeds += [list(np.asarray(current) + rng.normal(0.0, self.ik_seed_noise, len(current)))
                  for _ in range(perturbed)]
        seeds += [[state.get(j, 0.0) for j in self.arm_joints] for state in self.named_states.values()]
        return seeds

    async def _place_base(self, pose, base):
        """Nearest base (x, y), at base's heading, from which the arm reaches pose.

        Candidates on a placement_step grid, kept when link_base lands
        within [placement_min_reach, placement_reach] of the target
        horizontally, tried nearest to the current base first (the
        current position itself first of all), at most
        placement_candidates of them.
        """
        x0, y0, yaw = base
        try:
            mount = self.tf_buffer.lookup_transform(self.base_frame, self.arm_base_frame, rclpy.time.Time())
        except TransformException as e:
            raise StageFailed(f'no {self.base_frame} -> {self.arm_base_frame} transform: {e}')
        mx, my = mount.transform.translation.x, mount.transform.translation.y
        tx, ty = pose.pose.position.x, pose.pose.position.y
        c, s = math.cos(yaw), math.sin(yaw)

        def reach(bx, by):
            return math.hypot(bx + c * mx - s * my - tx, by + s * mx + c * my - ty)

        span = self.placement_reach + math.hypot(mx, my)
        n = int(math.ceil(span / self.placement_step))
        # Grid around the target, snapped to the current base position so
        # "don't move" is one of the candidates.
        gx0 = x0 + round((tx - x0) / self.placement_step) * self.placement_step
        gy0 = y0 + round((ty - y0) / self.placement_step) * self.placement_step
        grid = [(gx0 + i * self.placement_step, gy0 + j * self.placement_step)
                for i in range(-n, n + 1) for j in range(-n, n + 1)]
        grid.append((x0, y0))
        candidates = sorted({(round(bx, 6), round(by, 6)) for bx, by in grid
                             if self.placement_min_reach <= reach(bx, by) <= self.placement_reach},
                            key=lambda b: math.hypot(b[0] - x0, b[1] - y0))
        candidates = candidates[:self.placement_candidates]
        if not candidates:
            raise StageFailed('target out of reach from any base position')
        seeds = self._ik_seeds(2)
        for bx, by in candidates:
            try:
                arm = await self._ik(pose, (bx, by, yaw), seeds=seeds, timeout=self.placement_ik_timeout)
            except StageFailed:
                continue
            self.get_logger().info(f'[placement] base at x={bx:.3f} y={by:.3f} '
                                   f'({math.hypot(bx - x0, by - y0):.3f} m away)')
            return (bx, by), arm
        raise StageFailed(f'no base position among the {len(candidates)} nearest reaches the pose '
                          'collision-free')

    def _describe_pose(self, pose):
        p = pose.pose.position
        return f'arm to pose ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) in {pose.header.frame_id!r}'

    def _base_transform(self):
        try:
            return self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())
        except TransformException as e:
            raise StageFailed(f'no {self.odom_frame} -> {self.base_frame} transform: {e}')

    def _base_pose(self):
        """Current (x, y, yaw) of base_link in odom."""
        t = self._base_transform().transform
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.translation.x, t.translation.y, yaw

    def _pose_in_odom(self, pose, base):
        """Express a PoseStamped in odom.

        base = (x, y, yaw) where the base will be: a pose in base_link is
        taken relative to it (at base_link's current height). Other frames
        are resolved through TF as they are now.
        """
        if pose.header.frame_id in (self.base_frame, ''):
            bx, by, byaw = base
            frame = _matrix(bx, by, self._base_transform().transform.translation.z,
                            (0.0, 0.0, math.sin(byaw / 2), math.cos(byaw / 2)))
        else:
            try:
                t = self.tf_buffer.lookup_transform(self.odom_frame, pose.header.frame_id, rclpy.time.Time())
            except TransformException as e:
                raise StageFailed(f'cannot resolve the pose goal frame: {e}')
            tt, r = t.transform.translation, t.transform.rotation
            frame = _matrix(tt.x, tt.y, tt.z, (r.x, r.y, r.z, r.w))
        p, o = pose.pose.position, pose.pose.orientation
        m = frame @ _matrix(p.x, p.y, p.z, (o.x, o.y, o.z, o.w))
        out = PoseStamped()
        out.header.frame_id = self.odom_frame
        out.pose.position.x, out.pose.position.y, out.pose.position.z = m[:3, 3]
        (out.pose.orientation.x, out.pose.orientation.y,
         out.pose.orientation.z, out.pose.orientation.w) = _quaternion(m[:3, :3])
        return out

    def _at_named_state(self, name):
        state = self.named_states.get(name)
        if not state or any(j not in self.joint_positions for j in state):
            return False
        return all(abs(math.remainder(self.joint_positions[j] - v, 2 * math.pi)) <= self.state_tolerance
                   for j, v in state.items())

    # ---- controllers -------------------------------------------------------------
    async def _ensure_trajectory_controller(self):
        if not self.list_srv.wait_for_service(timeout_sec=5.0):
            raise StageFailed('controller_manager not available')
        listing = await self.list_srv.call_async(ListControllers.Request())
        states = {c.name: c.state for c in listing.controller}
        if states.get(self.trajectory_controller) == 'active':
            return
        if self.trajectory_controller not in states:
            raise StageFailed(f'{self.trajectory_controller} is not loaded')
        req = SwitchController.Request()
        req.activate_controllers = [self.trajectory_controller]
        req.deactivate_controllers = [self.velocity_controller] if states.get(self.velocity_controller) == 'active' else []
        req.strictness = SwitchController.Request.STRICT
        res = await self.switch_srv.call_async(req)
        if not res.ok:
            raise StageFailed(f'could not activate {self.trajectory_controller}')
        self.get_logger().info(f'Switched arm to {self.trajectory_controller}'
                               + (f' (deactivated {self.velocity_controller})' if req.deactivate_controllers else ''))


def _matrix(x, y, z, q):
    """4x4 transform from a translation and an (x, y, z, w) quaternion."""
    qx, qy, qz, qw = q
    m = np.eye(4)
    m[:3, :3] = [
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ]
    m[:3, 3] = (x, y, z)
    return m


def _quaternion(r):
    """(x, y, z, w) quaternion from a rotation matrix."""
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return x, y, z, w


def main():
    rclpy.init()
    node = Coordinator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
