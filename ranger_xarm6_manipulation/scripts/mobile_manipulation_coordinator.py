#!/usr/bin/env python3
"""Mobile manipulation coordinator: one action for base + arm goals.

Action: <ns>/mobile_manipulation/move_to_goal
(ranger_xarm6_manipulation/action/MoveToGoal).

SEQUENTIAL mode, each stage only if needed:
  1. transit: arm to an SRDF named state (default "stow") via MoveIt,
     skipped if the arm is already there. Only runs when the base moves.
  2. base:    base to base_goal (odom frame) via base_trajectory_server.py.
  3. arm:     arm to arm_named_goal or arm_pose_goal (link_tcp) via MoveIt.
     Pose goals are first solved with MoveIt's IK service seeded from the
     arm's current joints, then planned as a joint goal: the nearest
     configuration, not whichever of the many joint1/4/6 (+-2pi)
     solutions OMPL's goal sampler happens to pick, which can turn a
     10cm move into a sweep across the robot.
     The pose is handed to IK in the odom frame as MoveIt's model sees it:
     the SRDF's planar base joint carries only x/y/yaw, so the goal is
     taken relative to base_link through TF, then placed using the base's
     planar pose only. Letting MoveIt transform it through TF instead
     would include odom->base_link's z (0.15m in sim), which the planar
     joint can't represent, shifting every goal by that much.
WHOLE_BODY mode is rejected for now.

At startup it also adds a floor to MoveIt's planning scene (a box whose
top is floor_height below base_link, minus a 1cm gap so the wheels don't
touch it), so the arm can't plan into the ground.

MoveIt plans are executed on arm_trajectory_controller, which this node
switches in (deactivating arm_velocity_controller, wbc.py's controller) at
startup and before every arm stage: the two can't both drive the arm.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from moveit_msgs.action import MoveGroup
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, MoveItErrorCodes, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from ranger_xarm6_manipulation.action import MoveToGoal

MOVEIT_ERRORS = {v: k for k, v in vars(MoveItErrorCodes).items() if k.isupper() and isinstance(v, int)}


class StageFailed(Exception):
    pass


class Coordinator(Node):
    def __init__(self):
        super().__init__('mobile_manipulation_coordinator')
        p = self.declare_parameter
        prefix = p('prefix', '').value
        self.arm_group = p('arm_group', 'arm').value
        self.tcp_link = p('tcp_link', f'{prefix}link_tcp').value
        self.base_joint = p('base_joint', f'{prefix}base_virtual_joint').value
        self.odom_frame = p('odom_frame', f'{prefix}odom').value
        self.base_frame = p('base_frame', f'{prefix}base_link').value
        self.default_transit = p('default_transit_state', 'stow').value
        self.planning_time = p('planning_time', 5.0).value
        self.planning_attempts = p('planning_attempts', 5).value
        self.state_tolerance = p('named_state_tolerance', 0.01).value  # rad
        self.ik_timeout = p('ik_timeout', 0.2).value  # s, per seed
        self.ik_perturbed_seeds = p('ik_perturbed_seeds', 10).value
        self.ik_seed_noise = p('ik_seed_noise', 0.2).value  # rad, std dev
        self.arm_joints = [f'{prefix}joint{i}' for i in range(1, 7)]
        self.trajectory_controller = p('trajectory_controller', 'arm_trajectory_controller').value
        self.velocity_controller = p('velocity_controller', 'arm_velocity_controller').value
        activate_on_start = p('activate_trajectory_controller_on_start', True).value
        # base_link height above the floor (ranger_mini_v3's wheels bottom
        # out 0.315m below it). <= 0 disables the floor.
        self.floor_height = p('floor_height', 0.315).value

        group = ReentrantCallbackGroup()
        self.move_group = ActionClient(self, MoveGroup, 'move_action', callback_group=group)
        self.base = ActionClient(self, FollowJointTrajectory,
                                 'base_trajectory_controller/follow_joint_trajectory', callback_group=group)
        self.switch_srv = self.create_client(SwitchController, 'controller_manager/switch_controller', callback_group=group)
        self.list_srv = self.create_client(ListControllers, 'controller_manager/list_controllers', callback_group=group)
        self.params_srv = self.create_client(GetParameters, 'move_group/get_parameters', callback_group=group)
        self.ik_srv = self.create_client(GetPositionIK, 'compute_ik', callback_group=group)
        self.scene_srv = self.create_client(ApplyPlanningScene, 'apply_planning_scene', callback_group=group)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.joint_positions = {}
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
        self.get_logger().info('Waiting for move_group...')

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
        if self.floor_height > 0:
            await self._add_floor()
        if self._activate_on_start:
            try:
                await self._ensure_trajectory_controller()
            except StageFailed as e:
                self.get_logger().error(str(e))
        self.get_logger().info('Ready: mobile_manipulation/move_to_goal')

    async def _add_floor(self):
        # In odom as MoveIt's model sees it: the planar base joint keeps
        # base_link at z=0, whatever z odom->base_link has in TF.
        floor = CollisionObject(id='floor', operation=CollisionObject.ADD)
        floor.header.frame_id = self.odom_frame
        thickness = 0.02
        floor.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[200.0, 200.0, thickness])]
        pose = Pose()
        pose.position.z = -self.floor_height - 0.01 - thickness / 2
        pose.orientation.w = 1.0
        floor.primitive_poses = [pose]
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [floor]
        if not self.scene_srv.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('apply_planning_scene not available: no floor in the planning scene')
            return
        res = await self.scene_srv.call_async(ApplyPlanningScene.Request(scene=scene))
        if res.success:
            self.get_logger().info(f'Added floor {self.floor_height + 0.01:.3f}m below base_link to the planning scene')
        else:
            self.get_logger().error('Failed to add the floor to the planning scene')

    def _joint_states(self, msg):
        self.joint_positions.update(zip(msg.name, msg.position))

    # ---- action callbacks ---------------------------------------------------
    def _on_goal(self, goal):
        if self._busy:
            self.get_logger().warn('Rejecting goal: already executing one')
            return GoalResponse.REJECT
        if goal.mode == MoveToGoal.Goal.WHOLE_BODY:
            self.get_logger().warn('Rejecting goal: WHOLE_BODY mode is not implemented yet')
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
        stage = ''
        try:
            if goal.move_base:
                stage = 'transit'
                transit = goal.transit_state or self.default_transit
                if self._at_named_state(transit):
                    self._feedback(goal_handle, stage, f'arm already at {transit!r}')
                else:
                    self._feedback(goal_handle, stage, f'arm to {transit!r}')
                    await self._move_arm(self._named_constraints(transit), goal.velocity_scaling)
                self._check_cancel(goal_handle)

                stage = 'base'
                b = goal.base_goal
                self._feedback(goal_handle, stage, f'base to x={b.x:.3f} y={b.y:.3f} theta={b.theta:.3f}')
                await self._move_base(b.x, b.y, b.theta)
                self._check_cancel(goal_handle)

            if goal.move_arm:
                stage = 'arm'
                if goal.arm_named_goal:
                    self._feedback(goal_handle, stage, f'arm to {goal.arm_named_goal!r}')
                    constraints = self._named_constraints(goal.arm_named_goal)
                else:
                    pos = goal.arm_pose_goal.pose.position
                    self._feedback(goal_handle, stage,
                                   f'arm to pose ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) '
                                   f'in {goal.arm_pose_goal.header.frame_id!r}')
                    constraints = await self._ik_constraints(goal.arm_pose_goal)
                await self._move_arm(constraints, goal.velocity_scaling)

            result.success = True
            result.message = 'done'
            goal_handle.succeed()
        except StageFailed as e:
            result.success = False
            result.message = f'{stage}: {e}'
            self.get_logger().error(result.message)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        finally:
            self._child = None
            self._busy = False
        return result

    # ---- stages ---------------------------------------------------------------
    def _check_cancel(self, goal_handle):
        if goal_handle.is_cancel_requested:
            raise StageFailed('canceled')

    def _feedback(self, goal_handle, stage, status):
        self.get_logger().info(f'[{stage}] {status}')
        goal_handle.publish_feedback(MoveToGoal.Feedback(stage=stage, status=status))

    async def _move_arm(self, constraints, velocity_scaling):
        await self._ensure_trajectory_controller()
        if not self.move_group.wait_for_server(timeout_sec=5.0):
            raise StageFailed('move_group action server not available')
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.arm_group
        req.num_planning_attempts = self.planning_attempts
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = velocity_scaling
        req.max_acceleration_scaling_factor = velocity_scaling
        req.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        result = await self._run_child(self.move_group, goal)
        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            raise StageFailed(f'MoveIt {MOVEIT_ERRORS.get(code, code)}')

    async def _move_base(self, x, y, theta):
        if not self.base.wait_for_server(timeout_sec=5.0):
            raise StageFailed('base_trajectory_server not available')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [f'{self.base_joint}/{a}' for a in ('x', 'y', 'theta')]
        goal.trajectory.points = [JointTrajectoryPoint(positions=[x, y, theta])]
        result = await self._run_child(self.base, goal)
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise StageFailed(f'base {result.error_string or result.error_code}')

    async def _run_child(self, client, goal):
        handle = await client.send_goal_async(goal)
        if not handle.accepted:
            raise StageFailed('goal rejected')
        self._child = handle
        response = await handle.get_result_async()
        self._child = None
        return response.result

    # ---- goals -----------------------------------------------------------------
    def _named_constraints(self, name):
        if name not in self.named_states:
            raise StageFailed(f'unknown named state {name!r} (have {sorted(self.named_states)})')
        c = Constraints(name=name)
        for joint, value in self.named_states[name].items():
            c.joint_constraints.append(JointConstraint(
                joint_name=joint, position=value, tolerance_above=1e-3, tolerance_below=1e-3, weight=1.0))
        return c

    async def _ik_constraints(self, pose):
        """Joint goal for an IK solution near the arm's current joints.

        Seeds, in order: the current joints, small random perturbations of
        them (KDL stalls at the wrist singularity, joint5 ~ 0, which the
        gripper-pointing-down poses sit right next to), then the SRDF
        named states. The first collision-free solution wins, so the
        result stays as close to the current configuration as possible.
        """
        if not self.ik_srv.wait_for_service(timeout_sec=5.0):
            raise StageFailed('compute_ik service not available')
        target = self._to_model_odom(pose)
        current = [self.joint_positions.get(j, 0.0) for j in self.arm_joints]
        rng = np.random.default_rng(0)
        seeds = [current]
        seeds += [list(np.asarray(current) + rng.normal(0.0, self.ik_seed_noise, len(current)))
                  for _ in range(self.ik_perturbed_seeds)]
        seeds += [[state.get(j, 0.0) for j in self.arm_joints] for state in self.named_states.values()]
        code = None
        for seed in seeds:
            req = GetPositionIK.Request()
            ik = req.ik_request
            ik.group_name = self.arm_group
            ik.ik_link_name = self.tcp_link
            ik.pose_stamped = target
            ik.avoid_collisions = True
            ik.timeout = Duration(sec=int(self.ik_timeout), nanosec=int((self.ik_timeout % 1) * 1e9))
            # is_diff: only the arm joints are seeded; everything else,
            # including the base's planar joint, stays at move_group's
            # current state.
            ik.robot_state.is_diff = True
            ik.robot_state.joint_state = JointState(name=self.arm_joints, position=[float(v) for v in seed])
            res = await self.ik_srv.call_async(req)
            code = res.error_code.val
            if code == MoveItErrorCodes.SUCCESS:
                solution = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                c = Constraints(name='tcp_pose')
                for joint in self.arm_joints:
                    c.joint_constraints.append(JointConstraint(
                        joint_name=joint, position=solution[joint], tolerance_above=1e-3, tolerance_below=1e-3, weight=1.0))
                return c
        raise StageFailed(f'no collision-free IK solution for the pose from {len(seeds)} seeds '
                          f'({MOVEIT_ERRORS.get(code, code)})')

    def _to_model_odom(self, pose):
        """Express a PoseStamped in odom as MoveIt's planar-base model sees it."""
        try:
            to_base = self.tf_buffer.lookup_transform(self.base_frame, pose.header.frame_id, rclpy.time.Time())
            base = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())
        except TransformException as e:
            raise StageFailed(f'cannot resolve the pose goal frame: {e}')
        q = base.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        planar = _matrix(base.transform.translation.x, base.transform.translation.y, 0.0,
                         (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
        t, r = to_base.transform.translation, to_base.transform.rotation
        p, o = pose.pose.position, pose.pose.orientation
        m = planar @ _matrix(t.x, t.y, t.z, (r.x, r.y, r.z, r.w)) @ _matrix(p.x, p.y, p.z, (o.x, o.y, o.z, o.w))
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
