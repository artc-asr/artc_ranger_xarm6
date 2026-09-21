#!/usr/bin/env python3
"""Kinematic base driver + sole TF owner for Ranger's odom->base_link.

Ranger has no wired-up drive plugin (its wheels are passive -- see
ranger_mini_v3.xacro's header comment) and no wired-up ros2_control
command interface either (see the state-only <ros2_control> block in
ranger_xarm6.urdf.xacro), so nothing has ever consumed wbc.py's cmd_vel
or moved the base in simulation. This node is a simulation-only stand-in
for that missing drive stage: it integrates cmd_vel kinematically (body-
frame vx/vy/omega -> world-frame x/y/yaw, exactly the twist
wbc_visualize.py's "Initial Position" button already teleports the base
with, and the same twist wbc.py's _arbitrate_ranger_cmd_vel() already
restricts to a mode Ranger's real swerve base can actually execute:
Ackermann/spin OR crab, never both at once), then:
  1. Publishes the result as the live odom->base_link TF (dynamic, not
     static -- this is now a genuinely moving frame), so wbc.py's
     TransformListener-based update_robot_state() sees it every tick.
  2. Teleports the Gazebo entity to match, via the gz-transport Python
     API directly (not the 'gz service' CLI subprocess wbc_visualize.py
     used to shell out to for the one-off Initial Position teleport --
     that's ~fine at one call per button click, far too slow to call
     per control tick; the persistent gz.transport13.Node() API this
     uses instead benchmarks at ~7ms/call, so it can keep up in its own
     thread without blocking TF publishing).

This does NOT simulate real per-wheel swerve kinematics or physics
(no wheel-ground contact, so no collision response e.g. at the wall) --
it's a kinematic teleport, the same "trust the commanded twist" black-box
that the real ranger_ros2 driver is from wbc.py's point of view, just
without the internal wheel-level control. Good enough to let whole-body
path-following actually move the base in sim; not a substitute for
either real physics-based simulation or the real driver on hardware.

Also still the sole owner of odom->base_link for the "Initial Position"
button's teleport (set_base_pose topic) -- see git history on this file
for why a second TF publisher for the same edge is a race, and why
tf2_ros.StaticTransformBroadcaster can't be reused for updates (both
moot now that this publishes a normal dynamic transform instead of a
static one, but the single-owner principle still holds).
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

from gz.transport13 import Node as GzNode
from gz.msgs10 import pose_pb2, boolean_pb2, scene_pb2, empty_pb2


class BasePosePublisher(Node):
    def __init__(self):
        super().__init__('base_pose_publisher')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        # Gazebo teleport target -- must match how the entity was spawned
        # (see ranger_xarm6.launch.py's spawn_entity_node '-name' arg) and
        # which world it lives in (see tested_world.world's world name,
        # kept as 'robotnik_simple' on purpose -- see that file's header).
        self.declare_parameter('gz_world', 'robotnik_simple')
        self.declare_parameter('gz_entity_name', 'robot_a')
        # Hard safety clamp: this is a *kinematic* teleport, not physics, so
        # it has no collision awareness of its own -- confirmed the hard way
        # (see docs/initial_position_ik_notes.md): a whole-body-QP tracking
        # instability commanded the base toward the wall, and this node
        # obediently teleported the chassis to x=3.045 -- *inside* the wall
        # (collision box spans x=2.85..3.15) -- with no physics step in
        # between to stop it. Once embedded, ODE's stiff contact response
        # to that overlap spiked to multi-million-newton force readings
        # every subsequent tick, corrupting the whole force-control loop
        # downstream. Wall face is at world x=2.85 (see
        # tested_world.world); chassis half-length is ~0.25m (see
        # ranger_mini_v3_description's collision box), so clamping the
        # base origin's x below that leaves the chassis short of the wall
        # face regardless of what the controller commands.
        self.declare_parameter('max_x', 2.6)
        self.declare_parameter('tf_rate', 50.0)         # [Hz] cmd_vel integration + TF publish
        self.declare_parameter('gz_teleport_rate', 30.0)  # [Hz] Gazebo entity teleport (own thread)
        self.declare_parameter('cmd_vel_timeout', 0.5)    # [s] stale cmd_vel -> treat as zero

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.gz_world = self.get_parameter('gz_world').value
        self.gz_entity_name = self.get_parameter('gz_entity_name').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.max_x = self.get_parameter('max_x').value

        self.lock = threading.Lock()
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.z = self.get_parameter('z').value
        self.yaw = self.get_parameter('yaw').value
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.last_cmd_vel_time = None  # monotonic seconds; None = never received one

        self.broadcaster = TransformBroadcaster(self)
        self.gz_node = GzNode()
        self._wait_for_entity()
        self._gz_set_pose(self.x, self.y, self.z, self.yaw)
        self._publish_tf()

        self.create_subscription(Pose2D, 'set_base_pose', self._set_base_pose_cb, 10)
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)

        self._last_tick = self.get_clock().now()
        tf_rate = self.get_parameter('tf_rate').value
        self.create_timer(1.0 / tf_rate, self._tick)

        gz_rate = self.get_parameter('gz_teleport_rate').value
        self._gz_thread = threading.Thread(
            target=self._gz_teleport_loop, args=(1.0 / gz_rate,), daemon=True)
        self._gz_thread.start()

        self.get_logger().info(
            f'Base pose publisher ready ({self.frame_id} -> {self.child_frame_id}), '
            f'driving gz entity "{self.gz_entity_name}" in world "{self.gz_world}"')

    # Body-frame twist from wbc.py (or anything else) -- same convention
    # wbc.py publishes for Ranger: angular.z=omega, linear.x=vx, linear.y=vy.
    def _cmd_vel_cb(self, msg):
        with self.lock:
            self.vx = msg.linear.x
            self.vy = msg.linear.y
            self.omega = msg.angular.z
            self.last_cmd_vel_time = time.monotonic()

    # Absolute teleport (the "Initial Position" button). Zeroes any
    # in-flight cmd_vel so a stale command can't immediately start
    # dragging the base away from the pose it was just teleported to.
    def _set_base_pose_cb(self, msg):
        x = min(msg.x, self.max_x)
        with self.lock:
            self.x, self.y, self.yaw = x, msg.y, msg.theta
            self.vx = self.vy = self.omega = 0.0
            self.last_cmd_vel_time = None
        self._gz_set_pose(x, msg.y, self.z, msg.theta)
        self._publish_tf()
        self.get_logger().info(f'Base pose set: x={x} y={msg.y} yaw={msg.theta}')

    # Integrate the latest cmd_vel by the actual elapsed *simulated* time
    # since the last tick (not the nominal timer period, and not wall-clock
    # elapsed time either -- see below) and publish the result as TF. Runs
    # at tf_rate, independent of the slower Gazebo-teleport thread.
    #
    # Uses self.get_clock().now() (sim-time-aware, given this node's
    # use_sim_time param -- see ranger_xarm6.launch.py) for the integration
    # dt, NOT time.monotonic(): confirmed the hard way (see
    # docs/initial_position_ik_notes.md) that Gazebo's real_time_factor
    # drops well below 1.0 under the load a whole-body-QP + physics run
    # produces (~0.3 observed). A wall-clock dt under RTF<1 integrates the
    # base *faster* than the simulated world it's supposedly moving
    # through -- e.g. at RTF=0.3, a 1-second wall-clock gap is really only
    # ~0.3s of simulated time, so a wall-clock dt overshoots the true
    # simulated displacement by ~3x. wbc.py's own dt had the same bug
    # (fixed, not measured; see its control_loop()) and needed the same
    # fix, since it's what actually drives the cmd_vel this integrates.
    def _tick(self):
        now = time.monotonic()  # only for the cmd_vel staleness watchdog below
        sim_now = self.get_clock().now()
        dt = (sim_now - self._last_tick).nanoseconds / 1e9
        self._last_tick = sim_now

        with self.lock:
            if (self.last_cmd_vel_time is not None
                    and now - self.last_cmd_vel_time > self.cmd_vel_timeout):
                self.vx = self.vy = self.omega = 0.0
            vx, vy, omega = self.vx, self.vy, self.omega
            yaw = self.yaw
            # Body-frame twist -> world-frame pose update.
            c, s = math.cos(yaw), math.sin(yaw)
            self.x += (vx * c - vy * s) * dt
            self.y += (vx * s + vy * c) * dt
            self.yaw += omega * dt
            # Safety clamp -- see max_x's declare_parameter comment. Clamping
            # position (not velocity) is deliberate: it's a hard backstop
            # against a bad command actually reaching the wall, not a
            # smooth limit the controller is expected to respect.
            if self.x > self.max_x:
                self.x = self.max_x

        self._publish_tf()

    def _publish_tf(self):
        with self.lock:
            x, y, z, yaw = self.x, self.y, self.z, self.yaw
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.broadcaster.sendTransform(t)

    # Runs in its own thread since each gz-transport request blocks
    # (~7ms measured) -- calling it from the tf_rate timer would eat into
    # that budget and risk falling behind on TF publishing.
    def _gz_teleport_loop(self, period):
        while rclpy.ok():
            with self.lock:
                x, y, z, yaw = self.x, self.y, self.z, self.yaw
            self._gz_set_pose(x, y, z, yaw)
            time.sleep(period)

    def _gz_set_pose(self, x, y, z, yaw):
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        req = pose_pb2.Pose()
        req.name = self.gz_entity_name
        req.position.x, req.position.y, req.position.z = x, y, z
        req.orientation.x, req.orientation.y = qx, qy
        req.orientation.z, req.orientation.w = qz, qw
        try:
            result, response = self.gz_node.request(
                f'/world/{self.gz_world}/set_pose', req,
                pose_pb2.Pose, boolean_pb2.Boolean, 200)
            if not (result and response.data):
                self.get_logger().warn(
                    f'gz set_pose rejected for entity "{self.gz_entity_name}" '
                    f'(result={result}, ok={response.data if result else None})',
                    throttle_duration_sec=2.0)
                return False
            return True
        except Exception as e:
            self.get_logger().warn(f'gz set_pose failed: {e}', throttle_duration_sec=2.0)
            return False

    # Blocks (bounded) until the Gazebo entity actually exists, so the
    # startup teleport and the periodic teleport thread don't race the
    # separate 'create' spawner node. Polls the world's scene/info service
    # (Empty -> Scene, a read-only query) rather than retrying set_pose
    # itself: set_pose against a not-yet-existing entity makes Gazebo's own
    # C++ UserCommands plugin log '[Err] Unable to update the pose for
    # entity id:[0]' to its console on *every* rejected attempt -- that's
    # server-side logging this node can't suppress from the client side no
    # matter what it does with the (previously entirely discarded)
    # request() return value, so the fix is to not call set_pose at all
    # until scene/info confirms the entity is actually there.
    def _wait_for_entity(self, timeout_sec=15.0, poll_period=0.1):
        req = empty_pb2.Empty()
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            try:
                result, scene = self.gz_node.request(
                    f'/world/{self.gz_world}/scene/info', req,
                    empty_pb2.Empty, scene_pb2.Scene, 200)
                if result and any(m.name == self.gz_entity_name for m in scene.model):
                    return
            except Exception:
                pass
            time.sleep(poll_period)
        self.get_logger().warn(
            f'Gazebo entity "{self.gz_entity_name}" in world "{self.gz_world}" '
            f'never became available after {timeout_sec}s; proceeding anyway.')


def main(args=None):
    rclpy.init(args=args)
    node = BasePosePublisher()
    try:
        rclpy.spin(node)
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
