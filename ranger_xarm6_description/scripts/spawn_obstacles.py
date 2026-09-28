#!/usr/bin/env python3
"""Spawn random small boxes in artc_lab's open area, for collision avoidance tests.

    ros2 run ranger_xarm6_description spawn_obstacles.py --count 5
    ros2 run ranger_xarm6_description spawn_obstacles.py --count 5 --seed 42   # the same layout again
    ros2 run ranger_xarm6_description spawn_obstacles.py --clear                # remove them

Or at startup: ranger_xarm6.launch.py random_obstacles:=5 (obstacle_seed:=42).
Or as buttons: --serve keeps it running with ROS services
<ns>/obstacles/respawn and <ns>/obstacles/clear (std_srvs/Trigger; see
serve()), which ranger_xarm6_bringup's sim.launch.py starts for Foxglove.

The area is world_plan_view.png's red box (x 2.05-8.65, y 0.96-3.88), left
empty in artc_lab.world and so in the map: whatever lands there is only
seen by the robot's sensors. Boxes are static, random in size (sides
--min-side..--max-side, height --min-height..--max-height, so some are
below what the lidar sees up close) and yaw, kept --inset inside the area
and --gap apart (edge to edge, wider than the robot) so there's always a
way between them. Named obstacle_<i>; each run first removes the ones
already in the world. Waits for the world to exist, so it can start with the sim.
"""
import argparse
import math
import os
import random
import sys
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node

AREA = (2.05, 8.65, 0.96, 3.88)  # x0, x1, y0, y1 (world_plan_view.png, red)
COLOURS = [(0.9, 0.5, 0.1), (0.2, 0.6, 0.9), (0.8, 0.2, 0.6), (0.3, 0.7, 0.3), (0.9, 0.8, 0.2)]


def box_sdf(name, x, y, yaw, sx, sy, sz, rgb):
    r, g, b = rgb
    geom = f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
    return (f'<sdf version="1.9"><model name="{name}"><static>true</static>'
            f'<pose>{x:.3f} {y:.3f} {sz / 2:.3f} 0 0 {yaw:.3f}</pose><link name="link">'
            f'<collision name="collision">{geom}</collision>'
            f'<visual name="visual">{geom}<material><ambient>{r} {g} {b} 1</ambient>'
            f'<diffuse>{r} {g} {b} 1</diffuse></material></visual></link></model></sdf>')


def layout(rng, args):
    """Up to --count boxes: (x, y, yaw, sx, sy, sz), rejection-sampled."""
    x0, x1, y0, y1 = AREA
    boxes = []
    for _ in range(2000):
        if len(boxes) == args.count:
            break
        sx, sy = rng.uniform(args.min_side, args.max_side), rng.uniform(args.min_side, args.max_side)
        sz = rng.uniform(args.min_height, args.max_height)
        radius = math.hypot(sx, sy) / 2  # any yaw fits inside this circle
        x = rng.uniform(x0 + args.inset + radius, x1 - args.inset - radius)
        y = rng.uniform(y0 + args.inset + radius, y1 - args.inset - radius)
        if all(math.hypot(x - bx, y - by) >= radius + br + args.gap for bx, by, br, _ in boxes):
            boxes.append((x, y, radius, (rng.uniform(-math.pi, math.pi), sx, sy, sz)))
    return [(x, y, yaw, sx, sy, sz) for x, y, _, (yaw, sx, sy, sz) in boxes]


def request(node, service, msg, reqtype, timeout_ms=2000):
    ok, rep = node.request(service, msg, reqtype, Boolean, timeout_ms)
    return ok and rep.data


class Spawner:
    """The boxes in a running gz world: wait for it, clear them, spawn a layout."""

    def __init__(self, world):
        self.node = Node()
        self.world = world
        self.create, self.remove = f'/world/{world}/create', f'/world/{world}/remove'

    def wait_for_world(self):
        # Until the world is stepping, not just loaded: requests before that
        # are only queued.
        sim_time = [0.0]
        self.node.subscribe(WorldStatistics, f'/world/{self.world}/stats',
                            lambda m: sim_time.__setitem__(0, m.sim_time.sec + m.sim_time.nsec * 1e-9))
        for i in range(600):
            if sim_time[0] > 1.0 and self.create in self.node.service_list():
                break
            if i % 20 == 0:
                print(f'waiting for world {self.world} to run ...', flush=True)
            time.sleep(0.5)
        else:
            sys.exit(f'world {self.world} is not running')
        self.node.unsubscribe(f'/world/{self.world}/stats')

    def clear(self):
        """Remove the obstacle_<i> models; how many there were."""
        # Only the ones that exist: removing a missing name makes Gazebo log
        # an error. The query fails until this node has discovered the
        # service, so retry.
        for _ in range(20):
            ok, scene = self.node.request(f'/world/{self.world}/scene/info', Empty(), Empty, Scene, 2000)
            if ok:
                break
            time.sleep(0.5)
        else:
            raise RuntimeError(f'/world/{self.world}/scene/info not answering')
        existing = [m.name for m in scene.model if m.name.startswith('obstacle_')]
        for name in existing:
            e = Entity()
            e.name, e.type = name, Entity.MODEL
            request(self.node, self.remove, e, Entity, 2000)
        return len(existing)

    def spawn(self, args, seed):
        """Clear, then a new layout; a one-line summary."""
        self.clear()
        seed = seed if seed >= 0 else random.SystemRandom().randrange(10 ** 6)
        boxes = layout(random.Random(seed), args)
        failed = 0
        for i, (x, y, yaw, sx, sy, sz) in enumerate(boxes):
            f = EntityFactory()
            f.sdf = box_sdf(f'obstacle_{i}', x, y, yaw, sx, sy, sz, COLOURS[i % len(COLOURS)])
            for _ in range(20):  # a removed obstacle_<i> is gone only after the next step
                ok = request(self.node, self.create, f, EntityFactory, 5000)
                if ok:
                    break
                time.sleep(0.25)
            failed += not ok
            print(f'obstacle_{i}: {sx:.2f} x {sy:.2f} x {sz:.2f} m at ({x:.2f}, {y:.2f}), '
                  f'yaw {math.degrees(yaw):.0f} deg{"" if ok else "  FAILED"}', flush=True)
        note = f' (only {len(boxes)} fit with gap {args.gap} m)' if len(boxes) < args.count else ''
        return f'{len(boxes) - failed} obstacles, seed {seed}{note}' + (f', {failed} FAILED' if failed else '')


def serve(args):
    """ROS services <ns>/obstacles/respawn and <ns>/obstacles/clear (std_srvs/Trigger).

    respawn: clear, then 'count' boxes (parameter, default --count) with
    'seed' (parameter; -1: a new layout every call). For buttons, e.g.
    Foxglove's Service Call panel (ranger_xarm6_bringup's layout).
    """
    import rclpy
    from std_srvs.srv import Trigger

    rclpy.init()
    ros = rclpy.create_node('obstacle_spawner')
    count = ros.declare_parameter('count', args.count)
    seed = ros.declare_parameter('seed', args.seed)
    spawner = Spawner(args.world)
    spawner.wait_for_world()

    def respawn(_, reply):
        args.count = ros.get_parameter('count').value
        try:
            reply.message = spawner.spawn(args, ros.get_parameter('seed').value)
            reply.success = True
        except Exception as e:  # noqa: BLE001  report it on the button, keep serving
            reply.message = str(e)
        ros.get_logger().info(f'respawn: {reply.message}')
        return reply

    def clear(_, reply):
        try:
            reply.message = f'removed {spawner.clear()} obstacles'
            reply.success = True
        except Exception as e:  # noqa: BLE001
            reply.message = str(e)
        ros.get_logger().info(f'clear: {reply.message}')
        return reply

    ros.create_service(Trigger, 'obstacles/respawn', respawn)
    ros.create_service(Trigger, 'obstacles/clear', clear)
    ros.get_logger().info(f'obstacles/respawn ({count.value} boxes), obstacles/clear ready')
    try:
        rclpy.spin(ros)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--count', type=int, default=5)
    p.add_argument('--seed', type=int, default=-1, help='-1: a new layout every run')
    p.add_argument('--min-side', type=float, default=0.15)
    p.add_argument('--max-side', type=float, default=0.4)
    p.add_argument('--min-height', type=float, default=0.1)
    p.add_argument('--max-height', type=float, default=0.5)
    p.add_argument('--gap', type=float, default=0.9, help='min free space between boxes [m]')
    p.add_argument('--inset', type=float, default=0.3, help='margin inside the area [m]')
    p.add_argument('--world', default='artc_lab')
    p.add_argument('--clear', action='store_true', help='only remove the boxes')
    p.add_argument('--serve', action='store_true', help='keep running: ROS services to respawn / clear')
    args, _ = p.parse_known_args()  # ignore --ros-args when started by launch

    if args.serve:
        serve(args)
        return
    spawner = Spawner(args.world)
    spawner.wait_for_world()
    if args.clear:
        print(f'removed {spawner.clear()} obstacles')
        return
    print(spawner.spawn(args, args.seed))


if __name__ == '__main__':
    main()
    sys.stdout.flush()
    os._exit(0)  # gz-transport's Node aborts ('terminate called...') when torn down normally
