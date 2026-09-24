#!/usr/bin/env python3
"""Sample random arm/gripper states against a running move_group and report
which link pairs collide, and how often.

Usage (sim + move_group.launch.py running):
    python3 ranger_xarm6_moveit_config/scripts/sample_self_collisions.py [robot_id] [samples]

Pairs colliding in (nearly) every sample are touching/overlapping meshes,
not real collisions: review them, then add them to
config/collision_overrides.yaml and rerun generate_collision_matrix.py.
Pairs colliding only sometimes are real and must stay enabled.
Also reports whether the SRDF's named arm states are collision-free.
"""
import random
import sys
import xml.etree.ElementTree as ET
from collections import Counter

import rclpy
from moveit_msgs.srv import GetStateValidity
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM = [f'joint{i}' for i in range(1, 7)]


def main():
    robot_id = sys.argv[1] if len(sys.argv) > 1 else 'robot_a'
    samples = int(sys.argv[2]) if len(sys.argv) > 2 else 300
    prefix = f'{robot_id}_' if robot_id else ''
    ns = f'/{robot_id}' if robot_id else ''

    rclpy.init()
    node = Node('sample_self_collisions')

    params = node.create_client(GetParameters, f'{ns}/move_group/get_parameters')
    params.wait_for_service()
    fut = params.call_async(GetParameters.Request(names=['robot_description', 'robot_description_semantic']))
    rclpy.spin_until_future_complete(node, fut)
    urdf, srdf = (v.string_value for v in fut.result().values)

    limits = {}
    for j in ET.fromstring(urdf).findall('joint'):
        lim = j.find('limit')
        if lim is not None and j.get('type') == 'revolute':
            limits[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
    named = {
        s.get('name'): {j.get('name'): float(j.get('value')) for j in s.findall('joint')}
        for s in ET.fromstring(srdf).findall('group_state') if s.get('group') == 'arm'
    }

    check = node.create_client(GetStateValidity, f'{ns}/check_state_validity')
    check.wait_for_service()

    def contacts(positions):
        req = GetStateValidity.Request()
        req.group_name = 'arm'
        req.robot_state.joint_state = JointState(name=list(positions), position=list(positions.values()))
        req.robot_state.is_diff = True
        f = check.call_async(req)
        rclpy.spin_until_future_complete(node, f)
        res = f.result()
        pairs = {tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in res.contacts}
        return res.valid, pairs

    for name, state in named.items():
        valid, pairs = contacts(state)
        print(f'named state {name!r}: {"valid" if valid else "IN COLLISION " + str(sorted(pairs))}')

    counts = Counter()
    drive = f'{prefix}drive_joint'
    for _ in range(samples):
        state = {f'{prefix}{j}': random.uniform(*limits[f'{prefix}{j}']) for j in ARM}
        state[drive] = random.uniform(*limits[drive])
        counts.update(contacts(state)[1])

    print(f'\n{samples} random states; colliding pairs (fraction of samples):')
    for (a, b), n in counts.most_common():
        tag = '  <-- always: review for collision_overrides.yaml' if n >= 0.95 * samples else ''
        print(f'  {n / samples:5.2f}  {a.removeprefix(prefix)} / {b.removeprefix(prefix)}{tag}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
