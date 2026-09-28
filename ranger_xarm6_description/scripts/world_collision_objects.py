#!/usr/bin/env python3
"""Publish a Gazebo world's static models as MoveIt collision objects.

MoveIt only plans around what's in its planning scene; the world's walls,
tables etc. live only inside gz-sim. This node parses the same SDF file
gz-sim loaded (like world_markers.py does for RViz) and publishes each
static model's <collision> geometry (box, cylinder, sphere; planes and
meshes are skipped) as one moveit_msgs/CollisionObject on
'collision_object', which move_group's planning scene monitor listens to.
Non-static models (e.g. artc_lab.world's cubes) are left out: they move,
and a stale copy would block the arm from reaching them.

The world file's poses are in world_frame (the map frame: gz-sim's world
origin is the map's). They're sent in frame_id, MoveIt's planning frame
(odom), through the current map -> odom (map_to_odom.py): MoveIt converts
an object into its planning frame once, when it arrives, so with the EKF
and a localizer (odom drifts, map -> odom corrects it) they're sent again
whenever map -> odom (smoothed, 5 s) has moved 2 cm or 0.6 deg. Without a map frame (no
Nav2), map = odom, right for the sim's ground-truth odom.

move_group's subscription keeps no history, so the objects are also
(re)published whenever the number of subscribers grows: whenever
move_group starts or restarts, it gets the world. ADD replaces an object
of the same id, so repeats are harmless.
"""
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from tf_transformations import euler_matrix, quaternion_from_matrix

from map_to_odom import MapToOdom


def pose_matrix(text):
    """SDF '<pose>x y z roll pitch yaw</pose>' -> 4x4 homogeneous matrix."""
    v = [float(x) for x in (text or '').split()] + [0.0] * 6
    m = euler_matrix(v[3], v[4], v[5], 'sxyz')
    m[:3, 3] = v[:3]
    return m


def primitive(geometry):
    shape = geometry[0] if geometry is not None and len(geometry) else None
    if shape is None:
        return None
    if shape.tag == 'box':
        return SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in shape.findtext('size').split()])
    if shape.tag == 'cylinder':
        return SolidPrimitive(type=SolidPrimitive.CYLINDER,
                              dimensions=[float(shape.findtext('length')), float(shape.findtext('radius'))])
    if shape.tag == 'sphere':
        return SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[float(shape.findtext('radius'))])
    return None


class WorldCollisionObjects(Node):
    def __init__(self):
        super().__init__('world_collision_objects')
        world_file = self.declare_parameter('world_file', '').value
        self.frame_id = self.declare_parameter('frame_id', 'odom').value
        world_frame = self.declare_parameter('world_frame', 'map').value
        self.objects = self.load(world_file)
        self.map_to_odom = MapToOdom(self, world_frame, self.frame_id)
        self.pub = self.create_publisher(CollisionObject, 'collision_object', 10)
        self.subscribers = 0
        self.create_timer(1.0, self.check)
        self.get_logger().info(
            f"{len(self.objects)} static models from '{world_file}' ({world_frame}) -> collision_object "
            f"({self.frame_id}): {', '.join(o[0] for o in self.objects)}")

    def load(self, path):
        world = ET.parse(path).getroot().find('world')
        objects = []
        for model in world.findall('model'):
            if model.findtext('static', 'false').strip() not in ('true', '1'):
                continue
            parts = []  # (shape, pose in the world frame)
            model_m = pose_matrix(model.findtext('pose'))
            for link in model.findall('link'):
                link_m = model_m @ pose_matrix(link.findtext('pose'))
                for collision in link.findall('collision'):
                    shape = primitive(collision.find('geometry'))
                    if shape is None:
                        continue
                    parts.append((shape, link_m @ pose_matrix(collision.findtext('pose'))))
            if parts:
                objects.append((f'world/{model.get("name")}', parts))
        return objects

    def check(self):
        count = self.pub.get_subscription_count()
        m = self.map_to_odom.update()
        if count > self.subscribers or (count and self.map_to_odom.moved()):
            for name, parts in self.objects:
                obj = CollisionObject(id=name, operation=CollisionObject.ADD)
                obj.header.frame_id = self.frame_id
                obj.header.stamp = self.get_clock().now().to_msg()
                for shape, world_m in parts:
                    p = m @ world_m
                    pose = Pose()
                    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in p[:3, 3])
                    qx, qy, qz, qw = quaternion_from_matrix(p)
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
                    obj.primitives.append(shape)
                    obj.primitive_poses.append(pose)
                self.pub.publish(obj)
            self.map_to_odom.mark_sent()
            self.get_logger().info(
                f'Published {len(self.objects)} collision objects ({count} subscriber(s)), '
                f'map -> odom at ({m[0, 3]:.3f}, {m[1, 3]:.3f})')
        self.subscribers = count


def main():
    rclpy.init()
    node = WorldCollisionObjects()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
