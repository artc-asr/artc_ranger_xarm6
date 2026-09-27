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

Poses are in frame_id (the robot's odom frame): gz-sim's world origin and
odom coincide, since base_pose_publisher.py starts odom -> base_link at
the spawn pose and teleports the gz entity to follow it.

move_group's subscription keeps no history, so the objects are
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
        self.objects = self.load(world_file)
        self.pub = self.create_publisher(CollisionObject, 'collision_object', 10)
        self.subscribers = 0
        self.create_timer(1.0, self.check_subscribers)
        self.get_logger().info(
            f"{len(self.objects)} static models from '{world_file}' -> collision_object ({self.frame_id}): "
            f"{', '.join(o.id for o in self.objects)}")

    def load(self, path):
        world = ET.parse(path).getroot().find('world')
        objects = []
        for model in world.findall('model'):
            if model.findtext('static', 'false').strip() not in ('true', '1'):
                continue
            obj = CollisionObject(id=f'world/{model.get("name")}', operation=CollisionObject.ADD)
            obj.header.frame_id = self.frame_id
            model_m = pose_matrix(model.findtext('pose'))
            for link in model.findall('link'):
                link_m = model_m @ pose_matrix(link.findtext('pose'))
                for collision in link.findall('collision'):
                    shape = primitive(collision.find('geometry'))
                    if shape is None:
                        continue
                    m = link_m @ pose_matrix(collision.findtext('pose'))
                    pose = Pose()
                    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in m[:3, 3])
                    qx, qy, qz, qw = quaternion_from_matrix(m)
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
                    obj.primitives.append(shape)
                    obj.primitive_poses.append(pose)
            if obj.primitives:
                objects.append(obj)
        return objects

    def check_subscribers(self):
        count = self.pub.get_subscription_count()
        if count > self.subscribers:
            for obj in self.objects:
                obj.header.stamp = self.get_clock().now().to_msg()
                self.pub.publish(obj)
            self.get_logger().info(f'Published {len(self.objects)} collision objects ({count} subscriber(s))')
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
