#!/usr/bin/env python3
"""Republish a sensor_msgs/Imu with a corrected header.frame_id.

gz-sim's IMU sensor, unlike its camera sensor (see fixed_cam_sensor_tags'
own optical_frame_id in ranger_xarm6.urdf.xacro), has no SDF-level
frame_id override: checked sdformat14's imu.sdf (no such element in its
schema) and the compiled libgz-sensors8-imu.so (no override support, just
a generic internal "frame_id" string). It always reports an auto-
generated scoped entity path instead -- confirmed empirically for
hipnuc_imu_sensor: 'robot_a/robot_a_base_link/robot_a_hipnuc_imu_sensor',
not a real TF frame, so RViz's Imu display (or anything else that needs
to place this data in 3D) can't resolve it -- the same class of problem
optical_frame_id already solves for cameras, just with no equivalent SDF
knob to fix it at the source for this sensor type. This is the standard
workaround: subscribe to the raw bridged topic and republish the exact
same data with the real frame_id substituted in.
"""
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuFrameIdFixup(Node):
    def __init__(self, in_topic, out_topic, frame_id):
        super().__init__('imu_frame_id_fixup')
        self.frame_id = frame_id
        self.pub = self.create_publisher(Imu, out_topic, 10)
        self.create_subscription(Imu, in_topic, self._cb, 10)

    def _cb(self, msg):
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuFrameIdFixup(sys.argv[1], sys.argv[2], sys.argv[3])
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
