#!/usr/bin/env python3
"""Republish a geometry_msgs/WrenchStamped with a corrected header.frame_id.

gz-sim's force_torque sensor breaks -- stops publishing entirely, no
error, just silence -- if a <gz_frame_id> SDF element is added to it,
unlike every other sensor type in this file (IMU, lidar, cameras' own
<optical_frame_id>), which all support it fine. Verified empirically on
force_torque_sensor (see ranger_xarm6.urdf.xacro): removing <gz_frame_id>
restores data flow immediately; adding it back reproduces the silence
(gz topic advertised, zero messages) every time, reliably. This is the
pre-<gz_frame_id> style workaround (same shape as the old, now-removed
fix_imu_frame_id.py, before gz_frame_id was found to work for that
sensor): subscribe to the raw bridged topic and republish the exact same
data with the real frame_id substituted in.
"""
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


class WrenchFrameIdFixup(Node):
    def __init__(self, in_topic, out_topic, frame_id):
        super().__init__('wrench_frame_id_fixup')
        self.frame_id = frame_id
        self.pub = self.create_publisher(WrenchStamped, out_topic, 10)
        self.create_subscription(WrenchStamped, in_topic, self._cb, 10)

    def _cb(self, msg):
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = WrenchFrameIdFixup(sys.argv[1], sys.argv[2], sys.argv[3])
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
