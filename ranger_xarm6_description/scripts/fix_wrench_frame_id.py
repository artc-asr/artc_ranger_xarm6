#!/usr/bin/env python3
"""Republish a geometry_msgs/WrenchStamped at the F/T sensor's own frame.

Usage: fix_wrench_frame_id.py IN_TOPIC OUT_TOPIC FRAME_ID [OFFSET_Z]

gz-sim's force_torque sensor breaks -- stops publishing entirely, no
error, just silence -- if a <gz_frame_id> SDF element is added to it,
unlike every other sensor type in this file (IMU, lidar, cameras' own
<optical_frame_id>), which all support it fine. Verified empirically on
force_torque_sensor (see ranger_xarm6.urdf.xacro): removing <gz_frame_id>
restores data flow immediately; adding it back reproduces the silence
(gz topic advertised, zero messages) every time, reliably. This is the
pre-<gz_frame_id> style workaround (same shape as the old, now-removed
fix_imu_frame_id.py, before gz_frame_id was found to work for that
sensor): subscribe to the raw bridged topic and republish the same data
with the real frame_id substituted in.

The sensor also has to sit on joint6 (gz-sim can't measure across a fixed
joint), so its torque is resolved about joint6's origin. OFFSET_Z moves
it to FRAME_ID, which must be parallel to joint6's frame and OFFSET_Z
metres along its +Z (ft_sensor_frame in ranger_xarm6.urdf.xacro):
tau' = tau - r x F with r = (0, 0, OFFSET_Z). Force is unchanged.
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import WrenchStamped


class WrenchFrameIdFixup(Node):
    def __init__(self, in_topic, out_topic, frame_id, offset_z):
        super().__init__('wrench_frame_id_fixup')
        self.frame_id = frame_id
        self.offset_z = offset_z
        self.pub = self.create_publisher(WrenchStamped, out_topic, 10)
        self.create_subscription(WrenchStamped, in_topic, self._cb, 10)

    def _cb(self, msg):
        f, t = msg.wrench.force, msg.wrench.torque
        # r x F for r = (0, 0, z) is (-z*Fy, z*Fx, 0).
        t.x += self.offset_z * f.y
        t.y -= self.offset_z * f.x
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main():
    rclpy.init()
    args = remove_ros_args(sys.argv)
    offset_z = float(args[4]) if len(args) > 4 else 0.0
    node = WrenchFrameIdFixup(args[1], args[2], args[3], offset_z)
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
