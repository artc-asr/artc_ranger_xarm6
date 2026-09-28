#!/usr/bin/env python3
"""FAST-LIO .pcd map -> the same points in the map frame, for the localizer.

    ros2 run ranger_xarm6_navigation pcd_to_map_frame.py ~/ranger_xarm6_maps/lab.pcd \\
        --origin 0.917 4.177 0.793 1.5708

The .pcd is in FAST-LIO's 'camera_init' frame; --origin is that frame's
pose in the map frame, x y z yaw: the same numbers pcd_to_grid.py was given
for the 2D grid, so localizing against this cloud puts the robot where
Nav2's grid expects it. Writes <map>_map_frame.pcd (x y z, binary),
thinned to one point per --voxel cube: the localizer's matching voxels
are far coarser, and the full map is ~25 MB to load at every start.
"""
import argparse
import math
import os

import numpy as np

from pcd_to_grid import read_pcd_xyz


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('pcd')
    parser.add_argument('--origin', nargs=4, type=float, required=True, metavar=('X', 'Y', 'Z', 'YAW'),
                        help="camera_init's pose in the map frame (z: IMU height above the floor)")
    parser.add_argument('--voxel', type=float, default=0.05, help='keep one point per cube this size [m]; 0 = all')
    parser.add_argument('--out', help='output .pcd (default: <map>_map_frame.pcd next to the input)')
    args = parser.parse_args()

    pcd = os.path.expanduser(args.pcd)
    out = os.path.expanduser(args.out) if args.out else os.path.splitext(pcd)[0] + '_map_frame.pcd'
    xyz = read_pcd_xyz(pcd)
    x0, y0, z0, yaw = args.origin
    c, s = math.cos(yaw), math.sin(yaw)
    pts = np.stack([x0 + c * xyz[:, 0] - s * xyz[:, 1],
                    y0 + s * xyz[:, 0] + c * xyz[:, 1],
                    z0 + xyz[:, 2]], axis=1)
    if args.voxel > 0:
        _, keep = np.unique(np.floor(pts / args.voxel).astype(np.int64), axis=0, return_index=True)
        pts = pts[np.sort(keep)]
    pts = pts.astype(np.float32)
    with open(out, 'wb') as f:
        f.write((
            '# .PCD v0.7 - Point Cloud Data file format\n'
            'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
            f'WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA binary\n'
        ).encode('ascii'))
        f.write(pts.tobytes())
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    print(f'{out}: {len(pts)} of {len(xyz)} points, '
          f'x {lo[0]:.2f}..{hi[0]:.2f}  y {lo[1]:.2f}..{hi[1]:.2f}  z {lo[2]:.2f}..{hi[2]:.2f} (map frame)')


if __name__ == '__main__':
    main()
