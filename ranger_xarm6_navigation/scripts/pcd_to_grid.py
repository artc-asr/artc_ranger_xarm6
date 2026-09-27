#!/usr/bin/env python3
"""FAST-LIO .pcd map -> 2D occupancy grid for Nav2 (map_server .pgm + .yaml).

    ros2 run ranger_xarm6_navigation pcd_to_grid.py ~/ranger_xarm6_maps/lab.pcd \\
        --origin 0.917 4.177 0.793 1.5708

The .pcd is in FAST-LIO's 'camera_init' frame (the IMU's pose when
mapping started). --origin is that pose in the map frame, x y z yaw, with
z the IMU's height above the floor, so the map's z = 0 is the floor. In
sim, map = odom = Gazebo's world, and it's the spawn pose composed with
base_link -> <robot_id>_livox_imu_frame (the spawn in README.md's
examples, 0.94 4.35 yaw -90 deg, gives the command above).

A cell is occupied if at least --min-points map points fall in it within
--z-min..--z-max above the floor: the band the robot (arm included) can
hit. --z-min stays above the floor: FAST-LIO's height drifts a few cm,
which lifts the far floor into a low band as a false wall. Everything
else inside the occupied cells' bounding box is free, outside unknown.

--depth (default: <map>_depth.pcd, if it's there): the front depth
camera's voxels from the same mapping session (depth_mapper.py), in the
same frame: the low obstacles the lidar can't see up close. They were cut
to a height band against the floor under the robot when recorded, so
only --z-max applies to them, and each counts as --min-points.
"""
import argparse
import math
import os

import numpy as np


def read_pcd_xyz(path):
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii').strip()
            key, _, value = line.partition(' ')
            header[key] = value.split()
            if key == 'DATA':
                break
        fields, sizes, types = header['FIELDS'], header['SIZE'], header['TYPE']
        n = int(header['POINTS'][0])
        dtype = np.dtype([(name, f'{t.lower()}{s}') for name, s, t in zip(fields, sizes, types)])
        if header['DATA'][0] == 'binary':
            data = np.frombuffer(f.read(n * dtype.itemsize), dtype=dtype, count=n)
        elif header['DATA'][0] == 'ascii':
            data = np.loadtxt(f, dtype=dtype, max_rows=n)
        else:
            raise SystemExit(f'{path}: unsupported PCD DATA {header["DATA"][0]}')
    xyz = np.stack([data['x'], data['y'], data['z']], axis=1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('pcd')
    parser.add_argument('--origin', nargs=4, type=float, default=[0.0, 0.0, 0.0, 0.0], metavar=('X', 'Y', 'Z', 'YAW'),
                        help="camera_init's pose in the map frame (z: IMU height above the floor)")
    parser.add_argument('--depth', help="depth_mapper.py's .pcd (default: <map>_depth.pcd if it exists; '' for none)")
    parser.add_argument('--out', help='output path without extension (default: next to the .pcd)')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--z-min', type=float, default=0.15)
    parser.add_argument('--z-max', type=float, default=1.5)
    parser.add_argument('--min-points', type=int, default=2)
    parser.add_argument('--margin', type=float, default=0.5, help='unknown border around the map [m]')
    args = parser.parse_args()

    pcd = os.path.expanduser(args.pcd)
    out = os.path.expanduser(args.out) if args.out else os.path.splitext(pcd)[0]
    xyz = read_pcd_xyz(pcd)
    depth = os.path.expanduser(args.depth) if args.depth is not None else os.path.splitext(pcd)[0] + '_depth.pcd'
    depth_xyz = read_pcd_xyz(depth) if depth and os.path.exists(depth) else np.zeros((0, 3))

    x0, y0, z0, yaw = args.origin
    c, s = math.cos(yaw), math.sin(yaw)

    def to_map(p):
        return x0 + c * p[:, 0] - s * p[:, 1], y0 + s * p[:, 0] + c * p[:, 1], z0 + p[:, 2]

    mx, my, mz = to_map(xyz)
    band = (mz >= args.z_min) & (mz <= args.z_max)
    mx, my = mx[band], my[band]
    dx, dy, dz = to_map(depth_xyz)
    dx, dy = dx[dz <= args.z_max], dy[dz <= args.z_max]

    res = args.resolution
    lo_x, lo_y = mx.min() - args.margin, my.min() - args.margin
    width = int(math.ceil((mx.max() + args.margin - lo_x) / res))
    height = int(math.ceil((my.max() + args.margin - lo_y) / res))
    ix = ((mx - lo_x) / res).astype(int)
    iy = ((my - lo_y) / res).astype(int)
    counts = np.zeros((height, width), dtype=np.int32)
    np.add.at(counts, (iy, ix), 1)
    occupied = counts >= args.min_points
    jx = ((dx - lo_x) / res).astype(int)
    jy = ((dy - lo_y) / res).astype(int)
    inside = (jx >= 0) & (jx < width) & (jy >= 0) & (jy < height)
    from_depth = np.zeros_like(occupied)
    from_depth[jy[inside], jx[inside]] = True
    from_depth &= ~occupied
    occupied |= from_depth

    # 205 unknown, 254 free, 0 occupied (map_server trinary, negate 0)
    grid = np.full((height, width), 205, dtype=np.uint8)
    rows, cols = np.nonzero(occupied)
    grid[rows.min():rows.max() + 1, cols.min():cols.max() + 1] = 254
    grid[occupied] = 0

    with open(out + '.pgm', 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode())
        f.write(np.flipud(grid).tobytes())  # PGM's first row is the map's top (max y)
    with open(out + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(out)}.pgm\n'
                f'mode: trinary\n'
                f'resolution: {res}\n'
                f'origin: [{lo_x:.3f}, {lo_y:.3f}, 0.0]\n'
                f'negate: 0\n'
                f'occupied_thresh: 0.65\n'
                f'free_thresh: 0.25\n')
    print(f'{len(xyz)} points, {band.sum()} in z {args.z_min}..{args.z_max} m -> '
          f'{width}x{height} cells at {res} m, {occupied.sum()} occupied '
          f'({from_depth.sum()} only from {len(depth_xyz)} depth voxels): {out}.pgm, {out}.yaml')


if __name__ == '__main__':
    main()
