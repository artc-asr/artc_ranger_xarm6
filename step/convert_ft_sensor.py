#!/usr/bin/env python3
"""Convert the AI1302 F/T sensor STEP into URDF meshes, via OpenCascade (OCP).

Usage: python3 step/convert_ft_sensor.py  (from the workspace root)

Writes ranger_xarm6_description/meshes/ft_sensor_ai1302{,_collision}.stl, in
metres, re-framed so the mesh origin is the adapter's seating face on the
sensor axis, +Z pointing out toward the tool side, and yawed so the adapter's
single protruding dowel pin lands in end_tool_1300.stl's dowel hole at
(0, -25mm). Also prints mass properties: inertia from the solid geometry,
scaled to the manual's 445g (uniform density assumed).
Needs OCP (cadquery's OpenCascade bindings), numpy and trimesh.
"""
import math
import os

import numpy as np
import trimesh
from OCP.BRep import BRep_Tool
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, 'ft_sensor_AI1302_20240411.STEP')
OUT_DIR = os.path.join(HERE, '..', 'ranger_xarm6_description', 'meshes')

# Measured from the STEP itself (all its coaxial cylindrical faces, mm):
AXIS_XY = (-4.2387, 6.676)  # sensor axis
SEAT_Z = 60.5               # adapter's flange-contact face (dowel pin
                            # protrudes 4mm further, to z=56.5)
TOOL_Z = 119.1              # tool-side mounting face
YAW_DEG = 90.0              # dowel pin at 180deg in STEP -> -90deg (-Y)
MASS = 0.445                # kg, UFACTORY F/T sensor manual V2.2.0, spec table
LIN_DEFLECTION = 0.1        # mm, tessellation chord tolerance
ANG_DEFLECTION = 0.5        # rad


def load_step(path):
    reader = STEPControl_Reader()
    if reader.ReadFile(path) != IFSelect_RetDone:
        raise RuntimeError(f'failed to read {path}')
    reader.TransferRoots()
    return reader.OneShape()


def reframe(shape):
    """Move the seat-face/axis point to the origin, then yaw about +Z."""
    move = gp_Trsf()
    move.SetTranslation(gp_Vec(-AXIS_XY[0], -AXIS_XY[1], -SEAT_Z))
    rot = gp_Trsf()
    rot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), math.radians(YAW_DEG))
    return BRepBuilderAPI_Transform(shape, rot.Multiplied(move), True).Shape()


def tessellate(shape):
    BRepMesh_IncrementalMesh(shape, LIN_DEFLECTION, False, ANG_DEFLECTION, True)
    verts, faces, offset = [], [], 0
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(trsf)
                verts.append((p.X(), p.Y(), p.Z()))
            reversed_face = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = (offset + n - 1 for n in tri.Triangle(i).Get())
                faces.append((a, c, b) if reversed_face else (a, b, c))
            offset += tri.NbNodes()
        explorer.Next()
    mesh = trimesh.Trimesh(np.array(verts) * 1e-3, np.array(faces), process=True)
    mesh.merge_vertices()
    return mesh


def main():
    shape = reframe(load_step(SRC))

    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    density = MASS / props.Mass()  # kg/mm^3, uniform
    com = props.CentreOfMass()
    inertia = props.MatrixOfInertia()  # about the COM, mm^5
    I = np.array([[inertia.Value(r, c) for c in (1, 2, 3)] for r in (1, 2, 3)])
    I *= density * 1e-6  # -> kg*m^2

    visual = tessellate(shape)
    # Hull only above the seat face: the dowel pin sinks into link6's flange.
    collision = trimesh.convex.convex_hull(visual.vertices[visual.vertices[:, 2] >= 0.0])
    visual.export(os.path.join(OUT_DIR, 'ft_sensor_ai1302.stl'))
    collision.export(os.path.join(OUT_DIR, 'ft_sensor_ai1302_collision.stl'))

    print(f'visual: {len(visual.faces)} faces, bounds (m)\n{visual.bounds}')
    print(f'collision: {len(collision.faces)} faces')
    print(f'tool face z = {(TOOL_Z - SEAT_Z) * 1e-3:.4f} m')
    print(f'mass = {MASS:.4f} kg (implied density {density * 1e6:.2f} g/cm^3), com (m) = '
          f'{com.X() * 1e-3:.5f} {com.Y() * 1e-3:.5f} {com.Z() * 1e-3:.5f}')
    print('inertia (kg m^2): ixx={:.4e} ixy={:.4e} ixz={:.4e} iyy={:.4e} iyz={:.4e} izz={:.4e}'
          .format(I[0, 0], I[0, 1], I[0, 2], I[1, 1], I[1, 2], I[2, 2]))


if __name__ == '__main__':
    main()
