#!/usr/bin/env python3
"""Generate artc_lab.world, the pick-and-place / navigation test room.

Usage: python3 ranger_xarm6_description/worlds/generate_artc_lab.py

Layout traced from world_plan_view.png (repo root), scaled to a 10m x
5.2m room: the drawing's border spans 1381 x 715 px, so 1px ~ 7.2mm.
World frame: origin at the room's bottom-left corner as drawn, +x to the
right, +y up the drawing, z up. The red area in the drawing (x 2.05-8.65,
y 0.96-3.88) is deliberately left empty: small obstacles get spawned
there at runtime for online collision avoidance, so they must not end up
in the map.

Robot home (blue box in the drawing): centre (0.94, 4.35), front facing
down the drawing (-y), i.e. spawn with x:=0.94 y:=4.35 yaw:=-1.5708.
"""
import os

ROOM_X, ROOM_Y = 10.0, 5.2
WALL_T, WALL_H = 0.1, 2.0
TABLE_H, TABLE_TOP_T, LEG_W = 0.75, 0.04, 0.05
CUBE = 0.05

# (name, x_min, x_max, y_min, y_max) in metres, from the drawing, pulled
# 2cm off any wall they touch.
TABLES = [
    ('table_north_west', 2.085, 4.917, 3.964, 5.18),
    ('table_north_east', 5.981, 8.783, 3.949, 5.18),
    ('table_east', 8.776, 9.98, 0.865, 3.375),
    ('table_south', 4.004, 6.488, 0.02, 0.909),
]
CUPBOARD = ('cupboard', 9.102, 9.98, 3.789, 5.18, 1.8)
PLANT = ('plant', 9.49, 0.345)  # name, centre x, centre y (0.98 x 0.69 m spot)

# Cupboard: Gazebo Fuel's OpenRobotics/Cabinet (Nate Koenig, CC0), an open
# wooden shelf built from 2cm box plates (0.45 x 0.45 x 1.02m, back plate
# on +x), stretched to the drawing's footprint. Open side faces -x, into
# the room. Its Gazebo/Wood material is a Gazebo-classic script that
# Harmonic ignores, so the colour is set explicitly.
CABINET_PLATE = 0.02
# Plant foliage: Gazebo Fuel's slhdn/plant (big_plant.stl, Alireza Ahmadi,
# CC-BY 4.0), copied into meshes/. The raw mesh is a 28 x 29 x 15cm leaf
# rosette with its bbox off-centre; scaled up and set on a pot. PLANT_MESH_*
# are the raw mesh's bbox size, centre (x, y) and min z, measured from the STL.
PLANT_MESH = 'package://ranger_xarm6_description/meshes/big_plant.stl'
PLANT_MESH_SIZE = (0.285, 0.291, 0.152)
PLANT_MESH_CX, PLANT_MESH_CY, PLANT_MESH_Z0 = 0.031, 0.020, -0.004
PLANT_SCALE = (2.3, 2.3, 3.5)  # -> ~0.66 x 0.67 x 0.53m; taller than wide
POT_R, POT_H = 0.22, 0.4

# Cubes near each table's aisle-side edge (the xArm reaches ~0.5m past
# the robot's rear, so objects deep in a 1.2m table would be out of reach).
# (table, x, y, rgb)
RED, GREEN, BLUE, YELLOW = (0.8, 0.1, 0.1), (0.1, 0.7, 0.1), (0.1, 0.2, 0.8), (0.9, 0.8, 0.1)
CUBES = [
    ('table_north_west', 2.6, 4.12, RED),
    ('table_north_west', 3.5, 4.10, GREEN),
    ('table_north_west', 4.4, 4.14, BLUE),
    ('table_north_east', 6.5, 4.10, YELLOW),
    ('table_north_east', 7.4, 4.12, RED),
    ('table_north_east', 8.3, 4.10, GREEN),
    ('table_east', 8.93, 1.4, BLUE),
    ('table_east', 8.93, 2.8, YELLOW),
    ('table_south', 4.6, 0.75, GREEN),
    ('table_south', 5.9, 0.75, RED),
]


def box(name, size, pose, rgb, collide=True):
    sx, sy, sz = size
    x, y, z = pose
    geom = f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
    col = f'<collision name="{name}_collision">{geom}</collision>' if collide else ''
    r, g, b = rgb
    return (f'        <link name="{name}"><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>{col}'
            f'<visual name="{name}_visual">{geom}<material><ambient>{r} {g} {b} 1</ambient>'
            f'<diffuse>{r} {g} {b} 1</diffuse></material></visual></link>')


def static_model(name, links):
    body = '\n'.join(links)
    return f'    <model name="{name}">\n      <static>true</static>\n{body}\n    </model>\n'


def table(name, x0, x1, y0, y1):
    wood, steel = (0.55, 0.4, 0.25), (0.3, 0.3, 0.3)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    links = [box('top', (x1 - x0, y1 - y0, TABLE_TOP_T), (cx, cy, TABLE_H - TABLE_TOP_T / 2), wood)]
    leg_h = TABLE_H - TABLE_TOP_T
    inset = LEG_W / 2 + 0.03
    for i, (lx, ly) in enumerate([(x0 + inset, y0 + inset), (x1 - inset, y0 + inset),
                                  (x0 + inset, y1 - inset), (x1 - inset, y1 - inset)]):
        links.append(box(f'leg{i}', (LEG_W, LEG_W, leg_h), (lx, ly, leg_h / 2), steel))
    return static_model(name, links)


def cabinet(name, x0, x1, y0, y1, h):
    wood, t = (0.55, 0.4, 0.25), CABINET_PLATE
    cx, cy, dx, dy = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
    return static_model(name, [
        box('bottom', (dx, dy, t), (cx, cy, t / 2), wood),
        box('middle', (dx, dy, t), (cx, cy, h / 2), wood),
        box('top', (dx, dy, t), (cx, cy, h - t / 2), wood),
        box('back', (t, dy, h), (x1 - t / 2, cy, h / 2), wood),
        box('left', (dx, t, h), (cx, y1 - t / 2, h / 2), wood),
        box('right', (dx, t, h), (cx, y0 + t / 2, h / 2), wood),
    ])


def plant(name, px, py):
    sx, sy, sz = PLANT_SCALE
    mx, my, mz = px - PLANT_MESH_CX * sx, py - PLANT_MESH_CY * sy, POT_H - PLANT_MESH_Z0 * sz
    leaf_h = PLANT_MESH_SIZE[2] * sz
    leaf_r = max(PLANT_MESH_SIZE[0] * sx, PLANT_MESH_SIZE[1] * sy) / 2
    return f'''    <model name="{name}">
      <static>true</static>
      <link name="pot"><pose>{px} {py} {POT_H / 2} 0 0 0</pose>
        <collision name="c"><geometry><cylinder><radius>{POT_R}</radius><length>{POT_H}</length></cylinder></geometry></collision>
        <visual name="v"><geometry><cylinder><radius>{POT_R}</radius><length>{POT_H}</length></cylinder></geometry><material><ambient>0.45 0.25 0.15 1</ambient><diffuse>0.45 0.25 0.15 1</diffuse></material></visual></link>
      <link name="foliage"><pose>{mx:.3f} {my:.3f} {mz:.3f} 0 0 0</pose>
        <collision name="c"><pose>{PLANT_MESH_CX * sx:.3f} {PLANT_MESH_CY * sy:.3f} {PLANT_MESH_Z0 * sz + leaf_h / 2:.3f} 0 0 0</pose><geometry><cylinder><radius>{leaf_r:.3f}</radius><length>{leaf_h:.3f}</length></cylinder></geometry></collision>
        <visual name="v"><geometry><mesh><uri>{PLANT_MESH}</uri><scale>{sx} {sy} {sz}</scale></mesh></geometry><material><ambient>0.15 0.5 0.15 1</ambient><diffuse>0.15 0.5 0.15 1</diffuse></material></visual></link>
    </model>
'''


def cube(i, x, y, rgb):
    r, g, b = rgb
    m = 0.05
    inertia = m * CUBE * CUBE / 6
    geom = f'<geometry><box><size>{CUBE} {CUBE} {CUBE}</size></box></geometry>'
    return f'''    <model name="cube_{i}">
      <pose>{x:.3f} {y:.3f} {TABLE_H + CUBE / 2 + 0.001:.3f} 0 0 0</pose>
      <link name="link">
        <inertial><mass>{m}</mass><inertia><ixx>{inertia:.3e}</ixx><iyy>{inertia:.3e}</iyy><izz>{inertia:.3e}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="collision">{geom}<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="visual">{geom}<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material></visual>
      </link>
    </model>
'''


def main():
    wall = (0.85, 0.85, 0.82)
    walls = static_model('walls', [
        box('south', (ROOM_X + 2 * WALL_T, WALL_T, WALL_H), (ROOM_X / 2, -WALL_T / 2, WALL_H / 2), wall),
        box('north', (ROOM_X + 2 * WALL_T, WALL_T, WALL_H), (ROOM_X / 2, ROOM_Y + WALL_T / 2, WALL_H / 2), wall),
        box('west', (WALL_T, ROOM_Y, WALL_H), (-WALL_T / 2, ROOM_Y / 2, WALL_H / 2), wall),
        box('east', (WALL_T, ROOM_Y, WALL_H), (ROOM_X + WALL_T / 2, ROOM_Y / 2, WALL_H / 2), wall),
    ])
    cupboard = cabinet(*CUPBOARD)
    plant_model = plant(*PLANT)
    models = walls + ''.join(table(*t) for t in TABLES) + cupboard + plant_model
    models += ''.join(cube(i, x, y, rgb) for i, (_, x, y, rgb) in enumerate(CUBES))

    world = f'''<?xml version="1.0" ?>
<!--
  GENERATED by worlds/generate_artc_lab.py from world_plan_view.png: edit
  that script, not this file.
  10m x 5.2m lab: four 0.75m tables with 5cm cubes for pick and place, a
  cupboard, a plant. Origin at the room's bottom-left corner (as drawn),
  +x right, +y up the drawing. The middle of the room is left empty for
  runtime obstacles. Robot home: x:=0.94 y:=4.35 yaw:=-1.5708.
-->
<sdf version="1.9">
  <world name="artc_lab">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <plugin name="gz::sim::systems::Physics" filename="libgz-sim-physics-system.so"/>
    <plugin name="gz::sim::systems::UserCommands" filename="libgz-sim-user-commands-system.so"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="libgz-sim-scene-broadcaster-system.so"/>
    <!-- No gz::sim::systems::Sensors here: the robot's URDF registers it
         (loading it twice crashes Ogre2, see tested_world.world). -->
    <plugin name="gz::sim::systems::Imu" filename="libgz-sim-imu-system.so"/>
    <plugin name="gz::sim::systems::Contact" filename="libgz-sim-contact-system.so"/>
    <plugin name="gz::sim::systems::ForceTorque" filename="libgz-sim-forcetorque-system.so"/>

    <scene>
      <ambient>0.8 0.8 0.8</ambient>
      <background>0.7 0.7 0.7</background>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>5 2.6 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.2 -1</direction>
    </light>

    <!-- Floor visual sized to the room (walls included), like a
         CoppeliaSim floor; the collision plane is infinite regardless. -->
    <model name="ground_plane">
      <static>true</static>
      <pose>{ROOM_X / 2} {ROOM_Y / 2} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>{ROOM_X + 2 * WALL_T:.1f} {ROOM_Y + 2 * WALL_T:.1f}</size></plane></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual>
      </link>
    </model>

{models}  </world>
</sdf>
'''
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'artc_lab.world')
    with open(out, 'w') as f:
        f.write(world)
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
