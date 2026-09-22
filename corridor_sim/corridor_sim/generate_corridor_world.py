#!/usr/bin/env python3
"""
Programmatically generates the SDF world for the Shifting Wall &
Slipping Base Challenge corridor: a straight, flat, featureless
corridor plus the dynamic wall panel in its retracted (default) state.
Runtime movement of the panel is handled separately, by a mover node
that calls Gazebo's pose-set service — this script only sets up the
initial static scene.
"""

import os
import xml.etree.ElementTree as ET
from xml.dom import minidom

# --- Corridor parameters (meters) ---
CORRIDOR_LENGTH = 20.0
CORRIDOR_WIDTH = 3.5
CORRIDOR_HEIGHT = 2.5
WALL_THICKNESS = 0.2

# --- Wall panel parameters (meters) ---
PANEL_LENGTH = 2.0
PANEL_HEIGHT = CORRIDOR_HEIGHT
PANEL_THICKNESS = WALL_THICKNESS
PANEL_X = CORRIDOR_LENGTH / 2  # corridor midpoint, X=10
PANEL_SHIFT_DISTANCE = 1.5
PANEL_PROTRUSION = 0.01  # 1cm nudge so panel isn't perfectly coplanar with Wall A (avoids z-fighting)

# Wall A's centerline Y-position — walls always use this, unaffected by the panel nudge
_WALL_A_Y = CORRIDOR_WIDTH / 2 + WALL_THICKNESS / 2
PANEL_RETRACTED_Y = _WALL_A_Y - PANEL_PROTRUSION     # flush-ish against Wall A, slightly proud
PANEL_EXTENDED_Y = PANEL_RETRACTED_Y - PANEL_SHIFT_DISTANCE  # slid into the passage


def add_box_link(model, name, size, pose, color=(0.55, 0.55, 0.55, 1.0)):
    """Adds a single box (floor, wall segment, or panel) to a model element."""
    link = ET.SubElement(model, 'link', name=name)
    ET.SubElement(link, 'pose').text = pose

    for tag in ('collision', 'visual'):
        el = ET.SubElement(link, tag, name=f'{name}_{tag}')
        geometry = ET.SubElement(el, 'geometry')
        box = ET.SubElement(geometry, 'box')
        ET.SubElement(box, 'size').text = f'{size[0]} {size[1]} {size[2]}'
        if tag == 'visual':
            material = ET.SubElement(el, 'material')
            ET.SubElement(material, 'ambient').text = ' '.join(map(str, color))
            ET.SubElement(material, 'diffuse').text = ' '.join(map(str, color))


def build_world():
    sdf = ET.Element('sdf', version='1.9')
    world = ET.SubElement(sdf, 'world', name='corridor_world')

    ET.SubElement(world, 'physics', name='1ms', type='ignored')
    physics = world.find('physics')
    ET.SubElement(physics, 'max_step_size').text = '0.001'
    ET.SubElement(physics, 'real_time_factor').text = '1.0'

    for filename, name in [
        ('gz-sim-physics-system', 'gz::sim::systems::Physics'),
        ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
        ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster'),
    ]:
        ET.SubElement(world, 'plugin', filename=filename, name=name)

    # Sensors system — required for any onboard sensor (LiDAR, IMU, camera,
    # etc.) to actually produce data. Without this, sensor <plugin> blocks
    # declared on the robot are parsed but never instantiated: the sensor
    # topic simply never appears, with no error. Needs its own render_engine
    # child, so it can't be folded into the plain (filename, name) loop above.
    sensors_plugin = ET.SubElement(
        world, 'plugin',
        filename='gz-sim-sensors-system',
        name='gz::sim::systems::Sensors',
    )
    ET.SubElement(sensors_plugin, 'render_engine').text = 'ogre2'

    # IMU system — separate from Sensors above. The IMU sensor doesn't use
    # the rendering pipeline at all (it reads simulated physics state, not
    # a rendered scene), so it needs its own dedicated system plugin rather
    # than piggybacking on the Sensors/render_engine one.
    ET.SubElement(
        world, 'plugin',
        filename='gz-sim-imu-system',
        name='gz::sim::systems::Imu',
    )

    light = ET.SubElement(world, 'light', type='directional', name='sun')
    ET.SubElement(light, 'cast_shadows').text = 'true'
    ET.SubElement(light, 'pose').text = '0 0 10 0 0 0'
    ET.SubElement(light, 'diffuse').text = '0.8 0.8 0.8 1'
    ET.SubElement(light, 'specular').text = '0.2 0.2 0.2 1'
    ET.SubElement(light, 'direction').text = '-0.5 0.1 -0.9'

    floor_model = ET.SubElement(world, 'model', name='floor')
    ET.SubElement(floor_model, 'static').text = 'true'
    add_box_link(
        floor_model, 'floor_link',
        size=(CORRIDOR_LENGTH, CORRIDOR_WIDTH, 0.1),
        pose=f'{CORRIDOR_LENGTH / 2} 0 -0.05 0 0 0',
        color=(0.35, 0.35, 0.35, 1.0),
    )

    wall_a = ET.SubElement(world, 'model', name='wall_a')
    ET.SubElement(wall_a, 'static').text = 'true'
    add_box_link(
        wall_a, 'wall_a_link',
        size=(CORRIDOR_LENGTH, WALL_THICKNESS, CORRIDOR_HEIGHT),
        pose=f'{CORRIDOR_LENGTH / 2} {_WALL_A_Y} {CORRIDOR_HEIGHT / 2} 0 0 0',
    )

    wall_b = ET.SubElement(world, 'model', name='wall_b')
    ET.SubElement(wall_b, 'static').text = 'true'
    add_box_link(
        wall_b, 'wall_b_link',
        size=(CORRIDOR_LENGTH, WALL_THICKNESS, CORRIDOR_HEIGHT),
        pose=f'{CORRIDOR_LENGTH / 2} {-_WALL_A_Y} {CORRIDOR_HEIGHT / 2} 0 0 0',
    )

    # Wall panel — the MODEL carries the absolute world placement (since our
    # runtime mover calls set_pose on this model entity), and the LINK stays
    # at local origin. Keeping absolute coordinates on the link instead would
    # cause them to be added to whatever pose the mover sets on the model,
    # which is exactly the bug that sent the panel flying off to 2x its
    # intended position the first time we moved it at runtime.
    panel = ET.SubElement(world, 'model', name='wall_panel')
    ET.SubElement(panel, 'static').text = 'true'
    ET.SubElement(panel, 'pose').text = f'{PANEL_X} {PANEL_RETRACTED_Y} {PANEL_HEIGHT / 2} 0 0 0'
    add_box_link(
        panel, 'wall_panel_link',
        size=(PANEL_LENGTH, PANEL_THICKNESS, PANEL_HEIGHT),
        pose='0 0 0 0 0 0',  # local origin — model above already carries the absolute placement
        color=(0.85, 0.45, 0.1, 1.0),  # orange — visually distinct from the walls
    )

    return sdf


def main():
    sdf_root = build_world()
    rough = ET.tostring(sdf_root, 'utf-8')
    pretty = minidom.parseString(rough).toprettyxml(indent='  ')

    output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'worlds'))
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, 'corridor.sdf')

    with open(output_path, 'w') as f:
        f.write(pretty)

    print(f'Corridor world written to: {output_path}')
    print(f'Panel retracted Y: {PANEL_RETRACTED_Y}, extended Y: {PANEL_EXTENDED_Y}')


if __name__ == '__main__':
    main()
