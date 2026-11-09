#!/usr/bin/env python3
"""Write the 'pillars' world: PX4's default world with a textured 3 km ground and tall textured pillars.

For stereo obstacle avoidance tests: the baylands trees render black without texture, so stereo cannot see
them. The textures here are multi-scale noise, so stereo finds matches from tens to hundreds of metres.
The pillars stand across the straight route east of the takeoff point (ENU x), at flight altitude and above.
LAYOUTS also holds the 'forest' world (make_forest_world.py): pillars scattered over the route, not in one line.
Usage: make_pillars_world.py OUT_DIR   (writes OUT_DIR/pillars.sdf and OUT_DIR/textures/*.png)
"""
import os
import re
import sys

import numpy as np
from PIL import Image

PX4_WORLD = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         '../../external/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf')
PILLARS = [(500.0, 0.0), (650.0, 35.0), (650.0, -35.0), (800.0, 0.0), (950.0, 30.0)]  # ENU x, y [m]
PILLAR_RADIUS, PILLAR_HEIGHT = 12.0, 150.0
GROUND_SIZE = 3000.0
FOREST_AREA = (450.0, 1150.0, -250.0, 250.0)  # ENU x min, x max, y min, y max [m], over the route to 1200 m east
FOREST_SPACING = 120.0  # [m] between centres: gaps of 96 m or more, about 2.4 plane turn radii


def forest(seed=5, tries=5000):
    """Random pillars at least FOREST_SPACING apart (dart throwing). Seed 5 puts three on the straight route."""
    rng = np.random.default_rng(seed)
    pillars = []
    for _ in range(tries):
        x, y = rng.uniform(*FOREST_AREA[:2]), rng.uniform(*FOREST_AREA[2:])
        if all(np.hypot(x - px, y - py) >= FOREST_SPACING for px, py in pillars):
            pillars.append((round(float(x), 1), round(float(y), 1)))
    return pillars


LAYOUTS = {'pillars': PILLARS, 'forest': forest()}


def noise_texture(path, size, seed, base, contrast):
    """Sum of blurred noise at several scales, so there is detail at every viewing distance."""
    rng = np.random.default_rng(seed)
    image = np.zeros((size, size))
    for scale in (2, 8, 32, 128):
        coarse = rng.standard_normal((size // scale, size // scale))
        image += np.kron(coarse, np.ones((scale, scale)))[:size, :size] * scale ** 0.5
    image = (image - image.mean()) / image.std()
    rgb = np.clip(np.array(base)[None, None, :] + contrast * image[..., None], 0, 255).astype(np.uint8)
    Image.fromarray(rgb).save(path)


def pillar(name, x, y, texture):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {PILLAR_HEIGHT / 2} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>{PILLAR_RADIUS}</radius><length>{PILLAR_HEIGHT}</length></cylinder></geometry></collision>
        <visual name="visual">
          <geometry><cylinder><radius>{PILLAR_RADIUS}</radius><length>{PILLAR_HEIGHT}</length></cylinder></geometry>
          <material><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{texture}</albedo_map></metal></pbr></material>
        </visual>
      </link>
    </model>"""


def write_world(out, name):
    """Write OUT/NAME.sdf with the pillars of LAYOUTS[NAME]; returns its path."""
    out = os.path.abspath(out)
    textures = os.path.join(out, 'textures')
    os.makedirs(textures, exist_ok=True)
    ground_png, pillar_png = os.path.join(textures, 'ground.png'), os.path.join(textures, 'pillar.png')
    noise_texture(ground_png, 2048, 1, (95, 110, 80), 28)
    noise_texture(pillar_png, 2048, 2, (150, 120, 100), 45)

    sdf = open(PX4_WORLD).read()
    sdf = sdf.replace('<world name="default">', f'<world name="{name}">', 1)
    sdf = re.sub(r'<size>500 500</size>', f'<size>{GROUND_SIZE:.0f} {GROUND_SIZE:.0f}</size>', sdf, count=1)
    ground_material = re.search(r'(<visual name="visual">.*?)<material>.*?</material>', sdf, re.S)
    sdf = sdf[:ground_material.start()] + ground_material.group(1) + (
        f'<material><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{ground_png}</albedo_map></metal></pbr></material>'
    ) + sdf[ground_material.end():]
    models = ''.join(pillar(f'pillar_{i}', x, y, pillar_png) for i, (x, y) in enumerate(LAYOUTS[name]))
    sdf = sdf.replace('  </world>', models + '\n  </world>', 1)
    path = os.path.join(out, f'{name}.sdf')
    with open(path, 'w') as f:
        f.write(sdf)
    return path


if __name__ == '__main__':
    print(write_world(sys.argv[1], 'pillars'))
