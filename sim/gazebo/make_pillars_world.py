#!/usr/bin/env python3
"""Write the 'pillars' world: PX4's default world with a textured 3 km ground and tall textured pillars.

For stereo obstacle avoidance tests: the baylands trees render black without texture, so stereo cannot see
them. The textures here are multi-scale noise, so stereo finds matches from tens to hundreds of metres.
The pillars stand across the straight route east of the takeoff point (ENU x), at flight altitude and above.
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


def main():
    out = os.path.abspath(sys.argv[1])
    textures = os.path.join(out, 'textures')
    os.makedirs(textures, exist_ok=True)
    ground_png, pillar_png = os.path.join(textures, 'ground.png'), os.path.join(textures, 'pillar.png')
    noise_texture(ground_png, 2048, 1, (95, 110, 80), 28)
    noise_texture(pillar_png, 2048, 2, (150, 120, 100), 45)

    sdf = open(PX4_WORLD).read()
    sdf = sdf.replace('<world name="default">', '<world name="pillars">', 1)
    sdf = re.sub(r'<size>500 500</size>', f'<size>{GROUND_SIZE:.0f} {GROUND_SIZE:.0f}</size>', sdf, count=1)
    ground_material = re.search(r'(<visual name="visual">.*?)<material>.*?</material>', sdf, re.S)
    sdf = sdf[:ground_material.start()] + ground_material.group(1) + (
        f'<material><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{ground_png}</albedo_map></metal></pbr></material>'
    ) + sdf[ground_material.end():]
    models = ''.join(pillar(f'pillar_{i}', x, y, pillar_png) for i, (x, y) in enumerate(PILLARS))
    sdf = sdf.replace('  </world>', models + '\n  </world>', 1)
    with open(os.path.join(out, 'pillars.sdf'), 'w') as f:
        f.write(sdf)
    print(os.path.join(out, 'pillars.sdf'))


if __name__ == '__main__':
    main()
