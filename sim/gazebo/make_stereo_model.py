#!/usr/bin/env python3
"""Write a copy of PX4's advanced_plane with a stereo camera pair and a ground-truth depth camera.

Generated at run time from the PX4 submodule, so the plane stays in sync with PX4. The cameras sit on
base_link (FLU), pitched down; all three share one pose for the left eye, so the depth camera is
pixel-aligned with the left image. Topics: /stereo/{left,right}/image, /stereo/depth/image, each with
camera_info next to it.

Usage: make_stereo_model.py OUT_DIR [--width W --height H --hfov RAD --baseline M --pitch RAD --rate HZ]
"""
import argparse
import os
import re

PX4_MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         '../../external/PX4-Autopilot/Tools/simulation/gz/models/advanced_plane/model.sdf')
NAME = 'advanced_plane_stereo'


def camera(name, kind, y, args):
    image = '' if kind == 'depth_camera' else '<format>R8G8B8</format>'
    return f"""
      <sensor name="stereo_{name}" type="{kind}">
        <pose>0.3 {y} 0 0 {args.pitch} 0</pose>
        <always_on>1</always_on>
        <update_rate>{args.rate}</update_rate>
        <topic>stereo/{name}/image</topic>
        <gz_frame_id>stereo_left_optical</gz_frame_id>
        <camera>
          <horizontal_fov>{args.hfov}</horizontal_fov>
          <image><width>{args.width}</width><height>{args.height}</height>{image}</image>
          <clip><near>0.1</near><far>1000</far></clip>
        </camera>
      </sensor>"""


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('out_dir')
    # Defaults: cf_drone's simulated ZED X (zedx_info_pub.py), 20 deg down.
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=360)
    parser.add_argument('--hfov', type=float, default=1.8326)
    parser.add_argument('--baseline', type=float, default=0.12)
    parser.add_argument('--pitch', type=float, default=0.35)
    parser.add_argument('--rate', type=float, default=10.0)
    args = parser.parse_args()

    sdf = open(PX4_MODEL).read()
    sdf = re.sub(r"<model name='[^']*'>", f"<model name='{NAME}'>", sdf, count=1)
    half = args.baseline / 2
    sensors = (camera('left', 'camera', half, args) + camera('right', 'camera', -half, args)
               + camera('depth', 'depth_camera', half, args))
    # Into base_link: after its IMU sensor block (the first </sensor> in the file belongs to base_link).
    end = sdf.index('</sensor>') + len('</sensor>')
    sdf = sdf[:end] + sensors + sdf[end:]

    model_dir = os.path.join(args.out_dir, NAME)
    os.makedirs(model_dir, exist_ok=True)
    with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
        f.write(sdf)
    with open(os.path.join(model_dir, 'model.config'), 'w') as f:
        f.write(f'<?xml version="1.0"?><model><name>{NAME}</name><version>1.0</version>'
                '<sdf version="1.9">model.sdf</sdf></model>\n')
    print(os.path.join(model_dir, 'model.sdf'))


if __name__ == '__main__':
    main()
