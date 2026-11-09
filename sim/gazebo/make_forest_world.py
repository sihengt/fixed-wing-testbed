#!/usr/bin/env python3
"""Write the 'forest' world: the pillars world with pillars scattered over the route (LAYOUTS['forest']).

Usage: make_forest_world.py OUT_DIR   (writes OUT_DIR/forest.sdf and OUT_DIR/textures/*.png)
"""
import sys

from make_pillars_world import write_world

if __name__ == '__main__':
    print(write_world(sys.argv[1], 'forest'))
