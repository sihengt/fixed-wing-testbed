"""Hybrid A* for a fixed-wing at constant altitude.

Search over forward primitives (left arc, straight, right arc at the minimum turn radius) and try a
Dubins path to the goal at every expansion. The first collision-free Dubins path ends the search, then a
shortcut pass removes the wiggles the primitives leave. Poses and segments follow fw_planner.dubins (PX4 local NED).
"""
import heapq
import itertools
import math

from .dubins import integrate, shortest_path
from .world import shortcut


def plan(start, goal, radius, world, heading_bins=16, max_expansions=50000):
    """Segments from start to goal pose that stay free in world. Raises RuntimeError if none is found."""
    if not (world.point_free(*start[:2]) and world.point_free(*goal[:2])):
        raise RuntimeError(f'start {start} or goal {goal} is not free')

    step = radius * 2 * math.pi / heading_bins  # an arc primitive turns exactly one heading bin
    primitives = [(-1 / radius, step), (0.0, step), (1 / radius, step)]

    def cell(pose):
        heading_bin = round(pose[2] / (2 * math.pi / heading_bins)) % heading_bins
        return round(pose[0] / step), round(pose[1] / step), heading_bin

    def cost_to_go(pose):
        # Dubins length ignores obstacles, so it never overestimates.
        return sum(length for _, length in shortest_path(pose, goal, radius))

    tie = itertools.count()
    nodes = [(start, None, None)]  # (pose, parent index, segment from parent)
    open_set = [(cost_to_go(start), next(tie), 0.0, 0)]
    closed = set()
    while open_set and len(closed) < max_expansions:
        _, _, g, idx = heapq.heappop(open_set)
        pose = nodes[idx][0]
        if cell(pose) in closed:
            continue
        closed.add(cell(pose))

        shot = shortest_path(pose, goal, radius)
        if world.path_free(pose, shot):
            return shortcut(start, _segments_to(nodes, idx) + shot, radius, world)

        for primitive in primitives:
            new = integrate(pose, *primitive)
            if cell(new) not in closed and world.path_free(pose, [primitive]):
                nodes.append((new, idx, primitive))
                heapq.heappush(open_set, (g + step + cost_to_go(new), next(tie), g + step, len(nodes) - 1))
    raise RuntimeError(f'no path from {start} to {goal} after {len(closed)} expansions')


def _segments_to(nodes, idx):
    segments = []
    while nodes[idx][1] is not None:
        segments.append(nodes[idx][2])
        idx = nodes[idx][1]
    return segments[::-1]
