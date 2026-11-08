"""Hybrid A* for a fixed-wing at constant altitude.

Search over forward primitives (left arc, straight, right arc at the minimum turn radius) and try a
Dubins path to the goal at every expansion. The first collision-free Dubins path ends the search.
Poses and segments follow fw_planner.dubins (PX4 local NED).
"""
import heapq
import itertools
import math

from .dubins import integrate, sample, shortest_path

CHECK_STEP = 2.0  # [m] collision check spacing along a path


class World:
    """Circular no-fly zones [(x, y, r)] and optional bounds (x_min, x_max, y_min, y_max), with a margin."""

    def __init__(self, obstacles=(), bounds=None, margin=0.0):
        self.obstacles = [(x, y, r + margin) for x, y, r in obstacles]
        self.bounds = None if bounds is None else (
            bounds[0] + margin, bounds[1] - margin, bounds[2] + margin, bounds[3] - margin)

    def point_free(self, x, y):
        if self.bounds and not (self.bounds[0] <= x <= self.bounds[1] and self.bounds[2] <= y <= self.bounds[3]):
            return False
        return all((x - ox) ** 2 + (y - oy) ** 2 > r * r for ox, oy, r in self.obstacles)

    def path_free(self, start, segments):
        return all(self.point_free(x, y) for x, y, _, _ in sample(start, segments, CHECK_STEP))


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
            return _segments_to(nodes, idx) + shot

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


def plan_tour(goals, radius, world, **kwargs):
    """Closed path through goal poses and back to the first. Segments start at goals[0]."""
    segments = []
    for a, b in zip(goals, goals[1:] + goals[:1]):
        segments += plan(a, b, radius, world, **kwargs)
    return segments
