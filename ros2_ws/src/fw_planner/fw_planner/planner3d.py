"""3D fixed-wing planning: Dubins airplane (Dubins curve in plan view, altitude change within a climb angle).

Pose: (x north, y east, h altitude up, psi from north toward east). A segment is (curvature, length, dh): a
2D Dubins segment that also changes altitude by dh, linearly, with |dh| <= length * tan(max_climb).
World3D: no-fly cylinders with a bottom and a top altitude, plus a margin, bounds and an altitude band.
plan(): Hybrid A* over arc/straight primitives with climb, level and descend variants, trying a 3D Dubins shot
to the goal at every expansion. A plane can then fly over a low obstacle instead of around it.
"""
import heapq
import itertools
import math

from .dubins import integrate, shortest_path

CHECK_STEP = 2.0  # [m] collision check spacing along a path


class World3D:
    """Cylinders [(x, y, r, bottom, top)], bounds (x_min, x_max, y_min, y_max), altitude band (h_min, h_max)."""

    def __init__(self, obstacles=(), bounds=None, altitudes=(30.0, 150.0), margin=0.0):
        self.obstacles = [(x, y, r + margin, bottom - margin, top + margin) for x, y, r, bottom, top in obstacles]
        self.bounds = None if bounds is None else (
            bounds[0] + margin, bounds[1] - margin, bounds[2] + margin, bounds[3] - margin)
        self.altitudes = altitudes

    def point_free(self, x, y, h):
        if not self.altitudes[0] <= h <= self.altitudes[1]:
            return False
        if self.bounds and not (self.bounds[0] <= x <= self.bounds[1] and self.bounds[2] <= y <= self.bounds[3]):
            return False
        return all((x - ox) ** 2 + (y - oy) ** 2 > r * r or not bottom <= h <= top
                   for ox, oy, r, bottom, top in self.obstacles)

    def path_free(self, start, segments):
        return all(self.point_free(x, y, h) for x, y, h, _ in sample(start, segments, CHECK_STEP))


def integrate3d(pose, segment):
    x, y, psi = integrate((pose[0], pose[1], pose[3]), segment[0], segment[1])
    return x, y, pose[2] + segment[2], psi


def sample(start, segments, step):
    """Points (x, y, h, psi) every `step` metres of horizontal path."""
    points, pose = [start], start
    for k, length, dh in segments:
        n = max(1, math.ceil(length / step))
        for i in range(1, n + 1):
            points.append(integrate3d(pose, (k, length * i / n, dh * i / n)))
        pose = integrate3d(pose, (k, length, dh))
    return points


def shot(start, goal, radius, max_climb):
    """3D Dubins path: the 2D Dubins path with the altitude change spread evenly, or None if too steep."""
    flat = shortest_path((start[0], start[1], start[3]), (goal[0], goal[1], goal[3]), radius)
    length = sum(length for _, length in flat)
    dh = goal[2] - start[2]
    if abs(dh) > length * math.tan(max_climb) + 1e-9:
        return None
    return [(k, l, dh * l / length if length else 0.0) for k, l in flat]


def plan(start, goal, radius, world, max_climb=math.radians(8), heading_bins=16, max_expansions=50000):
    """Segments from start to goal pose (x, y, h, psi) that stay free. Raises RuntimeError if none is found."""
    if not (world.point_free(*start[:3]) and world.point_free(*goal[:3])):
        raise RuntimeError(f'start {start} or goal {goal} is not free')
    step = radius * 2 * math.pi / heading_bins
    climb = step * math.tan(max_climb)
    primitives = [(k, step, dh) for k in (-1 / radius, 0.0, 1 / radius) for dh in (-climb, 0.0, climb)]

    def cell(pose):
        return (round(pose[0] / step), round(pose[1] / step), round(pose[2] / climb),
                round(pose[3] / (2 * math.pi / heading_bins)) % heading_bins)

    def cost_to_go(pose):
        flat = sum(length for _, length in shortest_path((pose[0], pose[1], pose[3]), (goal[0], goal[1], goal[3]), radius))
        return max(flat, abs(goal[2] - pose[2]) / math.tan(max_climb))  # both are lower bounds

    tie = itertools.count()
    nodes = [(start, None, None)]
    open_set = [(cost_to_go(start), next(tie), 0.0, 0)]
    closed = set()
    while open_set and len(closed) < max_expansions:
        _, _, g, idx = heapq.heappop(open_set)
        pose = nodes[idx][0]
        if cell(pose) in closed:
            continue
        closed.add(cell(pose))
        direct = shot(pose, goal, radius, max_climb)
        if direct is not None and world.path_free(pose, direct):
            return _segments_to(nodes, idx) + direct
        for primitive in primitives:
            new = integrate3d(pose, primitive)
            if cell(new) not in closed and world.path_free(pose, [primitive]):
                nodes.append((new, idx, primitive))
                heapq.heappush(open_set, (g + step + cost_to_go(new), next(tie), g + step, len(nodes) - 1))
    raise RuntimeError(f'no 3D path from {start} to {goal} after {len(closed)} expansions')


def _segments_to(nodes, idx):
    segments = []
    while nodes[idx][1] is not None:
        segments.append(nodes[idx][2])
        idx = nodes[idx][1]
    return segments[::-1]


def plan_tour(goals, radius, world, **kwargs):
    """Closed tour through goal poses (x, y, h, psi) and back to the first."""
    segments = []
    for a, b in zip(goals, goals[1:] + goals[:1]):
        segments += plan(a, b, radius, world, **kwargs)
    return segments
