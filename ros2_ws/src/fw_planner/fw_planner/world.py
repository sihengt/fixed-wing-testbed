"""Planner-independent pieces: the world (no-fly zones, bounds), collision checks, Dubins shortcuts, tours.

Poses and segments follow fw_planner.dubins (PX4 local NED).
"""
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


def connect_poses(poses, radius, world, fallback=None):
    """Join poses with Dubins paths: from each pose, jump to the farthest later pose a free Dubins path reaches.

    If not even the next pose is reachable, use fallback(i) (segments from pose i to i + 1). Without a
    fallback, the Dubins path to the next pose is used even though it is not free.
    """
    result, i = [], 0
    while i < len(poses) - 1:
        for j in range(len(poses) - 1, i, -1):
            shot = shortest_path(poses[i], poses[j], radius)
            if world.path_free(poses[i], shot):
                result += shot
                break
        else:
            j = i + 1
            result += fallback(i) if fallback else shortest_path(poses[i], poses[j], radius)
        i = j
    return result


def shortcut(start, segments, radius, world):
    """Remove wiggles from a free path. A Dubins path is the shortest at this radius, so it never gets longer."""
    poses = [start]
    for segment in segments:
        poses.append(integrate(poses[-1], *segment))
    return connect_poses(poses, radius, world, fallback=lambda i: [segments[i]])


def plan_tour(goals, plan_leg):
    """Closed path through goal poses and back to the first: plan_leg(a, b) -> segments. Starts at goals[0]."""
    segments = []
    for a, b in zip(goals, goals[1:] + goals[:1]):
        segments += plan_leg(a, b)
    return segments
