"""One entry point for every tour planner, so the node and the benchmark run the same code.

- hybrid_astar: Hybrid A* with Dubins shots (fw_planner.hybrid_astar)
- rrt_star: RRT* in Dubins space (OMPL)
- jps3d_raw: cf_drone's jps3d grid path, flown as is (point turns)
- jps3d_fillet: the same grid path with Dubins fillets at the minimum turn radius
- jps3d_guided: the grid path as waypoints for Hybrid A* (route from jps3d, flyable legs from Hybrid A*)
"""
import math

from . import hybrid_astar, rrt_star
from .polyline import fillet_segments, raw_segments
from .world import plan_tour

ALGORITHMS = ('hybrid_astar', 'rrt_star', 'jps3d_raw', 'jps3d_fillet', 'jps3d_guided')
GUIDE_SPACING = 3.0  # [turn radii] between guide waypoints


def plan(algorithm, goals, radius, world, jps_leg=None, rrt_time=1.0):
    """Closed tour through goal poses -> (start pose, segments).

    jps_leg(a, b) -> [(x, y), ...] corner points from a to b (the jps3d /plan service). Grid planners do not
    take headings, so jps3d tours ignore the goal headings.
    """
    if algorithm == 'hybrid_astar':
        return goals[0], plan_tour(goals, lambda a, b: hybrid_astar.plan(a, b, radius, world))
    if algorithm == 'rrt_star':
        return goals[0], plan_tour(goals, lambda a, b: rrt_star.plan(a, b, radius, world, rrt_time))
    if algorithm not in ALGORITHMS:
        raise ValueError(f'unknown algorithm {algorithm}, use one of {ALGORITHMS}')

    points, goal_indices = [], []
    for a, b in zip(goals, goals[1:] + goals[:1]):
        goal_indices.append(len(points))
        points += jps_leg(a, b)[:-1]  # the next leg starts where this one ends
    if algorithm == 'jps3d_raw':
        return raw_segments(points)
    if algorithm == 'jps3d_fillet':
        return fillet_segments(points, radius, world, goal_indices)
    return goals[0], guided_tour(points, goal_indices, goals, radius, world)


def guided_tour(points, goal_indices, goals, radius, world):
    """jps3d_guided: the grid route as waypoints for Hybrid A* (flyable and collision-free, or it fails).

    Keeps route corners at least GUIDE_SPACING turn radii apart, headed along the route, and plans Hybrid A*
    legs between them. A waypoint Hybrid A* cannot reach is dropped and the next one is tried.
    """
    closed = points + points[:1]
    segments = []
    for g, (start, end) in enumerate(zip(goal_indices, goal_indices[1:] + [len(points)])):
        waypoints = [goals[g]]
        for i in range(start + 1, end):
            x, y = closed[i]
            nx, ny = closed[i + 1]
            if math.hypot(x - waypoints[-1][0], y - waypoints[-1][1]) >= GUIDE_SPACING * radius \
                    and math.hypot(goals[(g + 1) % len(goals)][0] - x, goals[(g + 1) % len(goals)][1] - y) >= GUIDE_SPACING * radius:
                waypoints.append((x, y, math.atan2(ny - y, nx - x)))
        waypoints.append(goals[(g + 1) % len(goals)])
        current, i = waypoints[0], 1
        while i < len(waypoints):
            try:
                segments += hybrid_astar.plan(current, waypoints[i], radius, world, max_expansions=5000)
                current = waypoints[i]
            except RuntimeError:
                if i == len(waypoints) - 1:
                    raise
            i += 1
    return segments
