"""One entry point for every tour planner, so the node and the benchmark run the same code.

- hybrid_astar: Hybrid A* with Dubins shots (fw_planner.hybrid_astar)
- rrt_star: RRT* in Dubins space (OMPL)
- jps3d_raw: cf_drone's jps3d grid path, flown as is (point turns)
- jps3d_fillet: the same grid path with Dubins fillets at the minimum turn radius
"""
from . import hybrid_astar, rrt_star
from .polyline import fillet_segments, raw_segments
from .world import plan_tour

ALGORITHMS = ('hybrid_astar', 'rrt_star', 'jps3d_raw', 'jps3d_fillet')


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
    return fillet_segments(points, radius, world, goal_indices)
