"""Turn a closed polyline from a grid planner (e.g. jps3d) into fixed-wing segments.

A grid planner returns corner points only: no headings, no turn radius. Two ways to fly it:
- raw: straight legs with point turns at the corners. This is what the grid planner asks for.
- fillet: Dubins paths at the minimum turn radius through the corners, heading along the corner bisector,
  with the farthest free Dubins jump taken where possible. Can still leave the free space: counted, not hidden.
Poses and segments follow fw_planner.dubins (PX4 local NED).
"""
import math

from .world import connect_poses

POINT_TURN_LENGTH = 1e-3  # [m] a corner becomes an arc this short (curvature = turn angle / length)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _leg_headings(points):
    closed = points + points[:1]
    return [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(closed, closed[1:])]


def raw_segments(points):
    """Closed polyline -> (start pose, segments). Starts at points[0] along the first leg."""
    headings = _leg_headings(points)
    closed = points + points[:1]
    segments = []
    for i, (a, b) in enumerate(zip(closed, closed[1:])):
        segments.append((0.0, math.hypot(b[0] - a[0], b[1] - a[1])))
        turn = _wrap(headings[(i + 1) % len(headings)] - headings[i])
        if abs(turn) > 1e-9:
            segments.append((turn / POINT_TURN_LENGTH, POINT_TURN_LENGTH))
    return (points[0][0], points[0][1], headings[0]), segments


def fillet_segments(points, radius, world, stops):
    """Closed polyline -> (start pose, segments) at the minimum turn radius. May be in collision: check it.

    stops: polyline indices the path must pass (the goals, at least two). Dubins jumps never skip them; on a
    closed loop, the farthest pose from the start would otherwise be the start itself.
    """
    headings = _leg_headings(points)
    poses = []
    for i, (x, y) in enumerate(points):
        incoming, outgoing = headings[i - 1], headings[i]
        poses.append((x, y, incoming + _wrap(outgoing - incoming) / 2))  # corner bisector
    closed = poses + poses[:1]
    ends = sorted(stops) + [len(points)]
    segments = []
    for a, b in zip(ends, ends[1:]):
        segments += connect_poses(closed[a:b + 1], radius, world)
    return closed[ends[0]], segments
