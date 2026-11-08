"""Shortest Dubins paths (forward only, bounded curvature) in the PX4 local NED frame.

Pose: (x north [m], y east [m], psi [rad] heading from north toward east).
A path is a list of segments (curvature [1/m], length [m]). Curvature > 0 turns right (psi increases).
"""
import math

TWO_PI = 2.0 * math.pi


def _mod2pi(a):
    return a % TWO_PI


# Solvers for the six Dubins words (Shkel & Lumelsky 2001). Inputs are normalized: a, b are the
# start/goal headings relative to the start->goal line, d is the distance in turn radii.
# Each returns normalized segment lengths (t, p, q), or None if the word is not feasible.
# "L" means psi increasing, which is a right turn in NED (curvature +1/R).

def _lsl(a, b, d):
    sa, sb, ca, cb = math.sin(a), math.sin(b), math.cos(a), math.cos(b)
    p_sq = 2 + d * d - 2 * math.cos(a - b) + 2 * d * (sa - sb)
    if p_sq < 0:
        return None
    tmp = math.atan2(cb - ca, d + sa - sb)
    return _mod2pi(tmp - a), math.sqrt(p_sq), _mod2pi(b - tmp)


def _rsr(a, b, d):
    sa, sb, ca, cb = math.sin(a), math.sin(b), math.cos(a), math.cos(b)
    p_sq = 2 + d * d - 2 * math.cos(a - b) + 2 * d * (sb - sa)
    if p_sq < 0:
        return None
    tmp = math.atan2(ca - cb, d - sa + sb)
    return _mod2pi(a - tmp), math.sqrt(p_sq), _mod2pi(tmp - b)


def _lsr(a, b, d):
    sa, sb, ca, cb = math.sin(a), math.sin(b), math.cos(a), math.cos(b)
    p_sq = -2 + d * d + 2 * math.cos(a - b) + 2 * d * (sa + sb)
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
    return _mod2pi(tmp - a), p, _mod2pi(tmp - b)


def _rsl(a, b, d):
    sa, sb, ca, cb = math.sin(a), math.sin(b), math.cos(a), math.cos(b)
    p_sq = -2 + d * d + 2 * math.cos(a - b) - 2 * d * (sa + sb)
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
    return _mod2pi(a - tmp), p, _mod2pi(b - tmp)


def _rlr(a, b, d):
    sa, sb, ca, cb = math.sin(a), math.sin(b), math.cos(a), math.cos(b)
    tmp = (6.0 - d * d + 2 * math.cos(a - b) + 2 * d * (sa - sb)) / 8.0
    if abs(tmp) > 1:
        return None
    p = _mod2pi(TWO_PI - math.acos(tmp))
    t = _mod2pi(a - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
    return t, p, _mod2pi(a - b - t + p)


def _lrl(a, b, d):
    sa, sb, ca, cb = math.sin(a), math.sin(b), math.cos(a), math.cos(b)
    tmp = (6.0 - d * d + 2 * math.cos(a - b) + 2 * d * (sb - sa)) / 8.0
    if abs(tmp) > 1:
        return None
    p = _mod2pi(TWO_PI - math.acos(tmp))
    t = _mod2pi(-a - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
    return t, p, _mod2pi(b - a - t + p)


# Word -> (solver, turn direction of each segment: +1 right, -1 left, 0 straight).
_WORDS = [
    (_lsl, (1, 0, 1)),
    (_rsr, (-1, 0, -1)),
    (_lsr, (1, 0, -1)),
    (_rsl, (-1, 0, 1)),
    (_rlr, (-1, 1, -1)),
    (_lrl, (1, -1, 1)),
]


def all_paths(start, goal, radius):
    """All feasible Dubins paths from start to goal pose, as segment lists."""
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    d = math.hypot(dx, dy) / radius
    theta = math.atan2(dy, dx) if d > 0 else 0.0
    a, b = _mod2pi(start[2] - theta), _mod2pi(goal[2] - theta)
    paths = []
    for solver, dirs in _WORDS:
        lengths = solver(a, b, d)
        if lengths is not None:
            paths.append([(s / radius, n * radius) for s, n in zip(dirs, lengths)])
    return paths


def shortest_path(start, goal, radius):
    """Shortest Dubins path from start to goal pose."""
    return min(all_paths(start, goal, radius), key=lambda p: sum(length for _, length in p))


def integrate(pose, curvature, length):
    """Pose after flying one segment from pose."""
    x, y, psi = pose
    if curvature == 0.0:
        return x + length * math.cos(psi), y + length * math.sin(psi), psi
    psi1 = psi + curvature * length
    return (x + (math.sin(psi1) - math.sin(psi)) / curvature,
            y - (math.cos(psi1) - math.cos(psi)) / curvature,
            psi1)


def end_pose(start, segments):
    pose = start
    for curvature, length in segments:
        pose = integrate(pose, curvature, length)
    return pose


def sample(start, segments, step):
    """Points (x, y, psi, curvature) along the path, at most `step` apart, including both ends."""
    pts = [(*start, segments[0][0] if segments else 0.0)]
    pose = start
    for curvature, length in segments:
        n = max(1, math.ceil(length / step))
        for i in range(1, n + 1):
            pts.append((*integrate(pose, curvature, length * i / n), curvature))
        pose = integrate(pose, curvature, length)
    return pts
