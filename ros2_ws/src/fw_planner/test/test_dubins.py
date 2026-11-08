import math
import random

import pytest

from fw_planner.dubins import all_paths, end_pose, sample, shortest_path

R = 40.0


def assert_pose_close(a, b, tol=1e-6):
    assert a[0] == pytest.approx(b[0], abs=tol)
    assert a[1] == pytest.approx(b[1], abs=tol)
    assert math.cos(a[2] - b[2]) == pytest.approx(1.0, abs=tol)


def random_pose(rng):
    return (rng.uniform(-300, 300), rng.uniform(-300, 300), rng.uniform(-math.pi, math.pi))


def test_every_word_reaches_the_goal():
    rng = random.Random(0)
    for _ in range(500):
        start, goal = random_pose(rng), random_pose(rng)
        for segments in all_paths(start, goal, R):
            assert_pose_close(end_pose(start, segments), goal)
            assert all(length >= 0 for _, length in segments)
            assert all(abs(k) in (0.0, 1 / R) for k, _ in segments)


def test_shortest_is_shortest_and_at_least_straight_distance():
    rng = random.Random(1)
    for _ in range(200):
        start, goal = random_pose(rng), random_pose(rng)
        best = sum(length for _, length in shortest_path(start, goal, R))
        lengths = [sum(length for _, length in p) for p in all_paths(start, goal, R)]
        assert best == pytest.approx(min(lengths))
        assert best >= math.hypot(goal[0] - start[0], goal[1] - start[1]) - 1e-9


def test_straight_ahead_is_a_straight_line():
    segments = shortest_path((0.0, 0.0, 0.0), (500.0, 0.0, 0.0), R)
    assert sum(length for _, length in segments) == pytest.approx(500.0)


def test_positive_curvature_turns_right_in_ned():
    # Heading north (psi=0), a right turn moves east (+y) and increases heading.
    x, y, psi = end_pose((0.0, 0.0, 0.0), [(1 / R, R * math.pi / 2)])
    assert (x, y) == (pytest.approx(R), pytest.approx(R))
    assert psi == pytest.approx(math.pi / 2)


def test_sample_spacing_and_endpoints():
    start, goal = (0.0, 0.0, 0.0), (0.0, 200.0, math.pi)
    segments = shortest_path(start, goal, R)
    pts = sample(start, segments, 5.0)
    assert_pose_close(pts[0][:3], start)
    assert_pose_close(pts[-1][:3], goal)
    gaps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
    assert max(gaps) <= 5.0 + 1e-9
