import math

import pytest

from fw_planner.dubins import end_pose
from fw_planner.polyline import fillet_segments, raw_segments
from fw_planner.world import World

R = 40.0
SQUARE = [(0.0, 0.0), (400.0, 0.0), (400.0, 400.0), (0.0, 400.0)]


def path_length(segments):
    return sum(length for _, length in segments)


def assert_pose_close(a, b, tol=1e-6):
    assert a[0] == pytest.approx(b[0], abs=tol)
    assert a[1] == pytest.approx(b[1], abs=tol)
    assert math.cos(a[2] - b[2]) == pytest.approx(1.0, abs=tol)


def test_raw_is_the_closed_polyline_with_point_turns():
    start, segments = raw_segments(SQUARE)
    assert_pose_close(end_pose(start, segments), start)
    assert path_length(segments) == pytest.approx(1600.0, abs=0.01)
    assert max(abs(k) for k, _ in segments) > 1 / R  # corners a plane cannot fly


def test_raw_skips_collinear_points():
    _, segments = raw_segments([(0.0, 0.0), (200.0, 0.0), (400.0, 0.0), (400.0, 400.0)])
    assert sum(1 for k, _ in segments if k) == 3  # three corners, none at the collinear point


def test_fillet_closes_within_the_turn_radius_and_cuts_corners():
    start, segments = fillet_segments(SQUARE, R, World(), stops=[0, 2])
    assert_pose_close(end_pose(start, segments), start)
    assert all(abs(k) <= 1 / R + 1e-9 for k, _ in segments)
    assert 1200.0 < path_length(segments) < 1600.0  # cuts the corners, but goes all the way round
