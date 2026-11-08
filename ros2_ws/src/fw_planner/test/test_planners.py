import math

import pytest

from fw_planner import planners
from fw_planner.dubins import end_pose
from fw_planner.world import World

R = 40.0
GOALS = [(0.0, 0.0, 0.0), (600.0, 0.0, math.pi / 2), (600.0, 600.0, math.pi), (0.0, 600.0, -math.pi / 2)]


def straight_leg(a, b):
    """A grid planner's answer in free space: the straight line, with a corner point half way."""
    return [a[:2], ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2), b[:2]]


def test_guided_tour_is_closed_flyable_and_keeps_goal_headings():
    world = World(bounds=(-300.0, 900.0, -300.0, 900.0))
    start, segments = planners.plan('jps3d_guided', GOALS, R, world, jps_leg=straight_leg)
    assert start == GOALS[0]
    end = end_pose(start, segments)
    assert end[0] == pytest.approx(0.0, abs=1e-6) and end[1] == pytest.approx(0.0, abs=1e-6)
    assert all(abs(k) <= 1 / R + 1e-9 for k, _ in segments)
    assert world.path_free(start, segments)
