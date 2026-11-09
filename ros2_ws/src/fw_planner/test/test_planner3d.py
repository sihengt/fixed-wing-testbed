import math

import pytest

from fw_planner import planner3d
from fw_planner.planner3d import World3D, sample, shot

R = 40.0
CLIMB = math.radians(8)


def end_pose(start, segments):
    pose = start
    for segment in segments:
        pose = planner3d.integrate3d(pose, segment)
    return pose


def assert_climb_ok(segments):
    assert all(abs(dh) <= length * math.tan(CLIMB) + 1e-6 for _, length, dh in segments)


def test_shot_reaches_the_goal_within_the_climb_angle():
    start, goal = (0.0, 0.0, 60.0, 0.0), (400.0, 100.0, 90.0, math.pi / 2)
    segments = shot(start, goal, R, CLIMB)
    end = end_pose(start, segments)
    assert end[:3] == pytest.approx(goal[:3], abs=1e-6)
    assert_climb_ok(segments)


def test_shot_too_steep_is_rejected():
    assert shot((0.0, 0.0, 60.0, 0.0), (100.0, 0.0, 120.0, 0.0), R, CLIMB) is None


def test_flies_over_a_low_obstacle():
    start, goal = (0.0, 0.0, 60.0, 0.0), (800.0, 0.0, 60.0, 0.0)
    world = World3D([(400.0, 0.0, 80.0, 0.0, 55.0)], altitudes=(30.0, 150.0), margin=10.0)
    segments = planner3d.plan(start, goal, R, world, CLIMB)
    points = sample(start, segments, 1.0)
    assert end_pose(start, segments)[:3] == pytest.approx(goal[:3], abs=1e-6)
    assert world.path_free(start, segments)
    assert max(abs(y) for _, y, _, _ in points) < 50.0  # stayed on the line: over, not around
    assert max(h for _, _, h, _ in points) >= 65.0
    assert_climb_ok(segments)


def test_goes_around_a_tall_obstacle():
    start, goal = (0.0, 0.0, 60.0, 0.0), (800.0, 0.0, 60.0, 0.0)
    world = World3D([(400.0, 0.0, 60.0, 0.0, 300.0)], altitudes=(30.0, 150.0), margin=10.0)
    segments = planner3d.plan(start, goal, R, world, CLIMB)
    points = sample(start, segments, 1.0)
    assert world.path_free(start, segments)
    assert max(abs(y) for _, y, _, _ in points) > 70.0
    assert_climb_ok(segments)
