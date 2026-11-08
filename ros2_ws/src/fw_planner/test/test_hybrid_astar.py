import math

import pytest

from fw_planner.dubins import end_pose, sample, shortest_path
from fw_planner.hybrid_astar import World, plan, plan_tour

R = 40.0


def assert_pose_close(a, b, tol=1e-6):
    assert a[0] == pytest.approx(b[0], abs=tol)
    assert a[1] == pytest.approx(b[1], abs=tol)
    assert math.cos(a[2] - b[2]) == pytest.approx(1.0, abs=tol)


def min_clearance(start, segments, obstacles):
    return min(math.hypot(x - ox, y - oy) - r
               for x, y, _, _ in sample(start, segments, 1.0) for ox, oy, r in obstacles)


def test_free_space_gives_the_dubins_path():
    start, goal = (0.0, 0.0, 0.0), (300.0, 200.0, math.pi / 2)
    segments = plan(start, goal, R, World())
    assert_pose_close(end_pose(start, segments), goal)
    assert sum(length for _, length in segments) == pytest.approx(
        sum(length for _, length in shortest_path(start, goal, R)))


def test_goes_around_an_obstacle_on_the_straight_line():
    start, goal = (0.0, 0.0, 0.0), (500.0, 0.0, 0.0)
    obstacles = [(250.0, 0.0, 60.0)]
    segments = plan(start, goal, R, World(obstacles, margin=10.0))
    assert_pose_close(end_pose(start, segments), goal)
    assert min_clearance(start, segments, obstacles) >= 10.0 - 1.0  # 1 m: check spacing along the path
    assert all(abs(k) <= 1 / R + 1e-12 for k, _ in segments)


def test_stays_inside_bounds():
    start, goal = (0.0, 0.0, 0.0), (0.0, 0.0, math.pi)  # turn around in place
    bounds = (-150.0, 150.0, -150.0, 150.0)
    segments = plan(start, goal, R, World(bounds=bounds))
    for x, y, _, _ in sample(start, segments, 1.0):
        assert bounds[0] <= x <= bounds[1] and bounds[2] <= y <= bounds[3]


def test_tour_is_closed():
    goals = [(0.0, 0.0, 0.0), (300.0, 300.0, math.pi / 2), (0.0, 300.0, math.pi)]
    segments = plan_tour(goals, R, World([(150.0, 150.0, 40.0)], margin=10.0))
    assert_pose_close(end_pose(goals[0], segments), goals[0])


def test_goal_inside_obstacle_raises():
    with pytest.raises(RuntimeError):
        plan((0.0, 0.0, 0.0), (300.0, 0.0, 0.0), R, World([(300.0, 0.0, 30.0)]))
