import math

import pytest

from fw_planner import rrt_star
from fw_planner.dubins import end_pose, sample, shortest_path
from fw_planner.world import World

R = 40.0
rrt_star.seed(1)


def path_length(segments):
    return sum(length for _, length in segments)


def assert_pose_close(a, b, tol=1e-6):
    assert a[0] == pytest.approx(b[0], abs=tol)
    assert a[1] == pytest.approx(b[1], abs=tol)
    assert math.cos(a[2] - b[2]) == pytest.approx(1.0, abs=tol)


def test_free_space_gives_the_dubins_path():
    start, goal = (0.0, 0.0, 0.0), (300.0, 200.0, math.pi / 2)
    segments = rrt_star.plan(start, goal, R, World(), time_limit=0.2)
    assert_pose_close(end_pose(start, segments), goal)
    assert path_length(segments) == pytest.approx(path_length(shortest_path(start, goal, R)), rel=1e-3)


def test_goes_around_an_obstacle_within_the_turn_radius():
    start, goal = (0.0, 0.0, 0.0), (500.0, 0.0, 0.0)
    obstacles = [(250.0, 0.0, 60.0)]
    segments = rrt_star.plan(start, goal, R, World(obstacles, (-200.0, 700.0, -300.0, 300.0), 10.0),
                             time_limit=0.5)
    assert_pose_close(end_pose(start, segments), goal)
    clearance = min(math.hypot(x - 250.0, y) - 60.0 for x, y, _, _ in sample(start, segments, 1.0))
    assert clearance >= 10.0 - 1.0  # 1 m: check spacing along the path
    assert all(abs(k) <= 1 / R + 1e-9 for k, _ in segments)


@pytest.mark.parametrize('start_heading, goal_heading', [(0.0, 3 * math.pi / 2), (math.pi, 0.0)])
def test_accepts_any_heading(start_heading, goal_heading):
    # OMPL takes yaw in [-pi, pi) only: pi itself as a start heading used to fail every time.
    start, goal = (0.0, 0.0, start_heading), (0.0, 300.0, goal_heading)
    segments = rrt_star.plan(start, goal, R, World(), time_limit=0.2)
    assert_pose_close(end_pose(start, segments), goal)


def test_blocked_goal_raises():
    with pytest.raises(RuntimeError):
        rrt_star.plan((0.0, 0.0, 0.0), (300.0, 0.0, 0.0), R, World([(300.0, 0.0, 30.0)]), time_limit=0.1)
