"""RRT* in Dubins space (OMPL), the common research baseline for fixed-wing planning.

OMPL samples poses, joins them with Dubins paths (minimum turn radius) and rewires toward the shortest path
for a fixed time. Poses and segments follow fw_planner.dubins (PX4 local NED). OMPL's yaw in the (x, y) plane
is the same angle as psi, so no conversion is needed.
"""
import math

from ompl import base as ob, geometric as og, util as ou

from .dubins import shortest_path
from .world import CHECK_STEP

ou.setLogLevel(ou.LOG_WARN)


def seed(value):
    """Make runs repeatable. OMPL accepts this only before its first random number, so call it once, first."""
    ou.RNG.setSeed(value)


def plan(start, goal, radius, world, time_limit=1.0):
    """Segments from start to goal pose that stay free in world. Raises RuntimeError if none is found in time."""
    space = ob.DubinsStateSpace(radius)
    bounds = ob.RealVectorBounds(2)
    if world.bounds:
        x_min, x_max, y_min, y_max = world.bounds
    else:  # OMPL needs a sampling box: around start and goal, with room to turn
        pad = 4 * radius
        x_min, x_max = min(start[0], goal[0]) - pad, max(start[0], goal[0]) + pad
        y_min, y_max = min(start[1], goal[1]) - pad, max(start[1], goal[1]) + pad
    bounds.setLow(0, x_min)
    bounds.setHigh(0, x_max)
    bounds.setLow(1, y_min)
    bounds.setHigh(1, y_max)
    space.setBounds(bounds)

    setup = og.SimpleSetup(space)
    setup.setStateValidityChecker(lambda state: world.point_free(state.getX(), state.getY()))
    info = setup.getSpaceInformation()
    info.setStateValidityCheckingResolution(CHECK_STEP / space.getMaximumExtent())

    def state(pose):
        s = space.allocState()
        s.setX(pose[0])
        s.setY(pose[1])
        # OMPL drops a start state with yaw outside [-pi, pi) without an error, so wrap into it (pi -> -pi).
        s.setYaw((pose[2] + math.pi) % (2 * math.pi) - math.pi)
        return s

    setup.setStartAndGoalStates(state(start), state(goal), 1e-6)
    setup.setPlanner(og.RRTstar(info))
    if not setup.solve(time_limit) or not setup.haveExactSolutionPath():
        raise RuntimeError(f'RRT* found no path from {start} to {goal} in {time_limit} s')
    setup.simplifySolution()

    path = setup.getSolutionPath()
    poses = [(s.getX(), s.getY(), s.getYaw()) for s in (path.getState(i) for i in range(path.getStateCount()))]
    # OMPL joins consecutive states with the shortest Dubins path, the same one fw_planner.dubins finds.
    return [segment for a, b in zip(poses, poses[1:]) for segment in shortest_path(a, b, radius)]
