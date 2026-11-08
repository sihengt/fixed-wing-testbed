import math

import pytest

from fw_viz.viz_node import ENU_FROM_NED, FRD_TO_FLU, quaternion_multiply


def rotate(q, v):
    w, x, y, z = quaternion_multiply(quaternion_multiply(q, (0.0, *v)), (q[0], -q[1], -q[2], -q[3]))
    return (x, y, z)


@pytest.mark.parametrize('ned, enu', [((1, 0, 0), (0, 1, 0)),  # north
                                      ((0, 1, 0), (1, 0, 0)),  # east
                                      ((0, 0, 1), (0, 0, -1))])  # down
def test_ned_to_enu(ned, enu):
    assert rotate(ENU_FROM_NED, ned) == pytest.approx(enu)


def test_level_plane_heading_east_points_flu_axes_right():
    # PX4 attitude for level flight heading east: yaw 90 deg about NED down.
    px4_q = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))
    body_flu = quaternion_multiply(px4_q, FRD_TO_FLU)
    world = quaternion_multiply(ENU_FROM_NED, body_flu)  # FLU body -> ENU
    assert rotate(world, (1, 0, 0)) == pytest.approx((1, 0, 0))  # nose east
    assert rotate(world, (0, 1, 0)) == pytest.approx((0, 1, 0))  # left wing north
    assert rotate(world, (0, 0, 1)) == pytest.approx((0, 0, 1))  # up is up
