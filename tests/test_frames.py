"""pac_common.frames: planner corner pose <-> box centre (common standard 10.1)."""
import math

import pytest
from pac_common import Pose3D, Size3D
from pac_common.frames import center_to_corner, corner_to_center

SIZE = Size3D(0.4, 0.3, 0.2)


def test_corner_to_center_and_back():
    corner = Pose3D("pallet", 0.2, 0.3, 0.4, yaw=0.0)
    center = corner_to_center(SIZE, corner)
    assert (center.x, center.y, center.z) == pytest.approx((0.4, 0.45, 0.5))
    back = center_to_corner(SIZE, center)
    assert (back.x, back.y, back.z, back.yaw) == pytest.approx((corner.x, corner.y, corner.z, corner.yaw))


def test_yaw_swaps_the_footprint():
    center = corner_to_center(SIZE, Pose3D("pallet", 0.2, 0.3, 0.4, yaw=math.pi / 2))
    assert (center.x, center.y, center.z) == pytest.approx((0.35, 0.5, 0.5))


def test_tilted_boxes_are_rejected():
    with pytest.raises(ValueError):
        corner_to_center(SIZE, Pose3D("pallet", 0.2, 0.3, 0.4, roll=0.4))
