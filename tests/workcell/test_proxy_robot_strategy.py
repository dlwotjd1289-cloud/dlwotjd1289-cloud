import math

import pytest
from pac_common.models import BoxState, BoxStatus, PlacementCandidate, Pose3D, Size3D
from pac_robot.hdr50_22_sim_adapter import Hdr50_22Adapter

BOX = BoxState("B001", "SKU_A", Size3D(0.4, 0.3, 0.2), 12.0, Pose3D("conveyor", 0, 0, 0.9),
               (0.0, math.pi / 2), BoxStatus.READY_FOR_PICK, 1.0, 1.0, "fixture")


def candidate(yaw):
    return PlacementCandidate("S0001-B001-C000", "B001", Pose3D("pallet", 0.2, 0.3, 0.4, yaw=yaw), 1)


def test_command_is_box_centre_in_xyzyaw_only():
    command = Hdr50_22Adapter().command_from_candidate(BOX, candidate(0.0))
    assert (command.frame_id, command.x, command.y, command.z, command.yaw) == pytest.approx(
        ("pallet", 0.4, 0.45, 0.5, 0.0))
    assert not hasattr(command, "roll")
    assert not hasattr(command, "pitch")


def test_yaw_swaps_footprint_for_centre():
    command = Hdr50_22Adapter().command_from_candidate(BOX, candidate(math.pi / 2))
    assert (command.x, command.y, command.z) == pytest.approx((0.35, 0.5, 0.5))


def test_tilted_or_other_box_rejected():
    tilted = PlacementCandidate("S0001-B001-C001", "B001", Pose3D("pallet", 0.2, 0.3, 0.4, roll=0.4), 1)
    with pytest.raises(ValueError):
        Hdr50_22Adapter().command_from_candidate(BOX, tilted)
    other = PlacementCandidate("S0001-B002-C000", "B002", Pose3D("pallet", 0.2, 0.3, 0.4), 1)
    with pytest.raises(ValueError):
        Hdr50_22Adapter().command_from_candidate(BOX, other)
