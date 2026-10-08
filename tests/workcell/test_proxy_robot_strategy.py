from pac_common.models import InventoryState, PalletState, PlacementCandidate, Pose3D, Size3D, SystemState
from pac_robot.hdr50_22_sim_adapter import Hdr50_22SimAdapter


def test_proxy_maps_candidate_to_xyzyaw_only():
    candidate = PlacementCandidate(
        candidate_id="S0001-B001-C000",
        box_id="B001",
        target_pose=Pose3D("pallet", 0.2, 0.3, 0.4, roll=0.4, pitch=-0.2, yaw=1.57),
        base_state_version=1,
    )
    command = Hdr50_22SimAdapter().command_from_candidate(candidate)
    assert (command.x, command.y, command.z, command.yaw) == (0.2, 0.3, 0.4, 1.57)
    assert not hasattr(command, "roll")
    assert not hasattr(command, "pitch")
