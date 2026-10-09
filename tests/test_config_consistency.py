"""config/default.yaml `pallet:` is the single source; everything else must agree.

Change the value there; if a geometry file (Gazebo SDF, workcell.yaml) cannot
read it, this test points at the file that still needs editing.
"""
import re
from pathlib import Path

import pytest
import yaml
from pac_common.config import load_common_config

ROOT = Path(__file__).resolve().parents[1]
PALLET = load_common_config(ROOT / "config" / "default.yaml").pallet
GAZEBO_WORLD = ROOT / "ros2_ws/src/pac_simulation/worlds/ahead_workcell_v4_2_physical_scale.sdf"
TOL_M = 0.0025  # Gazebo boards are modelled to the mm


def test_common_values_are_the_team_decision_shape():
    assert PALLET.cargo_size.z == PALLET.max_stack_height_m   # PalletState.size.z = above deck
    assert PALLET.max_total_height_m == pytest.approx(PALLET.deck_height_m + PALLET.max_stack_height_m)


def test_workcell_layout_pallet_matches():
    layout = yaml.safe_load((ROOT / "config/workcell.yaml").read_text(encoding="utf-8"))["layout"]
    x, y, z = layout["pallet"]["size_m"]
    assert (x, y, z) == pytest.approx((PALLET.size_x_m, PALLET.size_y_m, PALLET.deck_height_m))
    assert layout["pallet"]["center_world_m"][2] == pytest.approx(PALLET.deck_height_m / 2)


def test_bullet_simulator_reads_common_pallet():
    pytest.importorskip("pybullet")
    from pac_simulation.ahead_sim.config import load_config

    cfg, _ = load_config(ROOT / "config/ahead_simulator.yaml")
    p = cfg.pallet
    assert (p.length_m, p.width_m, p.deck_height_m, p.max_height_m) == pytest.approx(
        (PALLET.size_x_m, PALLET.size_y_m, PALLET.deck_height_m, PALLET.max_stack_height_m))


def test_generator_reads_common_pallet():
    from ahead_dataset_generator.config import load_yaml, resolve_pallet

    cfg = load_yaml(ROOT / "tools/ahead_dataset_generator/config/default.yaml")
    got = resolve_pallet(cfg)
    assert (got["size_x_m"], got["size_y_m"], got["deck_height_m"], got["max_stack_height_m"],
            got["max_load_kg"]) == pytest.approx((PALLET.size_x_m, PALLET.size_y_m, PALLET.deck_height_m,
                                                  PALLET.max_stack_height_m, PALLET.max_load_kg))


def test_gazebo_pallet_geometry_matches():
    """pallet_main top boards in the V4.2 world span the team footprint at the deck height."""
    sdf = GAZEBO_WORLD.read_text(encoding="utf-8")
    model = sdf[sdf.index('<model name="pallet_main">'):]
    model = model[:model.index("</model>")]
    boards = re.findall(r'<collision name="top_board_\d+_collision">\s*<pose>([^<]+)</pose>.*?<size>([^<]+)</size>',
                        model, re.S)
    assert boards, "top boards not found"
    xs, ys, tops = [], [], []
    for pose, size in boards:
        px, py, pz = map(float, pose.split()[:3])
        sx, sy, sz = map(float, size.split())
        xs += [px - sx / 2, px + sx / 2]
        ys += [py - sy / 2, py + sy / 2]
        tops.append(pz + sz / 2)
    assert max(xs) - min(xs) == pytest.approx(PALLET.size_x_m, abs=TOL_M)
    assert max(ys) - min(ys) == pytest.approx(PALLET.size_y_m, abs=TOL_M)
    assert max(tops) == pytest.approx(PALLET.deck_height_m, abs=TOL_M)


def _workcell():
    return yaml.safe_load((ROOT / "config/workcell.yaml").read_text(encoding="utf-8"))["layout"]


def test_robot_check_cell_matches_workcell_and_pedestal_urdf():
    """Stage 6 (config/taehyeon/robot_check.yaml) describes the same cell as workcell.yaml."""
    import math
    from pac_robot_check import load_robot_check_config

    layout = _workcell()
    urdf = (ROOT / "ros2_ws/src/pac_bringup/urdf/hdr50_22_pedestal.urdf.xacro").read_text(encoding="utf-8")
    joint = urdf[urdf.index('<joint name="world_joint"'):]
    pedestal_z = float(re.search(r'<origin xyz="[^ ]+ [^ ]+ ([^"]+)"', joint).group(1))
    base = layout["robot_base_world_m"]
    assert base[2] == pytest.approx(pedestal_z)
    cx, cy, cz = layout["pallet"]["center_world_m"]
    deck_top = cz + PALLET.deck_height_m / 2
    a = layout["pallet"]["frame_yaw_rad"]
    wx, wy = base[0] - cx, base[1] - cy                     # base relative to the deck centre, world axes
    cell = load_robot_check_config(ROOT / "config/taehyeon/robot_check.yaml").cell
    bx, by, bz, byaw = cell.base_from_pallet_center
    assert (bx, by) == pytest.approx((math.cos(a) * wx + math.sin(a) * wy, -math.sin(a) * wx + math.cos(a) * wy))
    assert bz == pytest.approx(base[2] - deck_top)
    assert math.remainder(byaw + a, 2 * math.pi) == pytest.approx(0.0, abs=1e-9)
    assert cell.deck_height_m == pytest.approx(PALLET.deck_height_m)
    pick = layout["pick_zone"]["center_world_m"]
    assert cell.pick_point_base_m == pytest.approx(tuple(p - b for p, b in zip(pick, base)))


def test_stage6_joints_put_the_tcp_on_the_gazebo_box_top():
    """pac_robot_check (pallet frame) and the Gazebo bridge (world) agree on where a box goes."""
    import sys

    sys.path.insert(0, str(ROOT / "scripts"))
    import hdr50_kinematics as K
    import mission_bridge_v45 as MB
    from pac_common import (BoxState, BoxStatus, InventoryState, PalletState, PlacementCandidate, Pose3D,
                            Size3D, SystemState)
    from pac_robot_check import RobotFeasibility, load_robot_check_config

    robot = RobotFeasibility(load_robot_check_config(ROOT / "config/taehyeon/robot_check.yaml"))
    size = Size3D(0.4, 0.3, 0.25)
    state = SystemState(1, 0.0, PalletState("P", PALLET.cargo_size, ()), InventoryState({}, {}))
    box = BoxState("B", "K", size, 8.0, Pose3D("conveyor", 0, 0, 0), (0.0,), BoxStatus.READY_FOR_PICK,
                   1.0, 0.0, "test")
    for corner in (Pose3D("pallet", 0.0, 0.0, 0.0), Pose3D("pallet", 0.6, 0.7, 0.5, yaw=1.5707963267948966)):
        v = robot.validate_robot_motion(box, PlacementCandidate("c", "B", corner, 1), state)
        assert v.success, v.details
        flange, R = K.fk(v.details["q_place"])
        tcp = flange + R[:, 0] * robot.cfg.gripper.tcp_offset_m          # flange +x = tool direction
        x, y, z, _ = MB.candidate_to_world(
            {"frame_id": "pallet", "x": corner.x, "y": corner.y, "z": corner.z, "yaw": corner.yaw},
            (size.x, size.y, size.z))
        assert tcp == pytest.approx((x, y, z + size.z / 2), abs=2e-3)
