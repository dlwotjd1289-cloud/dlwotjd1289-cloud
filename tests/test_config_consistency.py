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
