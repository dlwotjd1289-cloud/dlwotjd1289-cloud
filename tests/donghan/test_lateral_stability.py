"""Hard-mask step 13: lateral stability (pac_candidates/lateral.py, lateral_sim.py).

Hand-checked cases (CoG uncertainty 5 % of the side, size tolerance 2 mm):
- 0.3 x 0.3 x 0.6 box on the floor: tips at (0.15 - 0.015 - 0.002) / 0.3 = 0.443 g,
  slides at mu = 0.4 -> LP 0.40 g
- four stacked 0.3 m cubes: (0.15 - 0.015 - 0.002) / 0.6 = 0.222 g < 0.25 g
"""
from dataclasses import replace
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "taehyeon"))
from th_helpers import candidate, make_box, make_context, make_state, placed  # noqa: E402

from pac_candidates import CandidateBackend, CandidateConfig, load_candidate_config  # noqa: E402
from pac_candidates.config import LateralConfig  # noqa: E402
from pac_candidates.geometry import footprint  # noqa: E402
from pac_candidates.pallet_model import PalletModel  # noqa: E402
from pac_common import Pose3D  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
COLUMN = [placed(f"C{k}", 0.4, 0.4, 0.3 * k, size=(0.3, 0.3, 0.3), weight=10) for k in range(3)]


def lateral_config(**kw):
    base = CandidateConfig()
    return replace(base, constraints=replace(base.constraints, lateral=LateralConfig(enabled=True, **kw)))


def assess(boxes, box, x, y, z, **kw):
    from pac_candidates import lateral as L

    model = PalletModel(make_state(boxes), make_context(), lateral_config(**kw))
    return L.assess(model, box, Pose3D("pallet", x, y, z), footprint(x, y, box.size.x, box.size.y),
                    box.size.z, model.config.uncertainty.size_tolerance_m, False, lp_all=True)


def test_config_default_off_and_donghan_yaml_on():
    assert CandidateConfig().constraints.lateral.enabled is False
    cfg = load_candidate_config(REPO / "config/donghan/candidates_lateral.yaml").constraints.lateral
    assert cfg.enabled and cfg.accel_g == 0.25 and cfg.mu_box_box == 0.4 and cfg.mu_box_pallet == 0.4
    with pytest.raises(ValueError):
        LateralConfig(band_low_g=0.3, accel_g=0.25)


def test_deck_boards_match_the_bullet_simulator():
    import yaml
    from pac_candidates.lateral import deck_rects
    from pac_common import Size3D

    sim = yaml.safe_load((REPO / "config/ahead_simulator.yaml").read_text(encoding="utf-8"))["pallet"]
    for path in (None, REPO / "config/donghan/candidates_lateral.yaml"):
        cfg = LateralConfig() if path is None else load_candidate_config(path).constraints.lateral
        assert cfg.deck == "slatted"
        assert cfg.deck_board_count == sim["top_board_count"]
        assert cfg.deck_board_width_ratio == sim["top_board_width_ratio"]
        boards = deck_rects(cfg, Size3D(1.1, 1.1, 1.5))
        gaps = {round(b.y0 - a.y1, 5) for a, b in zip(boards, boards[1:])}
        assert len(boards) == 5 and gaps == {0.09625}                        # 143 mm boards, 96 mm gaps


def test_disabled_step_leaves_hard_mask_unchanged():
    backend = CandidateBackend(make_context(), CandidateConfig())
    top = make_box("N", (0.3, 0.3, 0.3), 10)
    state = make_state(COLUMN)
    verdict = backend.validate_constraints(top, candidate(top, 0.4, 0.4, 0.9), state)
    assert verdict.success
    assert "lateral_a_max_g" not in verdict.details["metrics"]


def test_lp_single_tall_box_is_friction_limited():
    pytest.importorskip("scipy")
    a = assess([], make_box("N", (0.3, 0.3, 0.6), 10), 0.4, 0.4, 0.0, deck="solid")
    assert min(a.a_geom) == pytest.approx(0.4433, abs=1e-3)
    assert min(a.a_lp) == pytest.approx(0.40, abs=2e-3)


def test_lp_four_high_column_below_quarter_g():
    pytest.importorskip("scipy")
    a = assess(COLUMN, make_box("N", (0.3, 0.3, 0.3), 10), 0.4, 0.4, 0.9, deck="solid")
    assert min(a.a_lp) == pytest.approx(0.2217, abs=2e-3)
    assert min(a.a_geom) == pytest.approx(0.2217, abs=2e-3)


def test_lp_side_support_from_neighbours():
    pytest.importorskip("scipy")
    walls = [placed("W1", 0.0, 0.3, 0.0, size=(0.395, 0.6, 0.6), weight=40),
             placed("W2", 0.555, 0.3, 0.0, size=(0.5, 0.6, 0.6), weight=40)]
    slender = make_box("N", (0.15, 0.6, 0.6), 5)
    alone = assess([], slender, 0.4, 0.3, 0.0, deck="solid")
    supported = assess(walls, slender, 0.4, 0.3, 0.0, deck="solid")
    assert min(alone.a_lp) == pytest.approx(0.218, abs=2e-3)
    assert min(supported.a_lp) > 0.35          # leans on the blocks 5 mm away
    assert min(supported.a_geom) == pytest.approx(0.218, abs=2e-3)   # geom ignores neighbours


def test_slatted_deck_box_over_gap():
    pytest.importorskip("scipy")
    box = make_box("N", (0.2, 0.2, 0.2), 5)
    assert min(assess([], box, 0.4, 0.15, 0.0, deck="solid").a_lp) > 0.3
    slatted = assess([], box, 0.4, 0.15, 0.0, deck="slatted")                      # 96 mm gaps
    assert min(slatted.a_lp) == 0.0 and "static_unstable" in slatted.lp_status
    assert min(assess([], box, 0.4, 0.15, 0.0, deck="slatted", deck_board_count=7).a_lp) > 0.3   # 16.5 mm gaps


def test_hard_mask_rejects_column_top_and_keeps_floor_box():
    pytest.importorskip("scipy")
    backend = CandidateBackend(make_context(), lateral_config(deck="solid", borderline="reject"))
    state = make_state(COLUMN)
    top = make_box("N", (0.3, 0.3, 0.3), 10)
    verdict = backend.validate_constraints(top, candidate(top, 0.4, 0.4, 0.9), state)
    assert not verdict.success
    assert any(r.startswith("LATERAL_ACCEL") for r in verdict.details["reasons"])
    floor = make_box("F", (0.4, 0.3, 0.2), 5)
    ok = backend.validate_constraints(floor, candidate(floor, 0.05, 0.05, 0.0), state)
    assert ok.success and ok.details["metrics"]["lateral_stage"] == "geom"


def test_geom_stage_equals_jaesung_stack_check_without_margins():
    """Stage 1 is Jaesung's stack_lateral_ok (tools/prototypes/lookahead) plus the
    team uncertainty margins, diagonals and the deck model: with those switched
    off both give the same verdicts (the 0.3 g column check of the unmerged
    branch claude/pensive-hypatia-9d5i3j is the same port)."""
    sys.path.insert(0, str(REPO / "tools/prototypes/lookahead"))
    import lookahead as LA
    from pac_candidates import lateral as L
    from pac_candidates.config import UncertaintyConfig

    base = CandidateConfig()
    cfg = replace(base,
                  uncertainty=UncertaintyConfig(size_tolerance_m=0.0, cog_uncertainty_ratio=0.0,
                                                cog_uncertainty_min_m=0.0, height_tolerance_m=0.004),
                  constraints=replace(base.constraints, lateral=LateralConfig(enabled=True, deck="solid")))
    layouts = [
        (COLUMN, ("N", 0.4, 0.4, 0.9, (0.3, 0.3, 0.3), 10)),                                  # a_c 0.25
        ([placed("A", 0.2, 0.2, 0.0, size=(0.4, 0.3, 0.25), weight=12)],
         ("N", 0.33, 0.25, 0.25, (0.3, 0.2, 0.35), 6)),                                      # offset on top
        ([placed("A", 0.1, 0.1, 0.0, size=(0.3, 0.3, 0.4), weight=8),
          placed("B", 0.42, 0.1, 0.0, size=(0.3, 0.3, 0.4), weight=8)],
         ("N", 0.25, 0.12, 0.4, (0.35, 0.25, 0.3), 9)),                                      # bridge
        ([placed("A", 0.5, 0.5, 0.0, size=(0.5, 0.4, 0.3), weight=20),
          placed("B", 0.55, 0.55, 0.3, size=(0.3, 0.25, 0.4), weight=7)],
         ("N", 0.6, 0.6, 0.7, (0.25, 0.2, 0.3), 5)),                                         # tower
    ]
    for boxes, (bid, x, y, z, size, kg) in layouts:
        model = PalletModel(make_state(boxes), make_context(), cfg)
        box = make_box(bid, size, kg)
        scene = L.Scene(model, box, Pose3D("pallet", x, y, z), footprint(x, y, size[0], size[1]),
                        size[2], 0.0, False)
        a_geom = min(L.geom_accel(scene, u) for u in L.directions(4))
        proto = [(b.pose.x, b.pose.y, b.pose.z, b.size.x, b.size.y, b.size.z, b.weight_kg) for b in boxes]
        new = (x, y, z, *size, kg)
        for a in (0.1, 0.2, 0.24, 0.26, 0.3, 0.4, 0.6):
            assert (a_geom > a) == LA.stack_lateral_ok(proto, new, a), (bid, boxes, a, a_geom)


def test_short_tilt_test_matches_hand_cases():
    pytest.importorskip("scipy")
    pytest.importorskip("pybullet")
    from pac_candidates import lateral as L
    from pac_candidates import lateral_sim as S

    cfg = lateral_config(deck="solid")
    for boxes, box, pos, expected in ((COLUMN, make_box("N", (0.3, 0.3, 0.3), 10), (0.4, 0.4, 0.9), False),
                                      ([], make_box("N", (0.3, 0.3, 0.6), 10), (0.4, 0.4, 0.0), True)):
        model = PalletModel(make_state(boxes), make_context(), cfg)
        asm = L.assess(model, box, Pose3D("pallet", *pos), footprint(pos[0], pos[1], box.size.x, box.size.y),
                       box.size.z, 0.002, False, lp_all=True)
        res = S.short_tilt_test(asm.scene, asm.system, L.worst_directions(asm, 2), cfg.constraints.lateral)
        assert res["passed"] is expected, res
