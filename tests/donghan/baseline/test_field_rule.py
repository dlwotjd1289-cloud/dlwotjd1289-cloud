"""Comparison baseline, field-practice rules (donghan): stage-4 rules, stage-5-3 rules, world and runtime hookup."""

from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from pac_candidates import CandidateConfig
from pac_common import BoxState, BoxStatus, PalletState, PlacedBox, PlacementCandidate, Pose3D, Size3D, SkuSpec
from pac_highlevel import ActionType, Arrival, HighLevelConfig, HighLevelDecider, PalletizingWorld, run_policy
from pac_planning.baseline.field_rule import (
    FieldRuleConfig,
    FieldRulePlacer,
    FieldRulePolicy,
    load_field_rule_config,
    field_rule_config_from_dict,
    field_rule_highlevel_config,
    field_rule_loaded_policy,
    side_contact,
)

REPO = Path(__file__).resolve().parents[3]
HALF_PI = math.pi / 2
PALLET = Size3D(1.2, 0.8, 1.5)
BOX = Size3D(0.3, 0.2, 0.1)
SMALL = Size3D(0.62, 0.42, 0.25)   # three 0.2 x 0.4 boxes per layer, two layers


def cand(cid, x, y, z=0.0, yaw=0.0):
    return PlacementCandidate(cid, "B000", Pose3D("pallet", x, y, z, yaw=yaw), 0)


def placed(box_id, x, y, z=0.0, size=(0.3, 0.2, 0.1)):
    return PlacedBox(box_id, "K", Size3D(*size), 5.0, Pose3D("pallet", x, y, z))


def scene(*boxes, size=PALLET):
    return SimpleNamespace(pallet=PalletState("P", size, tuple(boxes)))


def pick(cands, state=None, config=None):
    box = SimpleNamespace(size=BOX)
    return FieldRulePlacer(config)(cands, box, state or scene()).candidate_id


# ---------------------------------------------------------------- stage 5-3
def test_lowest_level_first():
    assert pick([cand("high-far-left", 0.0, 0.6, z=0.1), cand("floor", 0.9, 0.0)]) == "floor"


def test_tops_within_the_height_tolerance_are_one_level_then_far_row_wins():
    # robot on -y: the row at the +y edge is filled first
    assert pick([cand("near", 0.0, 0.0), cand("far", 0.5, 0.6, z=0.002)]) == "far"
    assert pick([cand("near", 0.0, 0.0), cand("far", 0.5, 0.6, z=0.01)]) == "near"


def test_same_row_more_contact_then_left():
    # corner (far edge + left edge) beats the middle of the far row
    assert pick([cand("middle", 0.5, 0.6), cand("corner", 0.0, 0.6)]) == "corner"
    # equal contact -> left-most seen from the robot
    assert pick([cand("b", 0.6, 0.6), cand("a", 0.3, 0.6)]) == "a"


def test_contact_counts_neighbour_sides_but_not_boxes_below():
    neighbour = scene(placed("N", 0.0, 0.6))
    # 8 mm next to the neighbour: far edge (0.3) + neighbour side (0.2) = 0.5 of the perimeter
    assert side_contact(BOX, Pose3D("pallet", 0.308, 0.6, 0.0), neighbour.pallet, 0.015, 0.003) == pytest.approx(0.5)
    assert pick([cand("free", 0.6, 0.6), cand("beside", 0.308, 0.6)], neighbour) == "beside"
    below = scene(placed("U", 0.5, 0.3))
    assert side_contact(BOX, Pose3D("pallet", 0.5, 0.3, 0.1), below.pallet, 0.015, 0.003) == 0.0


def test_robot_side_sets_far_row_and_left():
    cands = [cand("small_x", 0.0, 0.3), cand("large_x", 0.9, 0.3)]
    assert pick(cands, config=FieldRuleConfig(robot_side="+x")) == "small_x"
    assert pick(cands, config=FieldRuleConfig(robot_side="-x")) == "large_x"
    # robot on +y faces -y: its left is +x
    row = [cand("a", 0.3, 0.0), cand("b", 0.6, 0.0)]
    assert pick(row, config=FieldRuleConfig(robot_side="+y")) == "b"


def test_rank_is_a_full_deterministic_order():
    cands = [cand(f"c{i}", 0.1 * i, 0.06 * i, z=0.05 * (i % 3)) for i in range(8)]
    box = SimpleNamespace(size=BOX)
    placer = FieldRulePlacer()
    ranked = placer.rank(cands, box, scene())
    assert sorted(c.candidate_id for c in ranked) == sorted(c.candidate_id for c in cands)
    assert ranked[0] is placer(cands, box, scene())
    assert [c.candidate_id for c in placer.rank(list(reversed(cands)), box, scene())] == \
        [c.candidate_id for c in ranked]
    assert [r["candidate_id"] for r in placer.explain(cands, box, scene())] == [c.candidate_id for c in ranked]
    assert placer([], box, scene()) is None and placer.rank([], box, scene()) == []


def test_config_file_and_validation():
    assert load_field_rule_config(REPO / "config/donghan/baseline/field_rule.yaml") == FieldRuleConfig()
    with pytest.raises(ValueError, match="robot_side"):
        FieldRuleConfig(robot_side="left")
    with pytest.raises(ValueError, match="unknown"):
        field_rule_config_from_dict({"robot": "-y"})
    with pytest.raises(ValueError, match="row_tol_m"):
        FieldRuleConfig(row_tol_m=-0.01)


def test_highlevel_settings_turn_off_repack_and_early_close():
    hl = field_rule_highlevel_config(HighLevelConfig())
    assert hl.repack.enabled is False
    assert hl.close.mode == "fill" and hl.close.fill_before_buffer == 0.0
    assert hl.buffer == HighLevelConfig().buffer and hl.timing == HighLevelConfig().timing


# ---------------------------------------------------------------- stage 4
def fake_world(mask, current=True, stored=()):
    buffer = [None if s is None else SimpleNamespace(stored_at=s) for s in stored]
    return SimpleNamespace(action_mask=lambda: np.array(mask), current=object() if current else None,
                           buffer=buffer)


def test_policy_place_now_first():
    a = FieldRulePolicy()(fake_world([True, True, True, True], stored=(1, 2)))
    assert a.type == ActionType.PLACE_CURRENT


def test_policy_retrieves_the_oldest_buffered_box_that_fits():
    a = FieldRulePolicy()(fake_world([False, True, True, True], stored=(5, 2)))
    assert a.type == ActionType.RETRIEVE_BUFFER and a.slot == 1
    a = FieldRulePolicy()(fake_world([False, True, True, False], stored=(5, 2)))
    assert a.slot == 0  # the older box does not fit
    a = FieldRulePolicy()(fake_world([False, False, False, True], current=False, stored=(None, 3)))
    assert a.type == ActionType.RETRIEVE_BUFFER and a.slot == 1


def test_policy_buffers_when_nothing_fits_and_never_picks_a_masked_action():
    a = FieldRulePolicy()(fake_world([False, True, False, False], stored=(None, None)))
    assert a.type == ActionType.BUFFER_CURRENT
    with pytest.raises(RuntimeError, match="empty mask"):
        FieldRulePolicy()(fake_world([False, False, False, False], stored=(1, 2)))


# ---------------------------------------------------------------- whole algorithm
def make_box(i, size=(0.2, 0.4, 0.1), weight=5.0, sku="K"):
    return BoxState(f"B{i:03d}", sku, Size3D(*size), weight, Pose3D("conveyor", 0.0, 0.0, 0.0),
                    (0.0, HALF_PI), BoxStatus.READY_FOR_PICK, 1.0, float(i), "test")


def rule_world(n_boxes, slots=2, placer=None):
    boxes = [make_box(i) for i in range(n_boxes)]
    hl = HighLevelConfig()
    hl = field_rule_highlevel_config(replace(hl, buffer=replace(hl.buffer, slots=slots)))
    catalog = {"K": SkuSpec("K", Size3D(0.2, 0.4, 0.1), 5.0, (0.0, HALF_PI), 1000.0)}
    return PalletizingWorld([Arrival(b) for b in boxes], SMALL, catalog, CandidateConfig(), hl,
                            placer=placer or FieldRulePlacer())


def test_episode_places_every_box_safely_and_flushes_the_buffer_on_a_new_pallet():
    # 6 boxes fill the small pallet; boxes 7 and 8 wait in the buffer, the stream
    # ends, nothing fits -> close, then the buffered boxes go on the new pallet
    out = run_policy(rule_world(8, slots=2), FieldRulePolicy())
    assert out["placed"] == 8 and out["ng"] == 0 and out["safety_issues"] == 0
    assert out["pallets_closed"] == 1 and out["pallets"][0]["boxes"] == 6
    assert out["counts"]["BUFFER_CURRENT"] == 2 and out["counts"]["RETRIEVE_BUFFER"] == 2
    assert "PARTIAL_REPACK" not in out["counts"]


def test_full_buffer_closes_the_pallet():
    out = run_policy(rule_world(9, slots=1), FieldRulePolicy())
    assert out["placed"] == 9 and out["safety_issues"] == 0
    assert out["counts"]["PALLET_CLOSE"] == 1 and out["counts"]["BUFFER_CURRENT"] == 1


def test_runtime_ranker_hookup_gives_the_same_episode():
    from pac_runtime.placer import RobotAwarePlacer

    class AcceptAll:  # stage 6 stand-in: every candidate executable
        def first_executable(self, box, ranked, state, robot_state=None):
            return (ranked[0], SimpleNamespace(success=True, codes=()), {}) if ranked else (None, None, {})

    plain = run_policy(rule_world(8), FieldRulePolicy())
    runtime = run_policy(rule_world(8, placer=RobotAwarePlacer(AcceptAll(), 12, FieldRulePlacer().rank)),
                         FieldRulePolicy())
    assert runtime["pallets"] == plain["pallets"] and runtime["counts"] == plain["counts"]


def test_high_level_decider_runs_the_rule_policy():
    w = rule_world(3)
    decider = HighLevelDecider(w.context(), CandidateConfig(), w.config, policy=field_rule_loaded_policy(),
                               placer=FieldRulePlacer())
    d = decider.decide(w.state(), current_box_id=w.current.box.box_id)
    assert d.action.type == ActionType.PLACE_CURRENT and d.decided_by == "policy:baseline_field_rule"
    assert d.candidate.target_pose.z == 0.0
