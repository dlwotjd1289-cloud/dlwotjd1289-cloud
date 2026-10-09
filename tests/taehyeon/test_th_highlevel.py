"""Stage 4 high-level action selection (taehyeon): world, masks, rules."""

import math
from dataclasses import replace

import numpy as np
import pytest
from pac_candidates import CandidateConfig
from pac_common import PlacedBox, Pose3D, Size3D, SkuSpec
from pac_highlevel import (
    ActionType,
    Arrival,
    GreedyPolicy,
    HighLevelAction,
    HighLevelConfig,
    PalletizingWorld,
    RulePolicy,
    action_count,
    from_index,
    load_highlevel_config,
    run_policy,
    to_index,
)
from pac_highlevel.config import config_from_dict
from pac_highlevel.repack import accessible_ids, plan_repack
from th_helpers import HALF_PI, REPO, make_box

SMALL = Size3D(0.62, 0.42, 0.25)


def box(i, size=(0.2, 0.4, 0.1), weight=5.0, sku="K"):
    return replace(make_box(f"B{i:03d}", size, weight, sku=sku), stamp_sec=float(i))


def catalog(*specs):
    return {s: SkuSpec(s, Size3D(*size), w, (0.0, HALF_PI), 1000.0) for s, size, w in specs}


def world(boxes, pallet=SMALL, slots=2, **kw):
    cfg = HighLevelConfig()
    cfg = replace(cfg, buffer=replace(cfg.buffer, slots=slots), **kw)
    sizes = {b.sku_id: ((b.size.x, b.size.y, b.size.z), b.weight_kg) for b in boxes}
    cat = catalog(*[(s, size, w) for s, (size, w) in sizes.items()])
    return PalletizingWorld([Arrival(b) for b in boxes], pallet, cat, CandidateConfig(), cfg)


# ---------------------------------------------------------------- actions


def test_action_index_round_trip():
    slots = 3
    assert action_count(slots) == 5
    for i in range(action_count(slots)):
        assert to_index(from_index(i, slots)) == i
    with pytest.raises(ValueError):
        to_index(HighLevelAction(ActionType.PALLET_CLOSE))
    with pytest.raises(ValueError):
        HighLevelAction(ActionType.RETRIEVE_BUFFER)
    with pytest.raises(ValueError):
        from_index(5, slots)


def test_config_file_and_validation():
    cfg = load_highlevel_config(REPO / "config/taehyeon/highlevel.yaml")
    assert cfg == HighLevelConfig()  # yaml mirrors the code defaults
    with pytest.raises(ValueError):
        config_from_dict({"highlevel": {"buffer": {"slotz": 2}}})
    with pytest.raises(ValueError):
        config_from_dict({"highlevel": {"buffer": {"slots": 2, "travel_time_s": [1.0]}}})
    times = HighLevelConfig().buffer.travel_times()
    assert times == (4.0, 4.0, 5.0, 5.0)


# ---------------------------------------------------------------- world


def test_masks_follow_feasibility_and_buffer_state():
    w = world([box(i) for i in range(4)], slots=1)
    mask = w.action_mask()
    assert mask.tolist() == [True, True, False]
    w.step(HighLevelAction(ActionType.BUFFER_CURRENT))
    assert w.buffer[0] is not None and w.current.box.box_id == "B001"
    mask = w.action_mask()
    assert mask.tolist() == [True, False, True]  # slot full -> no BUFFER_CURRENT
    with pytest.raises(ValueError):
        w.step(HighLevelAction(ActionType.BUFFER_CURRENT))
    w.step(HighLevelAction(ActionType.RETRIEVE_BUFFER, 0))
    assert w.buffer[0] is None and [p.box_id for p in w.placed] == ["B000"]
    assert w.current.box.box_id == "B001"  # retrieving keeps the current box waiting


def test_close_rule_starts_a_new_pallet_and_every_box_is_valid():
    # three 0.2 x 0.4 floor boxes fill the 0.62 m pallet; heavier ones cannot
    # stack on lighter ones -> PALLET_CLOSE by rule, no buffer
    boxes = [box(i, weight=5.0) for i in range(3)] + [box(3 + i, weight=20.0, sku="H") for i in range(3)]
    no_repack = replace(HighLevelConfig().repack, enabled=False)
    out = run_policy(world(boxes, slots=0, repack=no_repack), GreedyPolicy())
    assert out["pallets_used"] == 2 and out["pallets_closed"] == 1
    assert out["counts"]["PALLET_CLOSE"] == 1
    assert out["placed"] == 6 and out["ng"] == 0 and out["safety_issues"] == 0
    assert out["pallets"][0]["boxes"] == 3
    # with PARTIAL_REPACK a light box is stacked on another light one, which
    # frees floor for the heavy boxes: the first pallet takes more boxes
    out = run_policy(world(boxes, slots=0), GreedyPolicy())
    assert out["counts"]["PARTIAL_REPACK"] >= 1
    assert out["pallets"][0]["boxes"] > 3 and out["safety_issues"] == 0


def test_buffer_avoids_a_close():
    # light, heavy, light, light, heavy, heavy: buffering the first heavy one
    # lets the light boxes finish the floor... the rule policy must never
    # use more pallets than the no-buffer baseline on this stream
    weights = [5, 20, 5, 5, 20, 20]
    boxes = [box(i, weight=w, sku="H" if w > 10 else "K") for i, w in enumerate(weights)]
    no_buf = run_policy(world(boxes, slots=0), GreedyPolicy())
    rule = run_policy(world(boxes, slots=2), RulePolicy(HighLevelConfig()))
    assert rule["pallet_equivalents"] <= no_buf["pallet_equivalents"] + 1e-9
    assert rule["safety_issues"] == 0 and rule["placed"] == 6


def test_oversize_box_goes_to_ng_not_forced():
    boxes = [box(0), box(1, size=(0.8, 0.5, 0.1), sku="X"), box(2)]
    out = run_policy(world(boxes), RulePolicy(HighLevelConfig()))
    assert out["ng"] == 1 and out["placed"] == 2
    assert out["counts"]["REJECT_NG"] == 1


def test_end_of_stream_flushes_the_buffer():
    boxes = [box(i) for i in range(3)]
    w = world(boxes, slots=2)
    w.step(HighLevelAction(ActionType.BUFFER_CURRENT))
    w.step(HighLevelAction(ActionType.BUFFER_CURRENT))
    w.step(HighLevelAction(ActionType.PLACE_CURRENT))
    assert w.current is None and not w.done
    mask = w.action_mask()
    assert not mask[0] and not mask[1] and mask[2] and mask[3]
    while not w.done:
        w.step(int(np.flatnonzero(w.action_mask())[0]))
    assert w.summary()["placed"] == 3


# ---------------------------------------------------------------- repack


def test_repack_moves_an_accessible_box_to_free_the_floor():
    w = world([box(9, size=(0.4, 0.4, 0.1), sku="C")], slots=0, repack=replace(HighLevelConfig().repack))
    # A at x=0, B at x=0.4 -> 0.2 m gap in the middle; C needs 0.4 m of floor
    a = PlacedBox("A", "K", Size3D(0.2, 0.4, 0.1), 5.0, Pose3D("pallet", 0.004, 0.004, 0.0))
    b = PlacedBox("B", "K", Size3D(0.2, 0.4, 0.1), 5.0, Pose3D("pallet", 0.414, 0.004, 0.0))
    for p in (a, b):
        w._boxes[p.box_id] = replace(make_box(p.box_id, (0.2, 0.4, 0.1)), sku_id="K")
        w.placed_truth[p.box_id] = w._boxes[p.box_id]
    w.placed = [a, b]
    w._invalidate()
    assert not w.action_mask().any()
    assert sorted(accessible_ids(w, w.placed)) == ["A", "B"]
    plan = plan_repack(w)
    assert plan is not None
    moves, _ = plan
    assert len(moves) == 1
    w._apply_repack(plan)
    assert w.action_mask()[0]
    w.step(HighLevelAction(ActionType.PLACE_CURRENT))
    s = w.summary()
    assert s["pallets_used"] == 1 and s["placed"] == 3 and s["safety_issues"] == 0


def test_close_before_buffer_rule():
    # 3 light floor boxes fill the 0.62 m pallet (fill 0.4); a heavy box
    # cannot go on top. Rule on: close instead of buffering it.
    boxes = [box(i, weight=5.0) for i in range(3)] + [box(3, weight=20.0, sku="H")]
    no_repack = replace(HighLevelConfig().repack, enabled=False)
    fill_mode = replace(HighLevelConfig().close, mode="fill")
    off = world(boxes, slots=2, repack=no_repack, close=replace(fill_mode, fill_before_buffer=0.0))
    for _ in range(3):
        off.step(HighLevelAction(ActionType.PLACE_CURRENT))
    assert off.action_mask().tolist() == [False, True, False, False]  # must buffer
    on = world(boxes, slots=2, repack=no_repack, close=replace(fill_mode, fill_before_buffer=0.3))
    for _ in range(3):
        on.step(HighLevelAction(ActionType.PLACE_CURRENT))
    assert on.counts["PALLET_CLOSE"] == 1 and on.placed == []
    assert on.action_mask()[0]  # heavy box goes on the new pallet directly


def test_dead_pallet_close_rule():
    # same cell: the only box still expected (the heavy one) has no safe spot,
    # so the whole expected volume is "dead" -> close (no fill threshold involved)
    boxes = [box(i, weight=5.0) for i in range(3)] + [box(3, weight=20.0, sku="H")]
    no_repack = replace(HighLevelConfig().repack, enabled=False)
    dead = world(boxes, slots=2, repack=no_repack, close=replace(HighLevelConfig().close, mode="dead"))
    for _ in range(3):
        dead.step(HighLevelAction(ActionType.PLACE_CURRENT))
    assert dead.counts["PALLET_CLOSE"] == 1 and dead.action_mask()[0]
    # a stricter share than what is dead keeps the pallet open (box must be buffered)
    keep = world(boxes, slots=2, repack=no_repack,
                 close=replace(HighLevelConfig().close, mode="dead", dead_share=1.01))
    for _ in range(3):
        keep.step(HighLevelAction(ActionType.PLACE_CURRENT))
    assert keep.counts["PALLET_CLOSE"] == 0


def test_order_list_known_flag_controls_remaining_counts():
    boxes = [box(i) for i in range(4)]
    known = world(boxes)
    assert known.state().inventory.remaining_by_sku == {"K": 3}
    hidden = world(boxes, features=replace(HighLevelConfig().features, order_list_known=False))
    assert dict(hidden.state().inventory.remaining_by_sku) == {}


def test_ng_before_first_decision_is_charged_once():
    boxes = [box(0, size=(0.8, 0.5, 0.1), sku="X"), box(1)]
    w = world(boxes)
    assert w.ng == ["B000"]
    r = w.step(HighLevelAction(ActionType.PLACE_CURRENT))
    vol = 0.2 * 0.4 * 0.1 / (SMALL.x * SMALL.y * SMALL.z)
    assert r < vol  # includes the -ng_penalty of the rejected first box
    assert r == pytest.approx(vol - HighLevelConfig().reward.ng_penalty
                              - HighLevelConfig().reward.time_weight * HighLevelConfig().timing.place_time_s)


def test_detected_damage_box_never_carries_anything():
    # B000 is damaged (detected): it may be placed, but nothing on top of it
    boxes = [box(i, size=(0.2, 0.4, 0.1)) for i in range(6)]
    cfg = HighLevelConfig()
    cfg = replace(cfg, buffer=replace(cfg.buffer, slots=0))
    cat = catalog(("K", (0.2, 0.4, 0.1), 5.0))
    arrivals = [Arrival(b, damage_detected=(b.box_id == "B000")) for b in boxes]
    w = PalletizingWorld(arrivals, SMALL, cat, CandidateConfig(), cfg)
    policy = GreedyPolicy()
    saw_b000 = False
    while not w.done:
        w.step(policy(w))
        if any(p.box_id == "B000" for p in w.placed):
            saw_b000 = True
            model = w.backend().model_for(w.state())
            assert all(c.supporter_id != "B000" for cs in model.contacts.values() for c in cs)
    assert saw_b000 and w.capacity_overrides == {"B000": 0.0}
    assert w.summary()["placed"] == 6 and w.summary()["safety_issues"] == 0


def test_repack_footprint_rule():
    from pac_highlevel.repack import _footprint

    a = Pose3D("pallet", 0.1, 0.1, 0.0, yaw=0.0)
    turned = Pose3D("pallet", 0.1, 0.1, 0.0, yaw=HALF_PI)
    flipped = Pose3D("pallet", 0.1, 0.1, 0.0, yaw=math.pi)
    size = Size3D(0.4, 0.2, 0.1)
    assert _footprint(a, size) != _footprint(turned, size)  # rotate in place: a real move
    assert _footprint(a, size) == _footprint(flipped, size)  # same footprint: not a move


def test_donghan_value_provider_requires_a_model():
    pytest.importorskip("pac_planning")
    from pac_highlevel.value import make_value_provider

    with pytest.raises(ValueError, match="model_path"):
        make_value_provider("donghan")
