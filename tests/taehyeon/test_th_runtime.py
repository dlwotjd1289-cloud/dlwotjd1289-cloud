"""Runtime stages 1-3, 7-8 and the 1 -> 8 loop (pac_runtime)."""

from collections import Counter
from dataclasses import replace
import random

import pytest

from th_helpers import REPO, HALF_PI, make_box

from pac_common import BoxStatus, PlacementCandidate, Pose3D, RejectCode, Size3D, SkuSpec
from pac_candidates import CandidateConfig
from pac_highlevel import HighLevelConfig, load_policy
from pac_robot_check import RobotFeasibility
from pac_runtime import (
    Anomaly,
    CellSpec,
    ExecutorSim,
    FieldBox,
    Mode,
    PerceptionSim,
    RuntimeConfig,
    RuntimeLoop,
    StateManager,
    StateValidator,
    Supervisor,
    TrueBox,
    load_runtime_config,
)

PALLET = Size3D(1.2, 1.0, 1.35)
CAT = {
    "K": SkuSpec("K", Size3D(0.4, 0.3, 0.2), 6.0, (0.0, HALF_PI), 2000.0),
    "H": SkuSpec("H", Size3D(0.6, 0.4, 0.3), 20.0, (0.0, HALF_PI), 3000.0),
}
RANGES = {"K": (2.0, 6.0), "H": (12.0, 20.0)}


def truth(i, sku="K", weight=None, size=None):
    s = CAT[sku].size if size is None else Size3D(*size)
    return replace(make_box(f"B{i:03d}", (s.x, s.y, s.z), weight or CAT[sku].weight_kg - 1, sku=sku),
                   status=BoxStatus.ON_CONVEYOR)


def quiet(**kw):
    """Perception without random anomalies (tests inject them explicitly)."""
    cfg = RuntimeConfig()
    base = dict(label_fail_probability=0.0, uncertain_probability=0.0, weight_noise_std_ratio=0.0,
                size_noise_std_m=0.0)
    p = replace(cfg.perception, **{**base, **kw})
    return replace(cfg, perception=p)


# ---------------------------------------------------------------- stage 1 + 2

def test_validator_ok_uses_nominal_size_and_measured_weight():
    cfg = quiet()
    per = PerceptionSim(cfg.perception, random.Random(0))
    val = StateValidator(CAT, cfg.validator, RANGES)
    fb = FieldBox(truth(1, weight=4.2))
    v = val.validate(per.observe(fb, 0.0), per, fb)
    assert v.kind == Anomaly.OK and v.route == "PLAN"
    assert v.box.size == CAT["K"].size and v.box.weight_kg == pytest.approx(4.2)
    assert v.box.status == BoxStatus.MEASURED and not v.uncertain


def test_label_failure_uses_the_base_view_then_inspection():
    cfg = quiet(label_fail_probability=1.0, base_view_recover_probability=1.0)
    per = PerceptionSim(cfg.perception, random.Random(0))
    val = StateValidator(CAT, cfg.validator, RANGES)
    fb = FieldBox(truth(1))
    v = val.validate(per.observe(fb, 0.0), per, fb)
    assert v.kind == Anomaly.OK and v.used_base_view
    cfg = quiet(label_fail_probability=1.0, base_view_recover_probability=0.0)
    per = PerceptionSim(cfg.perception, random.Random(0))
    v = StateValidator(CAT, cfg.validator, RANGES).validate(per.observe(fb, 0.0), per, fb)
    assert v.kind == Anomaly.RECOGNITION_FAIL and v.route == "INSPECTION"


def test_damage_policy_reject_or_no_load():
    cfg = quiet(damage_detect_probability=1.0)
    per = PerceptionSim(cfg.perception, random.Random(0))
    fb = FieldBox(truth(1), damaged=True)
    v = StateValidator(CAT, cfg.validator, RANGES).validate(per.observe(fb, 0.0), per, fb)
    assert v.kind == Anomaly.DAMAGED and v.route == "INSPECTION"
    keep = replace(cfg.validator, damage_policy="place_no_load")
    v = StateValidator(CAT, keep, RANGES).validate(per.observe(fb, 0.0), per, fb)
    assert v.kind == Anomaly.DAMAGED and v.route == "PLAN" and v.no_load_on_top


def test_spec_mismatch_is_remeasured_and_continues_with_measured_values():
    cfg = quiet()
    per = PerceptionSim(cfg.perception, random.Random(0))
    val = StateValidator(CAT, cfg.validator, RANGES)
    fb = FieldBox(truth(1, size=(0.4, 0.3, 0.26)))  # 6 cm taller than the SKU
    v = val.validate(per.observe(fb, 0.0), per, fb)
    assert v.kind == Anomaly.SPEC_MISMATCH and v.route == "PLAN" and v.uncertain
    assert v.box.size.z == pytest.approx(0.26, abs=0.005)
    assert RejectCode.SIZE_MISMATCH in v.codes
    heavy = FieldBox(truth(2, weight=9.0))  # SKU range 2..6 kg
    v = val.validate(per.observe(heavy, 0.0), per, heavy)
    assert v.kind == Anomaly.SPEC_MISMATCH and RejectCode.WEIGHT_MISMATCH in v.codes
    assert v.box.weight_kg == pytest.approx(9.0)
    # x/y swapped is the same carton, not a mismatch
    turned = FieldBox(truth(3, size=(0.3, 0.4, 0.2)))
    assert val.validate(per.observe(turned, 0.0), per, turned).kind == Anomaly.OK


# ---------------------------------------------------------------- stage 3 + 8

def test_supervisor_modes_and_missing():
    sup = Supervisor(RuntimeConfig().supervisor, {"K": 3, "H": 1})
    sup.on_arrival("K")
    assert sup.can_pick()
    blocked = sup.pallet_change(10.0)
    assert blocked == 60.0 and sup.mode == Mode.NORMAL
    assert [e["to"] for e in sup.log] == ["PALLET_CHANGE", "NORMAL"]
    assert sup.confirm_missing(100.0, idle_s=5.0) == Counter()  # not idle long enough
    out = sup.confirm_missing(100.0, idle_s=60.0)
    assert out == Counter({"K": 2, "H": 1}) and sup.outstanding() == Counter()
    sup.hold(200.0, "L4")
    assert sup.time_in["HOLD"] == 120.0


def test_state_manager_versions_inventory_and_snapshot():
    sm = StateManager(PALLET, CAT, {"K": 2, "H": 1}, pallet_max_weight_kg=500, buffer_slots=2)
    b = replace(truth(1), status=BoxStatus.MEASURED)
    sm.arrive(b, uncertain=True)
    s = sm.snapshot()
    assert s.state_version == 1 and s.inventory.remaining_by_sku == {"K": 1, "H": 1}
    assert s.inventory.tracked_boxes["B001"].status == BoxStatus.READY_FOR_PICK
    assert sm.context().uncertain_box_ids == ("B001",)
    sm.to_buffer("B001", 1)
    assert sm.buffer_slots() == {1: "B001"} and sm.current_id() is None
    sm.place("B001", Pose3D("pallet", 0.0, 0.0, 0.0))
    assert sm.buffer_slots() == {} and len(sm.snapshot().pallet.boxes) == 1
    (cx, cy), w = sm.cog_and_weight()
    assert (cx, cy) == pytest.approx((0.2, 0.15)) and w == pytest.approx(5.0)
    sm.confirm_missing({"H": 1})
    assert sm.snapshot().inventory.remaining_by_sku == {"K": 1}
    v = sm.version
    sm.close_pallet()
    assert sm.version == v + 1 and sm.pallet_id.endswith("-02") and not sm.placed


# ---------------------------------------------------------------- stage 7

def test_executor_levels_and_true_geometry():
    cfg = RuntimeConfig()
    ex = ExecutorSim(replace(cfg.execution, place_xy_noise_std_m=0.0), cfg.verify, random.Random(0))
    box = replace(truth(1), status=BoxStatus.MEASURED)
    cand = PlacementCandidate("c", "B001", Pose3D("pallet", 0.1, 0.1, 0.0), 0)
    out = ex.place(box, box.size, cand, [], PALLET)
    assert out.level == "L0" and out.true_pose.x == pytest.approx(0.1)
    # a 6 cm wider true box at the pallet edge protrudes -> L4
    edge = PlacementCandidate("c", "B001", Pose3D("pallet", 0.8, 0.0, 0.0), 0)
    out = ex.place(box, Size3D(0.46, 0.3, 0.2), edge, [], PALLET)
    assert out.level == "L4" and "PROTRUSION" in out.issues
    # settles on what is really under it; half support -> L4 LOW_SUPPORT
    under = [TrueBox("U", Size3D(0.2, 0.3, 0.2), Pose3D("pallet", 0.1, 0.1, 0.0))]
    top = PlacementCandidate("c", "B001", Pose3D("pallet", 0.1, 0.1, 0.2), 0)
    out = ex.place(box, box.size, top, under, PALLET)
    assert out.true_pose.z == pytest.approx(0.2) and any(i.startswith("LOW_SUPPORT") for i in out.issues)


def test_grip_retry_and_other_grasp():
    cfg = RuntimeConfig()
    always = ExecutorSim(replace(cfg.execution, grip_fail_probability=1.0), cfg.verify, random.Random(0))
    assert always.grip() == (False, cfg.execution.max_grip_retries + 2, True)
    never = ExecutorSim(replace(cfg.execution, grip_fail_probability=0.0), cfg.verify, random.Random(0))
    assert never.grip() == (True, 1, False)


# ---------------------------------------------------------------- 1 -> 8

def _cell(stream, expected):
    return CellSpec(stream=tuple(stream), expected_by_sku=expected, pallet_size=PALLET, catalog=CAT,
                    weight_ranges=RANGES, pallet_max_weight_kg=800.0, pallet_prefix="T")


def test_full_loop_places_everything_safely_and_handles_anomalies():
    rng = random.Random(1)
    stream = []
    for i in range(24):
        sku = "H" if rng.random() < 0.3 else "K"
        lo, hi = RANGES[sku]
        stream.append(FieldBox(truth(i, sku, weight=round(rng.uniform(lo, hi), 2))))
    stream[5] = FieldBox(stream[5].truth, damaged=True)
    expected = Counter(fb.truth.sku_id for fb in stream)
    expected["K"] += 2  # two ordered boxes never arrive
    hl = replace(HighLevelConfig(), buffer=replace(HighLevelConfig().buffer, slots=2))
    cfg = replace(quiet(damage_detect_probability=1.0), seed=3)
    out = RuntimeLoop(_cell(stream, dict(expected)), CandidateConfig(), hl, cfg, RobotFeasibility(),
                      load_policy("rule", config=hl)).run()
    assert out["placed"] + len(out["inspection"]) == 24
    assert {"box_id": "B005", "reason": "DAMAGED", "stage": 2} in out["inspection"]
    assert out["missing"] == {"K": 2}
    assert out.get("decisions", {}).get("L4", 0) == 0
    assert out["anomalies"]["DAMAGED"] == 1
    # every placed box was robot-checked: true layout within the pallet, no penetration
    for p in out["pallet_list"]:
        for b in p["layout"]:
            x, y, z, yaw = b["pose"]
            assert -0.003 <= x and -0.003 <= y and z >= 0
    events = [e["event"] for e in out["events"]]
    assert events.count("PLACE") == out["placed"] and "MISSING" in events


def test_runtime_yaml_round_trip():
    assert load_runtime_config(REPO / "config/taehyeon/runtime.yaml") == RuntimeConfig()
