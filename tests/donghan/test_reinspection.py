"""RI reasons on top of the integrated StateValidator (origin/main, unmodified)."""
import math
from dataclasses import replace
from pathlib import Path

import pytest
from pac_common import RejectCode, Size3D
from pac_runtime.config import ValidatorConfig
from pac_runtime.perception import RawObservation
from pac_runtime.state_validator import Anomaly, StateValidator

from pac_perception.depth_measurement import DepthConfig, measure_box
from pac_perception.signals import PerceptionSignals, signals_from_measurement
from pac_reinspection import REASONS, Reacquired, ReinspectionValidator, inventory_consistency, load_policy, policy_from_dict

from perception_testkit import CATALOG, WEIGHT_RANGES, camera, render

POLICY_PATH = Path(__file__).resolve().parents[2] / "config" / "donghan" / "reinspection_policy.yaml"
K08 = Size3D(0.34, 0.25, 0.21)
K03 = Size3D(0.27, 0.18, 0.15)
S = PerceptionSignals


def obs(label="K08", weight=3.0, size=K08, conf=1.0, damage=False, box_id="B001"):
    return RawObservation(box_id, label, weight, size, conf, damage, "top", 1.0)


def base():
    return StateValidator(CATALOG, ValidatorConfig(), WEIGHT_RANGES)


class Script:
    """reacquire(kind, box_id, attempt): scripted stage-1 results per kind, None when exhausted."""

    def __init__(self, **by_kind):
        self.by_kind = {k: list(v) for k, v in by_kind.items()}
        self.calls = []

    def __call__(self, kind, box_id, attempt):
        self.calls.append(kind)
        q = self.by_kind.get(kind, [])
        return q.pop(0) if q else None


class BaseView:
    def __init__(self, label=None):
        self.label = label

    def base_view(self, _field, prev):
        label = prev.label_sku or self.label
        return replace(prev, label_sku=label, confidence=1.0 if label else prev.confidence, view="base")


@pytest.fixture(scope="module")
def policy():
    return load_policy(POLICY_PATH)


def run(policy, o, signals=None, script=None, remaining=None, perception=None):
    v = ReinspectionValidator(base(), policy, script, remaining)
    verdict = v.validate(o, perception, None, signals=signals)
    return verdict, v.last


# ------------------------------------------------------------------ policy
def test_policy_is_marked_assumed_and_only_enables_implemented_codes(policy):
    assert policy.assumed
    assert policy.retries == {"label": 2, "weigh": 2, "frames": 3}
    for code, cfg in policy.reasons.items():
        assert REASONS[code].implemented_here or not cfg.get("enabled")


def test_every_code_is_documented():
    doc = (Path(__file__).resolve().parents[2] / "docs" / "donghan" / "reinspection_policy.md").read_text(encoding="utf-8")
    assert [c for c in REASONS if f"| {c} |" not in doc] == []


def test_policy_rejects_hardware_codes_and_unknown_codes():
    data = {"schema_version": 1, "budget": {"total_time_s": 1, "max_verdict_changes": 1},
            "retries": {"label": 1, "weigh": 1, "frames": 1},
            "costs_s": dict.fromkeys(("relabel", "reweigh", "rezero", "recapture", "frames", "separate", "second_view"), 1)}
    with pytest.raises(ValueError, match="RI-C4"):
        policy_from_dict({**data, "reasons": {"RI-C4": {"enabled": True}}})
    with pytest.raises(ValueError, match="unknown"):
        policy_from_dict({**data, "reasons": {"RI-Z9": {}}})


# ------------------------------------------------------------------ regression: base behaviour kept
@pytest.mark.parametrize("o", [
    obs(), obs(label=None), obs(conf=0.4), obs(label="X99"), obs(damage=True), obs(conf=0.6),
    obs(size=Size3D(0.34, 0.25, 0.30)), obs(weight=8.0), obs(label="K13", size=Size3D(0.55, 0.48, 0.40), weight=20.0),
])
def test_without_signals_and_retries_kind_route_and_codes_match_the_base_validator(policy, o):
    expected = base().validate(o, None, None)
    got, _ = run(policy, o)
    assert (got.kind, got.route, got.uncertain) == (expected.kind, expected.route, expected.uncertain)
    assert tuple(got.codes) == tuple(expected.codes)
    assert got.notes[:len(expected.notes)] == expected.notes


# ------------------------------------------------------------------ existing reasons, tagged
def test_existing_reasons_are_tagged(policy):
    assert "RI-A1" in run(policy, obs(label=None), perception=BaseView())[1].ri_codes
    v, r = run(policy, obs(label=None), perception=BaseView("K08"))
    assert v.kind == Anomaly.OK and "RI-A1" in r.ri_codes and r.attempts["base_view"] == 1
    assert "RI-A2" in run(policy, obs(conf=0.4))[1].ri_codes
    v, r = run(policy, obs(label="X99"))
    assert (v.kind, v.route) == (Anomaly.UNKNOWN_SKU, "INSPECTION") and "RI-A4" in r.ri_codes
    assert "RI-D1" in run(policy, obs(damage=True))[1].ri_codes
    v, r = run(policy, obs(conf=0.6))
    assert v.uncertain and "RI-C1" in r.ri_codes
    assert "RI-B5" in run(policy, obs(size=Size3D(0.25, 0.34, 0.21)))[1].ri_codes
    v, r = run(policy, obs(weight=1.0))
    assert v.kind == Anomaly.SPEC_MISMATCH and {"RI-B2", "RI-D3"} <= set(r.ri_codes)
    assert any(n.startswith("RI-B2:") for n in v.notes)


# ------------------------------------------------------------------ A. identity
def test_a3_bad_label_payload_is_reread_or_dropped(policy):
    ok = Script(relabel=[Reacquired(signals=S(label_payload_ok=True))])
    v, r = run(policy, obs(), S(label_payload_ok=False), ok)
    assert v.kind == Anomaly.OK and "RI-A3" in r.ri_codes and r.attempts["label"] == 1
    v, r = run(policy, obs(), S(label_payload_ok=False), Script())
    assert (v.kind, v.route) == (Anomaly.RECOGNITION_FAIL, "INSPECTION") and {"RI-A3", "RI-A1"} <= set(r.ri_codes)


def test_a7_top_and_second_view_disagree_majority_of_three(policy):
    v, _ = run(policy, obs(), S(base_label_sku="K13"), Script(relabel=[Reacquired(obs=obs())]))
    assert (v.kind, v.sku) == (Anomaly.OK, "K08")
    v, r = run(policy, obs(), S(base_label_sku="K13"), Script(relabel=[Reacquired(obs=obs(label="K03"))]))
    assert (v.kind, v.route) == (Anomaly.RECOGNITION_FAIL, "INSPECTION") and "RI-A7" in r.ri_codes


def test_a6_sold_out_sku_is_reread_then_inspected(policy):
    script = Script(relabel=[Reacquired(obs=obs()), Reacquired(obs=obs())])
    v, r = run(policy, obs(), script=script, remaining={"K08": 0})
    assert (v.kind, v.route) == (Anomaly.UNKNOWN_SKU, "INSPECTION") and RejectCode.INVALID_STATE in v.codes
    assert r.attempts["label"] == 2 and "RI-A6" in r.ri_codes
    v, r = run(policy, obs(), remaining=lambda: {"K08": 1})
    assert v.route == "PLAN" and "RI-A6" not in r.ri_codes


# ------------------------------------------------------------------ B. sensor cross-check
def test_b3_size_and_weight_off_rereads_the_label_first(policy):
    k13 = obs(label="K08", size=Size3D(0.52, 0.48, 0.40), weight=20.0)
    v, r = run(policy, k13, script=Script(relabel=[Reacquired(obs=replace(k13, label_sku="K13"))]))
    assert (v.kind, v.sku) == (Anomaly.OK, "K13") and "RI-B3" in r.ri_codes


def test_b4_height_only_is_recaptured(policy):
    tall = obs(size=Size3D(0.34, 0.25, 0.26))
    v, r = run(policy, tall, script=Script(recapture=[Reacquired(obs=obs())]))
    assert v.kind == Anomaly.OK and "RI-B4" in r.ri_codes
    v, r = run(policy, tall)
    assert v.kind == Anomaly.SPEC_MISMATCH and "RI-B4" in r.ri_codes


def test_b2_weight_off_is_reweighed(policy):
    v, r = run(policy, obs(weight=8.0), script=Script(reweigh=[Reacquired(obs=obs(weight=3.0))]))
    assert v.kind == Anomaly.OK and r.attempts["weigh"] == 1


def test_b6_b7_unstable_scale_reweigh_or_plan_with_delta(policy):
    good = Script(reweigh=[Reacquired(obs=obs(), signals=S(scale_settled=True, scale_stdev_kg=0.02))])
    v, r = run(policy, obs(), S(scale_settled=False), good)
    assert v.kind == Anomaly.OK and not v.uncertain and RejectCode.SENSOR_UNCERTAIN not in v.codes
    v, r = run(policy, obs(), S(scale_stdev_kg=0.3), Script())
    assert (v.route, v.uncertain) == ("PLAN", True) and RejectCode.SENSOR_UNCERTAIN in v.codes and "RI-B6" in r.ri_codes
    v, r = run(policy, obs(), S(tare_ok=False))
    assert v.uncertain and "RI-B7" in r.ri_codes


# ------------------------------------------------------------------ C. perception quality
def test_c2_frame_spread_takes_more_frames_and_uses_the_median(policy):
    spread = S(frame_sizes=(K08, Size3D(0.36, 0.25, 0.21)))
    tight = S(frame_sizes=(K08, Size3D(0.342, 0.25, 0.21), Size3D(0.341, 0.251, 0.21)))
    v, r = run(policy, obs(), spread, Script(frames=[Reacquired(signals=tight)]))
    assert v.kind == Anomaly.OK and not v.uncertain and "RI-C2" in r.ri_codes
    v, _ = run(policy, obs(), spread)
    assert v.uncertain and RejectCode.SENSOR_UNCERTAIN in v.codes


def test_c3_c7_partial_view_and_missing_depth(policy):
    v, r = run(policy, obs(), S(in_full_view=False), Script(recapture=[Reacquired(signals=S(in_full_view=True))]))
    assert not v.uncertain and "RI-C3" in r.ri_codes
    script = Script(recapture=[Reacquired()] * 5)
    v, r = run(policy, obs(), S(in_full_view=False), script)
    assert r.attempts["frames"] == 3 and v.route == "PLAN" and v.uncertain and RejectCode.SENSOR_UNCERTAIN in v.codes
    v, r = run(policy, obs(), S(valid_ratio=0.5))
    assert v.uncertain and "RI-C7" in r.ri_codes


def test_c8_arrival_pose_is_checked_modulo_90_degrees(policy):
    assert "RI-C8" not in run(policy, obs(), S(arrival_yaw_rad=math.radians(88)))[1].ri_codes
    v, r = run(policy, obs(), S(arrival_yaw_rad=math.radians(40)))
    assert (v.kind, v.route) == (Anomaly.RECOGNITION_FAIL, "INSPECTION") and RejectCode.SENSOR_UNCERTAIN in v.codes


def test_c5_two_boxes_seen_as_one_wait_for_separation(policy):
    merged = obs(label="K03", size=Size3D(0.54, 0.20, 0.15), weight=2.25)
    v, r = run(policy, merged, script=Script(separate=[Reacquired(obs=obs(label="K03", size=K03, weight=1.39))]))
    assert v.kind == Anomaly.OK and "RI-C5" in r.ri_codes
    v, r = run(policy, merged)
    assert (v.kind, v.route) == (Anomaly.SPEC_MISMATCH, "INSPECTION") and "RI-C5" in r.ri_codes


# ------------------------------------------------------------------ D. appearance
def test_d2_damage_near_the_threshold_gets_a_second_view(policy):
    near = S(damage_score=1.1)
    v, _ = run(policy, obs(damage=True), near, Script(second_view=[Reacquired(signals=S(damage_score=0.3))]))
    assert v.kind == Anomaly.OK
    v, _ = run(policy, obs(), S(damage_score=0.8), Script(second_view=[Reacquired(signals=S(damage_score=1.6))]))
    assert (v.kind, v.route) == (Anomaly.DAMAGED, "INSPECTION")
    v, r = run(policy, obs(), S(damage_score=0.9))
    assert v.kind == Anomaly.DAMAGED and "RI-D2" in r.ri_codes


# ------------------------------------------------------------------ F6, budget
def test_f6_flipping_verdicts_force_inspection(policy):
    rv = ReinspectionValidator(base(), policy)
    kinds = [rv.validate(obs(weight=w), None, None).route for w in (8.0, 3.0, 8.0, 3.0)]
    assert kinds[:3] == ["PLAN", "PLAN", "PLAN"] and kinds[3] == "INSPECTION"
    assert "RI-F6" in rv.last.ri_codes


def test_time_budget_stops_re_sensing(policy):
    tight = replace(policy, total_time_s=1.0)
    script = Script(reweigh=[Reacquired(obs=obs())])
    v, r = run(tight, obs(), S(scale_settled=False), script)
    assert script.calls == [] and v.uncertain and any("예산" in n for n in v.notes)


# ------------------------------------------------------------------ E3, stage-1 signals
def test_e3_inventory_consistency():
    assert inventory_consistency({"K08": 2}, {"K08": 1}, {}, {"K08": 1}) == []
    issues = inventory_consistency({"K08": 2}, {"K08": 1}, {}, {"K08": 2})
    assert issues and issues[0].startswith("RI-E3")
    assert inventory_consistency({"K08": 1}, {"K08": 1}, {"K08": 1}, {})


def test_signals_from_measurement():
    cfg = DepthConfig(roi_xy_m=(-0.6, 0.6, -0.45, 0.45))
    clean = signals_from_measurement(measure_box(render(0.34, 0.25, 0.21), camera(), cfg), cfg, scale_settled=True)
    assert clean.in_full_view and clean.damage_score < 0.7 and clean.scale_settled
    dented = signals_from_measurement(measure_box(render(0.34, 0.25, 0.21, dent_m=0.03), camera(), cfg), cfg)
    assert dented.damage_score >= 1.0
