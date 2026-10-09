"""PyBullet stability tests (pac_simulation.stability_tests). Skipped without PyBullet."""

import math

import pytest

pytest.importorskip("pybullet")

from pac_candidates.stability import StackBox  # noqa: E402
from pac_simulation.stability_tests import (  # noqa: E402
    G,
    LayoutWorld,
    Profile,
    gravity_for,
    loading_test,
    loading_verdict,
    run_layout,
)

PALLET = (1.1, 1.1)
T, R = 0.10, 10.0


def cube(i, x, y, z, dx=0.3, dy=0.3, dz=0.3, m=10.0):
    return StackBox(f"b{i}", x, y, z, dx, dy, dz, m)


def single(profile, box, friction):
    run = run_layout([box], PALLET, friction, [profile], [(T, R)])
    return run.verdict(T, R)


def test_gravity_vectors_for_tilt_and_acceleration():
    tilt = Profile("t", "tilt", 0.3, 1.0)
    acc = Profile("a", "acceleration", 0.3, 1.0)
    gx, _, gz = gravity_for(tilt, "+x", 1.0)
    assert math.isclose(math.hypot(gx, gz), G) and math.isclose(gx / -gz, 0.3)
    assert gravity_for(acc, "+x", 1.0) == pytest.approx((-0.3 * G, 0.0, -G))
    assert gravity_for(acc, "-y", 0.5) == pytest.approx((0.0, 0.15 * G, -G))


def test_box_stays_below_friction_and_slides_above_it():
    box = cube(0, 0.4, 0.4, 0.0)
    assert single(Profile("tilt", "tilt", 0.3, 2.0, directions=("+x",)), box, 0.5)["pass"]
    # 0.492 g > mu 0.375: slides (0.117 g for 2 s is ~2 m)
    res = single(Profile("brake", "acceleration", 0.492, 2.0, directions=("+x",)), box, 0.375)
    assert not res["pass"]
    assert res["tests"][0]["directions"]["+x"]["motion"]["translation_m"] > T


def test_tall_box_tips_beyond_half_width_over_height():
    tall = cube(0, 0.45, 0.45, 0.0, 0.2, 0.2, 0.8)       # b / (2 h_cg) = 0.1 / 0.4 = 0.25 g
    assert single(Profile("low", "acceleration", 0.2, 2.0, directions=("+x",)), tall, 0.5)["pass"]
    res = single(Profile("high", "acceleration", 0.3, 2.0, directions=("+x",)), tall, 0.5)
    assert not res["pass"]
    assert res["tests"][0]["directions"]["+x"]["motion"]["rotation_deg"] > R


def test_loading_test_stops_at_the_first_unstable_box():
    boxes = [cube(0, 0.1, 0.1, 0.0, 0.4, 0.4, 0.2), cube(1, 0.4, 0.1, 0.2, 0.4, 0.4, 0.2),   # 25 % support
             cube(2, 0.6, 0.6, 0.0)]
    world = LayoutWorld(boxes, PALLET, 0.5)
    try:
        steps = loading_test(world, boxes, T, R)
    finally:
        world.close()
    verdict = loading_verdict(steps, len(boxes), T, R)
    assert verdict["j_max"] == 1 and math.isclose(verdict["ls"], 1 / 3) and not verdict["stable"]


def test_stable_stack_passes_loading_and_records_every_direction():
    boxes = [cube(0, 0.1, 0.1, 0.0, 0.4, 0.4, 0.3), cube(1, 0.1, 0.1, 0.3, 0.4, 0.4, 0.3)]
    profile = Profile("lat", "acceleration", 0.33, 1.0)
    run = run_layout(boxes, PALLET, 0.5, [profile], [(T, R), (0.05, 5.0)])
    v = run.verdict(T, R)
    assert v["loading"]["ls"] == 1.0 and v["pass"]
    assert set(v["tests"][0]["directions"]) == {"+x", "-x", "+y", "-y"}
    assert run.verdict(0.05, 5.0)["loading"]["stable"]


def _layout(pid, boxes):
    return {"pallet_id": pid, "pallet_size": [1.1, 1.1, 1.5],
            "layout": [{"box_id": bx.box_id, "size": [bx.dx, bx.dy, bx.dz], "pose": [bx.x, bx.y, bx.z, 0.0],
                        "mass_kg": bx.mass_kg} for bx in boxes]}


def _config(tmp_path, wrapped):
    from pathlib import Path

    from stability_validation import load_validation_config

    root = Path(__file__).resolve().parents[2]
    text = (root / "config/stability_validation.yaml").read_text(encoding="utf-8")
    text = text.replace("hold_s: 27.18", "hold_s: 0.5")
    if not wrapped:   # unwrapped boxes slide under braking above mu: test only the lateral profiles
        text = text.replace("assumed: true", "assumed: false")
        text = text.replace("braking_max_1000ms:       # Table 3: min ax1000 -0.492 g; Table 10: longest "
                            "longitudinal event > 0.2 g 11.65 s\n        enabled: true",
                            "braking_max_1000ms:\n        enabled: false")
        text = text.replace("braking_max_80ms:         # Table 3: min ax80 -0.660 g\n        enabled: true",
                            "braking_max_80ms:\n        enabled: false")
    path = tmp_path / "cfg.yaml"
    path.write_text(text, encoding="utf-8")
    cfg = load_validation_config(path)
    assert cfg.wrapped == wrapped
    return cfg


STABLE = [cube(0, 0.1, 0.1, 0.0, 0.4, 0.4, 0.3)]
TALL_CENTRED = [cube(0, 0.45, 0.45, 0.0, 0.2, 0.2, 0.8)]


def test_wrapped_unit_makes_the_tall_centred_box_a_lateral_ue(tmp_path):
    from stability_validation import evaluate_layout, summarize

    cfg = _config(tmp_path, wrapped=True)
    results = [evaluate_layout(_layout("stable", STABLE), cfg), evaluate_layout(_layout("tall", TALL_CENTRED), cfg)]
    assert all(r["ok"] for r in results[1]["wrapped_unit"].values())     # 0.55 / 0.55 m -> 1 g
    s = summarize(results)
    assert s["layouts"] == 2 and s["reference_stable"] == 2
    assert s["lateral"]["UE"] == 1 and s["lateral"]["OE"] == 0       # the 0.3 g check is stricter than needed


def test_unwrapped_reference_agrees_with_the_lateral_check(tmp_path):
    from stability_validation import evaluate_layout, summarize

    cfg = _config(tmp_path, wrapped=False)
    results = [evaluate_layout(_layout("stable", STABLE), cfg), evaluate_layout(_layout("tall", TALL_CENTRED), cfg)]
    s = summarize(results)
    assert s["reference_stable"] == 1
    assert s["lateral"]["accuracy_pct"] == 100.0        # 0.3 g agrees with the simulation on both
    assert s["sme"]["OE"] == 1                          # SME (gravity only) accepts the tall column
