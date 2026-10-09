"""Validation config and scoring (no PyBullet needed)."""

import json
from pathlib import Path
import subprocess
import sys

from stability_validation import analytic_checks, classify, evaluate_layout, load_validation_config, summarize

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "config/stability_validation.yaml"


def layout(pid, boxes):
    return {"pallet_id": pid, "pallet_size": [1.1, 1.1, 1.5], "layout": boxes}


def b(i, x, y, z, size, m=10.0):
    return {"box_id": f"{i}", "size": list(size), "pose": [x, y, z, 0.0], "mass_kg": m}


STABLE = layout("stable", [b(0, 0.1, 0.1, 0.0, (0.4, 0.4, 0.3)), b(1, 0.1, 0.1, 0.3, (0.4, 0.4, 0.3))])
TALL = layout("tall", [b(0, 0.4, 0.4, 0.0, (0.2, 0.2, 0.8))])


def test_repo_config_lists_only_sourced_tests_and_their_values():
    cfg = load_validation_config(CONFIG)
    names = {pr.name: pr for pr in cfg.profiles}
    assert cfg.wrapped and cfg.wrapping["deck_height_m"] == 0.15
    # EN 12195-1 design values are off by default
    assert set(names) == {"tilt_10deg_prEN17321", "tilt_0_3g", "lateral_max_1000ms", "lateral_max_80ms",
                          "braking_max_1000ms", "braking_max_80ms"}
    assert round(names["tilt_10deg_prEN17321"].tilt_deg, 1) == 10.0
    assert round(names["tilt_0_3g"].tilt_deg, 1) == 16.7
    assert (names["lateral_max_1000ms"].accel_g, names["lateral_max_1000ms"].hold_s) == (0.330, 27.18)
    assert cfg.thresholds == [(0.10, 10.0), (0.05, 5.0), (0.15, 15.0)]
    assert cfg.lateral == {"enabled": True, "accel_g": 0.3}


def test_analytic_checks_follow_the_enable_flags(tmp_path):
    from pac_candidates.stability import boxes_from_layout

    cfg = load_validation_config(CONFIG)
    res = analytic_checks(boxes_from_layout(TALL["layout"]), cfg)
    assert set(res) == {"lateral", "sme", "pbs"}
    assert not res["lateral"]["ok"] and res["sme"]["ok"] and res["pbs"]["ok"]

    off = tmp_path / "cfg.yaml"
    off.write_text(CONFIG.read_text(encoding="utf-8").replace(
        "    sme:                        # [M] static mechanical equilibrium, most robust in Table 8\n      enabled: true",
        "    sme:\n      enabled: false"), encoding="utf-8")
    assert set(analytic_checks(boxes_from_layout(TALL["layout"]), load_validation_config(off))) == {"lateral", "pbs"}


def test_classification_follows_mazur_table_4():
    assert classify(True, True) == "correct" and classify(False, False) == "correct"
    assert classify(False, True) == "UE"          # rejects a stable layout: overly restrictive
    assert classify(True, False) == "OE"          # accepts an unstable layout: dangerous


def fake(analytic_ok, reference_ok, failing=()):
    tests = [{"profile": name, "pass": False} for name in failing]
    run = {"loading": {"stable": True}, "tests": tests, "pass": reference_ok}
    return {"analytic": {"lateral": {"ok": analytic_ok}}, "primary": "T0.1_R10",
            "reference": {"T0.1_R10": {"pass": reference_ok, "runs": [run, run]}}}


def test_summary_counts_ue_oe_and_failures_once_per_layout():
    results = [fake(True, True), fake(True, False, ["tilt_0_3g"]), fake(False, True), fake(False, False)]
    s = summarize(results)
    lat = s["lateral"]
    assert (lat["correct"], lat["UE"], lat["OE"]) == (2, 1, 1)
    assert lat["accuracy_pct"] == 50.0 and lat["OE_among_passed_pct"] == 50.0
    assert s["reference_failures_among_lateral_passed"] == {"tilt_0_3g": 1}


def test_cli_analytic_only(tmp_path):
    src = tmp_path / "layouts.json"
    src.write_text(json.dumps({"layouts": [STABLE, TALL]}), encoding="utf-8")
    out = tmp_path / "report.json"
    subprocess.run([sys.executable, str(ROOT / "tools/stability/run_stability_validation.py"),
                    "--layouts", str(src), "--analytic-only", "--report", str(out)], check=True,
                   capture_output=True, text=True)
    report = json.loads(out.read_text(encoding="utf-8"))
    assert [r["analytic"]["lateral"]["ok"] for r in report["layouts"]] == [True, False]
    assert report["summary"]["primary"] == {}


def test_evaluate_layout_without_simulation_scores_the_wrapped_unit():
    cfg = load_validation_config(CONFIG)
    res = evaluate_layout(STABLE, cfg, run_simulation=False)
    assert res["boxes"] == 2 and "reference" not in res
    assert all(r["ok"] for r in res["wrapped_unit"].values())


def test_unit_tipping_uses_centre_of_mass_edge_distance_over_height():
    from pac_candidates.stability import StackBox, unit_tipping_check

    centred = [StackBox("c", 0.45, 0.45, 0.0, 0.2, 0.2, 0.8, 10.0)]       # edge 0.55, h 0.4 + 0.15
    assert unit_tipping_check(centred, 0.99, (1.1, 1.1), 0.15).ok
    assert not unit_tipping_check(centred, 1.01, (1.1, 1.1), 0.15).ok
    corner = [StackBox("k", 0.0, 0.0, 0.0, 0.2, 0.2, 0.8, 10.0)]          # edge 0.1 -> 0.18 g
    assert unit_tipping_check(corner, 0.17, (1.1, 1.1), 0.15).ok
    assert not unit_tipping_check(corner, 0.3, (1.1, 1.1), 0.15).ok
