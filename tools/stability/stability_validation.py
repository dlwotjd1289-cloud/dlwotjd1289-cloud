"""Validate the 0.3 g stack check against other stability checks.

For every layout the analytic checks (``lateral`` = the check under
validation, ``sme``, ``pbs``) are compared with a reference label:

* wrapping assumed (default): PyBullet loading test before wrapping, and every
  transport profile as tipping of the whole wrapped unit;
* no wrapping: PyBullet loading test + every profile on the free boxes.

Each analytic check is scored as in Mazur et al. 2025 (ITOR 34(1), Table 4):

* correct: check verdict == reference
* UE (underestimation): check says unstable, reference stable -> overly restrictive
* OE (overestimation): check says stable, reference unstable -> potentially dangerous

Configuration and sources: ``config/stability_validation.yaml``.
"""

from dataclasses import dataclass
from pathlib import Path

import yaml

from pac_candidates.stability import (
    boxes_from_layout,
    lateral_check,
    pbs_check,
    sme_check,
    support_graph,
    unit_tipping_check,
)


@dataclass(frozen=True)
class ValidationConfig:
    lateral: dict
    sme: dict
    pbs: dict
    wrapping: dict
    simulation: dict
    profiles: tuple      # pac_simulation.stability_tests.Profile, enabled only

    @property
    def wrapped(self):
        return bool(self.wrapping.get("assumed"))

    @property
    def thresholds(self):
        """(T, R) pairs; the first is the primary one."""
        sim = self.simulation
        pairs = [(float(sim["max_translation_m"]), float(sim["max_rotation_deg"]))]
        sens = sim.get("sensitivity") or {}
        if sens.get("enabled"):
            pairs += [tuple(map(float, pr)) for pr in sens.get("thresholds", ()) if tuple(map(float, pr)) != pairs[0]]
        return pairs


def load_validation_config(path):
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if raw.get("schema_version") != 1:
        raise ValueError("Unsupported stability validation config schema")
    root = raw["stability_validation"]
    analytic, sim = root["analytic"], root["simulation"]
    from pac_simulation.stability_tests import Profile   # no PyBullet needed for parsing

    profiles = tuple(
        Profile(name, t["mode"], float(t["accel_g"]), float(t["hold_s"]),
                float(t.get("ramp_s", sim.get("ramp_s", 0.5))),
                tuple(t.get("directions", sim.get("directions", ("+x", "-x", "+y", "-y")))))
        for name, t in (sim.get("tests") or {}).items() if t.get("enabled"))
    return ValidationConfig(analytic["lateral"], analytic["sme"], analytic["pbs"], root.get("wrapping") or {},
                            sim, profiles)


def analytic_checks(boxes, cfg):
    graph = support_graph(boxes)
    out = {}
    if cfg.lateral.get("enabled"):
        out["lateral"] = lateral_check(boxes, float(cfg.lateral["accel_g"]), graph)
    if cfg.sme.get("enabled"):
        out["sme"] = sme_check(boxes, graph)
    if cfg.pbs.get("enabled"):
        out["pbs"] = pbs_check(boxes, float(cfg.pbs["alpha"]), graph)
    return {k: {"ok": r.ok, "margin": round(r.margin_m, 5), "worst_box": r.worst_box} for k, r in out.items()}


def wrapped_unit(boxes, pallet_xy, cfg):
    """Every transport profile as tipping of the whole wrapped unit (tilt and
    sustained acceleration give the same tipping limit for a rigid unit)."""
    deck = float(cfg.wrapping.get("deck_height_m", 0.15))
    out = {}
    for prof in cfg.profiles:
        r = unit_tipping_check(boxes, prof.accel_g, pallet_xy, deck)
        out[prof.name] = {"ok": r.ok, "margin": round(r.margin_m, 5), "accel_g": prof.accel_g}
    return out


def simulate(boxes, pallet_xy, cfg):
    """Raw runs per friction value (``LayoutRun``). With wrapping assumed only
    the loading test runs in PyBullet (the profiles act on the wrapped unit)."""
    from pac_simulation.stability_tests import SimSettings, run_layout

    sim = cfg.simulation
    settings = SimSettings(physics_hz=int(sim["physics_hz"]), solver_iterations=int(sim["solver_iterations"]),
                           loading_step_s=float(sim["loading_step_s"]))
    frictions = sim["friction"] if isinstance(sim["friction"], (list, tuple)) else [sim["friction"]]
    profiles = () if cfg.wrapped else cfg.profiles
    return [run_layout(boxes, pallet_xy, float(mu), profiles, cfg.thresholds, settings) for mu in frictions]


def reference(runs, threshold, unit_failed=()):
    """Reference verdict at (T, R): stable only if every friction value passes
    and (wrapping assumed) the wrapped unit stays upright in every profile."""
    verdicts = [r.verdict(*threshold) for r in runs]
    return {"pass": all(v["pass"] for v in verdicts) and not unit_failed, "runs": verdicts,
            "unit_failed": list(unit_failed)}


def evaluate_layout(layout, cfg, run_simulation=True):
    boxes = boxes_from_layout(layout["layout"])
    pallet_xy = tuple(layout["pallet_size"][:2])
    result = {"pallet_id": layout.get("pallet_id", ""), "boxes": len(boxes),
              "analytic": analytic_checks(boxes, cfg)}
    unit_failed = ()
    if cfg.wrapped:
        result["wrapped_unit"] = wrapped_unit(boxes, pallet_xy, cfg)
        unit_failed = [name for name, r in result["wrapped_unit"].items() if not r["ok"]]
    if run_simulation and cfg.simulation.get("enabled"):
        runs = simulate(boxes, pallet_xy, cfg)
        result["reference"] = {f"T{t:g}_R{r:g}": reference(runs, (t, r), unit_failed) for t, r in cfg.thresholds}
        result["primary"] = "T{:g}_R{:g}".format(*cfg.thresholds[0])
    return result


def classify(predicted_stable, reference_stable):
    if predicted_stable == reference_stable:
        return "correct"
    return "OE" if predicted_stable else "UE"


def summarize(results, key=None):
    """Mazur Table 4/8 scores of every analytic check against the reference
    at threshold ``key`` (default: the primary one). Rates are % of layouts."""
    scored = [r for r in results if "reference" in r]
    if not scored:
        return {}
    key = key or scored[0]["primary"]
    out = {"threshold": key, "layouts": len(scored),
           "reference_stable": sum(r["reference"][key]["pass"] for r in scored)}
    for check in scored[0]["analytic"]:
        counts = {"correct": 0, "UE": 0, "OE": 0}
        for r in scored:
            counts[classify(r["analytic"][check]["ok"], r["reference"][key]["pass"])] += 1
        n = len(scored)
        passed = sum(r["analytic"][check]["ok"] for r in scored)
        out[check] = {**counts, "accuracy_pct": round(100 * counts["correct"] / n, 2),
                      "UE_pct": round(100 * counts["UE"] / n, 2), "OE_pct": round(100 * counts["OE"] / n, 2),
                      "check_passed": passed,
                      # share of layouts passing the check that the reference rejects
                      "OE_among_passed_pct": round(100 * counts["OE"] / passed, 2) if passed else None}
    lateral_passed = [r for r in scored if r["analytic"].get("lateral", {}).get("ok")]
    failures = {}
    for r in lateral_passed:                 # layouts counted once per failed test
        failed = set()
        for run in r["reference"][key]["runs"]:
            if not run["loading"]["stable"]:
                failed.add("loading")
            failed.update(t["profile"] for t in run["tests"] if not t["pass"])
        failed.update("unit:" + name for name in r["reference"][key].get("unit_failed", ()))
        for name in failed:
            failures[name] = failures.get(name, 0) + 1
    out["reference_failures_among_lateral_passed"] = failures
    return out
