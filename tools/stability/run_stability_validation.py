#!/usr/bin/env python3
"""Score the 0.3 g stack check (and SME / PBS) against PyBullet reference tests.

Input: pallet layouts as written by ``tools/runtime/scripts/physics_replay.py
--report`` (key ``layouts``) or a plain list of
``{"pallet_id", "pallet_size": [x, y, z], "layout": [{"box_id", "size", "pose": [x, y, z, yaw], "mass_kg"}]}``
(pallet frame, box min corner).

    python3 tools/stability/run_stability_validation.py --layouts runtime_physics.json \
        --report stability_validation.json
    python3 tools/stability/run_stability_validation.py --layouts ... --analytic-only

Which checks and tests run is set in config/stability_validation.yaml (``enabled``).
PyBullet is needed for the simulation part (Docker image ``docker/run.sh``).
"""

import argparse
import json
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[2]
for path in (REPO / "tools" / "stability", *(REPO / "ros2_ws/src" / name
                                            for name in ("pac_common", "pac_candidates", "pac_simulation"))):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from stability_validation import evaluate_layout, load_validation_config, summarize  # noqa: E402


def read_layouts(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return data["layouts"] if isinstance(data, dict) else data


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--layouts", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=REPO / "config/stability_validation.yaml")
    parser.add_argument("--limit", type=int, default=0, help="first N layouts (0 = all)")
    parser.add_argument("--analytic-only", action="store_true", help="skip the PyBullet tests")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()

    cfg = load_validation_config(args.config)
    layouts = read_layouts(args.layouts)
    if args.limit:
        layouts = layouts[: args.limit]
    results = []
    for k, layout in enumerate(layouts):
        res = evaluate_layout(layout, cfg, run_simulation=not args.analytic_only)
        results.append(res)
        checks = " ".join(f"{name}={'ok' if r['ok'] else 'NG'}" for name, r in res["analytic"].items())
        unit = res.get("wrapped_unit", {})
        if unit:
            failed = [name for name, r in unit.items() if not r["ok"]]
            checks += " wrapped_unit=" + ("ok" if not failed else "NG(" + ",".join(failed) + ")")
        ref = res.get("reference", {}).get(res.get("primary", ""), {})
        ref_txt = f" reference={'stable' if ref.get('pass') else 'unstable'}" if ref else ""
        print(f"[{k + 1}/{len(layouts)}] {res['pallet_id']} boxes={res['boxes']} {checks}{ref_txt}", flush=True)

    summary = {"primary": summarize(results)}
    thresholds = list(results[0].get("reference", {})) if results else []
    summary["sensitivity"] = {key: summarize(results, key) for key in thresholds[1:]}
    if summary["primary"]:
        print(json.dumps(summary["primary"], indent=1))
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps({"config": str(args.config), "summary": summary, "layouts": results},
                                          indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
