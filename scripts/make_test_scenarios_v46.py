#!/usr/bin/env python3
"""Short, targeted V4.6 test scenarios (generator-format arrival files) so each executor feature is
checked in a few robot moves instead of waiting for it inside a full 24-box run.

  python3 scripts/make_test_scenarios_v46.py      # -> test_data/v46_fixtures/dataset/simulation_observations/
Scenarios
  T_BUFFER_SWAP : S0001 boxes 1-12 (restored from test_data/v46_fixtures/pallet12, skipped) + 4 boxes:
                  K11 (no slot -> buffer 0), K05 (fits -> pallet, buffered box re-planned), K08 (no
                  slot -> buffer 1, buffer full), K11 (no slot + buffer full -> pallet change, both
                  buffered boxes from the buffer onto the new pallet, then this box). Test height 0.45 m.
  T_RECOVERY    : S0001 boxes 1-2 on an empty pallet, run with
                  PAC_FAULT=place_offset:box_01,suction_once:box_02 (re-place, re-grip).
  S0001 (full)  : the original 24-box scenario, real height 1.5 / 1.6 m (use the generator dataset).
No-slot / fits were checked offline with ahead_planner_bridge_v44.py on the restored 12-box state.
"""
import copy
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT.parent / "v44_generated/sample_seed20261009/simulation_observations/S0001.jsonl"
CATALOG = ROOT.parent / "ahead-dataset-generator/config/default.yaml"
OUT = ROOT / "test_data/v46_fixtures/dataset/simulation_observations"


def main() -> int:
    import yaml
    sku = {s["sku_id"]: s for s in yaml.safe_load(open(CATALOG))["sku_catalog"]}
    rows = [json.loads(line) for line in open(SRC)]
    OUT.mkdir(parents=True, exist_ok=True)

    def extra(template, idx, sku_id, mass, scen):
        r = copy.deepcopy(template)
        r["arrival_index"] = idx
        r["scenario_id"] = scen
        o = r["observation"]
        o["sku_id"] = sku_id
        o["box_id"] = f"{scen}-B{idx + 1:03d}"
        o["size"] = {k: float(sku[sku_id]["size_m"][k]) for k in ("x", "y", "z")}
        o["weight_kg"] = mass
        o["pose"]["z"] = o["size"]["z"] / 2
        return r

    buf = rows[:12] + [extra(rows[0], 12 + i, s, m, "T_BUFFER_SWAP")
                       for i, (s, m) in enumerate((("K11", 12.0), ("K05", 3.0), ("K08", 6.0), ("K11", 15.0)))]
    rec = rows[:2]
    for name, rs in (("T_BUFFER_SWAP", buf), ("T_RECOVERY", rec)):
        with open(OUT / f"{name}.jsonl", "w") as f:
            for r in rs:
                f.write(json.dumps(r) + "\n")
        print(f"{name}: {len(rs)} arrivals -> {OUT / (name + '.jsonl')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
