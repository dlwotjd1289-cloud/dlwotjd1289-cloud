#!/usr/bin/env python3
"""Run the COMPARISON BASELINE (field-practice rules, pac_planning.baseline.field_rule) on generator scenarios.

    python3 scripts/donghan/baseline/run_field_rule.py --dataset tools/highlevel/output/dataset80_x40_s8 \
        --split test --episodes 3 --output reports/baseline_field_rule_test.json
    # timed run of one episode for the 3D viewer (tools/realtime/export_viewer.py)
    python3 scripts/donghan/baseline/run_field_rule.py --dataset tools/highlevel/output/dataset80_x40_s8 \
        --split test --episode-ids 0 --timeline reports/baseline_field_rule_test0_timeline.json

The cell (pallet sizes, buffer slots, cycle times) comes from
config/taehyeon/environment.yaml and the 5-1/5-2 safety settings from
config/taehyeon/candidates.yaml, loaded with the team's stage-4 helper.
Episode i maps to a scenario and pallet footprint exactly like the team's
evaluation scripts (world_factory, --seed).
"""

import argparse
import json
from pathlib import Path
import random
import subprocess
import sys
import time

OWN = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(OWN / "tools/highlevel/scripts"))
sys.path.insert(0, str(OWN / "ros2_ws/src/pac_planning"))
from _common import add_common_args, load_all, load_env  # noqa: E402  (bootstraps team paths)

from pac_common import plain  # noqa: E402
from pac_highlevel import run_policy  # noqa: E402
from pac_highlevel.realtime import ConveyorConfig, TimedRun  # noqa: E402
from pac_planning.baseline.field_rule import (  # noqa: E402
    FieldRulePlacer,
    FieldRulePolicy,
    load_field_rule_config,
    field_rule_highlevel_config,
)
from virtual_data.highlevel import split_ids, world_factory  # noqa: E402

SUMMARY_KEYS = ("boxes", "placed", "ng", "pallets_used", "pallets_closed", "closed_fill_mean",
                "fill_per_pallet_used", "pallet_equivalents", "time_s", "decisions", "counts", "safety_issues")


def scenario_of(specs, seed, episode):
    """Same episode -> scenario mapping as virtual_data.highlevel.world_factory."""
    epoch, k = divmod(int(episode), len(specs))
    order = list(range(len(specs)))
    random.Random(f"{seed}:{epoch}").shuffle(order)
    return specs[order[k]].scenario_id


def git_commit():
    try:
        return subprocess.run(["git", "-C", str(OWN), "rev-parse", "--short", "HEAD"],
                              capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(parser)
    parser.add_argument("--split", default="test")
    parser.add_argument("--episodes", type=int, default=1, help="first N episodes (ignored with --episode-ids)")
    parser.add_argument("--episode-ids", type=int, nargs="+")
    parser.add_argument("--seed", type=int, default=4242, help="episode -> scenario shuffle (team scripts: 4242)")
    parser.add_argument("--rule-config", type=Path, default=OWN / "config/donghan/baseline/field_rule.yaml")
    parser.add_argument("--output", type=Path, help="per-episode summaries (JSON)")
    parser.add_argument("--timeline", type=Path, help="timed run of the first episode for the 3D viewer")
    args = parser.parse_args()

    dataset, cand, vcfg, hl = load_all(args)
    hl = field_rule_highlevel_config(hl)
    cfg = load_field_rule_config(args.rule_config)
    specs = split_ids(dataset, args.split)
    make = world_factory(dataset, specs, cand, vcfg, hl, shuffle_seed=args.seed, placer=FieldRulePlacer(cfg))
    episodes = args.episode_ids if args.episode_ids else list(range(args.episodes))

    rows = []
    for i in episodes:
        started = time.perf_counter()
        world = make(i)
        out = run_policy(world, FieldRulePolicy())
        row = {"episode": i, "scenario_id": scenario_of(specs, args.seed, i),
               "pallet_size": [world.pallet_size.x, world.pallet_size.y, world.pallet_size.z],
               **{k: out[k] for k in SUMMARY_KEYS}, "wall_s": round(time.perf_counter() - started, 2)}
        rows.append(row)
        fill = row["closed_fill_mean"]
        print(f"ep {i:3d} {row['scenario_id']}: placed {row['placed']}/{row['boxes']}  ng {row['ng']}  "
              f"pallets {row['pallets_used']} (closed {row['pallets_closed']}, "
              f"fill {'-' if fill is None else f'{fill:.1%}'})  eq {row['pallet_equivalents']:.3f}  "
              f"safety {row['safety_issues']}  {row['wall_s']} s")

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({
            "algorithm": "baseline_field_rule", "git_commit": git_commit(), "dataset": str(args.dataset),
            "split": args.split, "seed": args.seed, "rule_config": plain(cfg),
            "highlevel": {"repack": plain(hl.repack), "close": plain(hl.close), "buffer_slots": hl.buffer.slots},
            "episodes": rows,
        }, indent=1), encoding="utf-8")

    if args.timeline:
        env = load_env(args)
        visible = env.visible_boxes if env.visible_boxes >= 0 else 5
        conveyor = ConveyorConfig(env.conveyor_interval_s or 6.0, visible + 1)
        out = TimedRun(make(episodes[0]), FieldRulePolicy(), conveyor, horizon=0).run()
        out.update(policy="baseline_field_rule", placer="baseline_field_rule", episode=episodes[0],
                   scenario_id=scenario_of(specs, args.seed, episodes[0]))
        args.timeline.parent.mkdir(parents=True, exist_ok=True)
        args.timeline.write_text(json.dumps(out), encoding="utf-8")
        print(json.dumps({k: out[k] for k in ("summary", "makespan_s", "robot_idle_s")}, indent=1))


if __name__ == "__main__":
    main()
