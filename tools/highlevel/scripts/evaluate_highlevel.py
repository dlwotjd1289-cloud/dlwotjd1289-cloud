#!/usr/bin/env python3
"""Compare stage-4 policies on held-out scenarios (same box streams).

Policies: no_buffer (slots = 0), greedy, rule (1st step) and ppo (trained
MaskablePPO, deterministic). Every policy sees identical arrivals, noise and
pallet footprints; differences are therefore paired.

    python tools/highlevel/scripts/evaluate_highlevel.py --run-generator 10 \
        --split test --policy-file ros2_ws/src/pac_highlevel/models/highlevel_ppo.json \
        --report docs/taehyeon/reports/highlevel_eval.json
"""

import argparse
import json
import multiprocessing as mp
import statistics
from dataclasses import replace
from pathlib import Path

from _common import add_common_args, load_all
from pac_highlevel import (
    GreedyPolicy,
    MaskablePPO,
    RulePolicy,
    agent_chooser,
    feature_names,
    policy_contract,
    run_policy,
)
from virtual_data.highlevel import split_ids, world_factory

METRICS = ("pallet_equivalents", "pallets_used", "fill_per_pallet_used", "closed_fill_mean", "placed", "ng",
           "time_s", "decisions", "return", "safety_issues")


def _run(job):
    name, i = job
    world = FACTORIES[name](i)
    out = run_policy(world, CHOOSERS[name])
    out.pop("pallets", None)
    return name, i, out


def main():
    global FACTORIES, CHOOSERS
    parser = argparse.ArgumentParser()
    add_common_args(parser)
    parser.add_argument("--split", default="test")
    parser.add_argument("--passes", type=int, default=3, help="noise / footprint passes per scenario")
    parser.add_argument("--policy-file", type=Path)
    parser.add_argument("--policies", nargs="+", default=["no_buffer", "greedy", "rule", "ppo"])
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    dataset, cand, vcfg, hl = load_all(args)
    specs = split_ids(dataset, args.split)
    episodes = len(specs) * args.passes
    no_buf = replace(hl, buffer=replace(hl.buffer, slots=0))
    FACTORIES, CHOOSERS = {}, {}
    for name in args.policies:
        cfg = no_buf if name == "no_buffer" else hl
        FACTORIES[name] = world_factory(dataset, specs, cand, vcfg, cfg, shuffle_seed=12345)
        if name in ("no_buffer", "greedy"):
            CHOOSERS[name] = GreedyPolicy()
        elif name == "rule":
            CHOOSERS[name] = RulePolicy(hl)
        elif name == "ppo":
            if args.policy_file is None:
                parser.error("--policy-file is required for ppo")
            agent = MaskablePPO.load(args.policy_file, feature_names=feature_names(hl.buffer.slots),
                                     contract=policy_contract(hl, args.candidate_config.name))
            CHOOSERS[name] = agent_chooser(agent, deterministic=True)
        else:
            parser.error(f"unknown policy {name}")
    jobs = [(n, i) for n in args.policies for i in range(episodes)]
    with mp.get_context("fork").Pool(args.workers) as pool:
        rows = pool.map(_run, jobs, chunksize=1)
    by = {n: {} for n in args.policies}
    for name, i, out in rows:
        by[name][i] = out
    summary = {}
    for name in args.policies:
        res = [by[name][i] for i in range(episodes)]
        summary[name] = {
            m: statistics.fmean([r[m] for r in res if r[m] is not None])
            for m in METRICS if any(r[m] is not None for r in res)
        }
        counts = {}
        for r in res:
            for k, v in r["counts"].items():
                counts[k] = counts.get(k, 0) + v
        summary[name]["counts_per_episode"] = {k: v / episodes for k, v in sorted(counts.items())}
    paired = {}
    if "rule" in by:
        for name in args.policies:
            if name == "rule":
                continue
            d = [by[name][i]["pallet_equivalents"] - by["rule"][i]["pallet_equivalents"] for i in range(episodes)]
            paired[name + "_minus_rule"] = {
                "pallet_eq_mean": statistics.fmean(d),
                "better": sum(x < -1e-9 for x in d), "equal": sum(abs(x) <= 1e-9 for x in d),
                "worse": sum(x > 1e-9 for x in d),
            }
    report = {
        "split": args.split, "scenarios": [s.scenario_id for s in specs], "episodes": episodes,
        "buffer_slots": hl.buffer.slots, "summary": summary, "paired": paired,
        "per_episode": {n: [by[n][i] for i in range(episodes)] for n in args.policies},
    }
    for name in args.policies:
        s = summary[name]
        print(f"{name:10s} pallet_eq {s['pallet_equivalents']:.3f} pallets {s['pallets_used']:.2f} "
              f"fill {s['fill_per_pallet_used']:.3f} placed {s['placed']:.1f} ng {s['ng']:.2f} "
              f"time {s['time_s']:.0f}s return {s['return']:.3f} issues {s['safety_issues']:.0f}")
    print(json.dumps(paired, indent=1))
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=1, default=float), encoding="utf-8")


if __name__ == "__main__":
    main()
