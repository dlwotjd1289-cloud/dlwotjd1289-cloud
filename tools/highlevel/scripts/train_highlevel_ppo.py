#!/usr/bin/env python3
"""Train the stage-4 MaskablePPO policy on the virtual data (train split).

    python tools/highlevel/scripts/train_highlevel_ppo.py --run-generator 10 \
        --steps 30000 --workers 4 --output ros2_ws/src/pac_highlevel/models/highlevel_ppo.json
"""

import argparse
import json
from dataclasses import replace
from pathlib import Path

from _common import REPO, add_common_args, load_all
from pac_highlevel import RulePolicy, imitate_teacher, new_agent, policy_contract, train
from virtual_data.highlevel import split_ids, world_factory


def main():
    parser = argparse.ArgumentParser()
    add_common_args(parser)
    parser.add_argument("--steps", type=int)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--learning-rate", type=float, help="override ppo.learning_rate")
    parser.add_argument("--entropy-coef", type=float, help="override ppo.entropy_coef")
    parser.add_argument("--output", type=Path,
                        default=REPO / "ros2_ws/src/pac_highlevel/models/highlevel_ppo.json")
    parser.add_argument("--log", type=Path)
    parser.add_argument("--imitation-episodes", type=int, default=0,
                        help="warm-start from the rule policy with this many teacher episodes")
    parser.add_argument("--imitation-epochs", type=int, default=20)
    args = parser.parse_args()
    dataset, cand, vcfg, hl = load_all(args)
    overrides = {k: v for k, v in (("seed", args.seed), ("learning_rate", args.learning_rate),
                                   ("entropy_coef", args.entropy_coef)) if v is not None}
    if overrides:
        hl = replace(hl, ppo=replace(hl.ppo, **overrides))
    steps = args.steps or hl.ppo.total_steps
    specs = split_ids(dataset, "train")
    make_world = world_factory(dataset, specs, cand, vcfg, hl, shuffle_seed=hl.ppo.seed)
    agent = new_agent(hl, policy_contract(hl, args.candidate_config.name))
    log_file = args.log.open("w", encoding="utf-8") if args.log else None

    def log(row):
        print(json.dumps({k: row[k] for k in ("update", "steps", "episodes", "elapsed_s", "entropy",
                                              "approx_kl", "recent_return", "recent_pallet_eq",
                                              "recent_fill")}), flush=True)
        if log_file:
            log_file.write(json.dumps(row) + "\n")
            log_file.flush()

    if args.imitation_episodes:
        hist = imitate_teacher(agent, make_world, RulePolicy(hl), args.imitation_episodes,
                               workers=args.workers, epochs=args.imitation_epochs)
        print("imitation", json.dumps(hist[-1]), flush=True)
    train(agent, make_world, steps, workers=args.workers, log=log)
    agent.save(args.output, extra={
        "trained_on": {"dataset": str(dataset.root.relative_to(REPO)) if dataset.root.is_relative_to(REPO)
                       else str(dataset.root),
                       "split": "train", "scenarios": [s.scenario_id for s in specs], "steps": steps,
                       "imitation_episodes": args.imitation_episodes},
    })
    print("saved", args.output)


if __name__ == "__main__":
    main()
