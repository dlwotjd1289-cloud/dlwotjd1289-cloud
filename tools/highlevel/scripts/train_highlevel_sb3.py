#!/usr/bin/env python3
"""Train the stage-4 policy with sb3-contrib MaskablePPO (PyTorch).

Same world, features, masks and rule-teacher warm start as the NumPy
trainer (train_highlevel_ppo.py); only the learner differs.

    pip install torch sb3-contrib gymnasium
    python tools/highlevel/scripts/train_highlevel_sb3.py --run-generator 10 \
        --steps 100000 --imitation-episodes 120 \
        --output ros2_ws/src/pac_highlevel/models/highlevel_sb3.zip
"""

import argparse
from dataclasses import replace
import json
from pathlib import Path
import time

from _common import REPO, add_common_args, load_all

from pac_highlevel import RulePolicy, policy_contract
from pac_highlevel import sb3
from pac_highlevel.trainer import collect_teacher
from virtual_data.highlevel import split_ids, world_factory


def main():
    parser = argparse.ArgumentParser()
    add_common_args(parser)
    parser.add_argument("--steps", type=int)
    parser.add_argument("--envs", type=int, default=4)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--learning-rate", type=float)
    parser.add_argument("--entropy-coef", type=float)
    parser.add_argument("--imitation-episodes", type=int, default=120)
    parser.add_argument("--imitation-epochs", type=int, default=20)
    parser.add_argument("--imitation-lr", type=float, default=3e-4)
    parser.add_argument("--no-normalize", action="store_true", help="disable VecNormalize")
    parser.add_argument("--output", type=Path,
                        default=REPO / "ros2_ws/src/pac_highlevel/models/highlevel_sb3.zip")
    parser.add_argument("--log", type=Path)
    args = parser.parse_args()
    dataset, cand, vcfg, hl = load_all(args)
    overrides = {k: v for k, v in (("seed", args.seed), ("learning_rate", args.learning_rate),
                                   ("entropy_coef", args.entropy_coef)) if v is not None}
    if overrides:
        hl = replace(hl, ppo=replace(hl.ppo, **overrides))
    steps = args.steps or hl.ppo.total_steps
    specs = split_ids(dataset, "train")
    make_world = world_factory(dataset, specs, cand, vcfg, hl, shuffle_seed=hl.ppo.seed)
    contract = policy_contract(hl, args.candidate_config.name)
    history = []
    started = time.perf_counter()

    env = sb3.make_vec_env(make_world, hl.buffer.slots, n_envs=args.envs, normalize=not args.no_normalize)
    model = sb3.new_model(env, hl.ppo)
    if args.imitation_episodes:
        data = collect_teacher(make_world, RulePolicy(hl), args.imitation_episodes,
                               gamma=hl.ppo.gamma, workers=args.envs)
        hist = sb3.imitate(model, data, epochs=args.imitation_epochs, batch_size=hl.ppo.batch_size,
                           lr=args.imitation_lr)
        history += hist
        print("imitation", json.dumps(hist[-1]), flush=True)

    from stable_baselines3.common.callbacks import BaseCallback

    class Log(BaseCallback):
        def __init__(self):
            super().__init__()
            self.episodes = []

        def _on_step(self):
            for info in self.locals.get("infos", ()):
                if "summary" in info:
                    self.episodes.append(info["summary"])
            return True

        def _on_rollout_end(self):
            recent = self.episodes[-20:]
            row = {"steps": self.num_timesteps, "episodes": len(self.episodes),
                   "elapsed_s": round(time.perf_counter() - started, 1)}
            if recent:
                row["recent_pallet_eq"] = sum(e["pallet_equivalents"] for e in recent) / len(recent)
                row["recent_fill"] = sum(e["fill_per_pallet_used"] for e in recent) / len(recent)
            for key in ("train/entropy_loss", "train/approx_kl", "train/value_loss"):
                if key in self.logger.name_to_value:
                    row[key.split("/")[1]] = float(self.logger.name_to_value[key])
            history.append(row)
            print(json.dumps(row), flush=True)

    model.learn(total_timesteps=steps, callback=Log())
    env.close()
    sb3.save(model, args.output, hl.buffer.slots, contract, extra={
        "trained_on": {"split": "train", "scenarios": [s.scenario_id for s in specs], "steps": steps,
                       "imitation_episodes": args.imitation_episodes,
                       "imitation_lr": args.imitation_lr, "normalize": not args.no_normalize},
        "history": history,
    })
    if args.log:
        args.log.write_text("\n".join(json.dumps(r) for r in history), encoding="utf-8")
    print("saved", args.output)


if __name__ == "__main__":
    main()
