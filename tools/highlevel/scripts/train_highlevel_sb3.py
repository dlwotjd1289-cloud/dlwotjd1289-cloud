#!/usr/bin/env python3
"""Train the stage-4 policy with sb3-contrib MaskablePPO (PyTorch).

Same world, features, masks and rule-teacher warm start as the NumPy
trainer (train_highlevel_ppo.py); only the learner differs.

    pip install torch sb3-contrib gymnasium
    python tools/highlevel/scripts/train_highlevel_sb3.py --run-generator 10 \
        --steps 100000 --imitation-episodes 120 \
        --output ros2_ws/src/pac_highlevel/models/highlevel_sb3.zip

Training aids (all optional):
  --rule-baseline      reward = return minus the Rule's return on the same episode
  --teacher-bc A B     behaviour-cloning towards the Rule on visited states,
                       weight decaying from A to B over training
  --val-every N        every N steps evaluate on the val split and keep the best
                       model in --output (the last one goes to <output>_last.zip)
"""

_VAL = None  # (make_world, choose) for forked val workers


def _val_episode(i):
    make_world, choose = _VAL
    return run_policy(make_world(i), choose)["pallet_equivalents"]


def _val_mean(make_world, choose, episodes, workers):
    global _VAL
    _VAL = (make_world, choose)
    try:
        with mp.get_context("fork").Pool(workers) as pool:
            values = pool.map(_val_episode, range(episodes), chunksize=1)
    finally:
        _VAL = None
    return statistics.fmean(values)


import argparse
from dataclasses import replace
import json
import multiprocessing as mp
from pathlib import Path
import statistics
import time

from _common import REPO, add_common_args, load_all

from pac_highlevel import RulePolicy, policy_contract, run_policy
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
    parser.add_argument("--gamma", type=float)
    parser.add_argument("--rule-baseline", action="store_true")
    parser.add_argument("--teacher-bc", type=float, nargs=2, metavar=("START", "END"))
    parser.add_argument("--val-every", type=int, default=0)
    parser.add_argument("--val-passes", type=int, default=1)
    parser.add_argument("--output", type=Path,
                        default=REPO / "ros2_ws/src/pac_highlevel/models/highlevel_sb3.zip")
    parser.add_argument("--log", type=Path)
    args = parser.parse_args()
    dataset, cand, vcfg, hl = load_all(args)
    overrides = {k: v for k, v in (("seed", args.seed), ("learning_rate", args.learning_rate),
                                   ("entropy_coef", args.entropy_coef), ("gamma", args.gamma)) if v is not None}
    if overrides:
        hl = replace(hl, ppo=replace(hl.ppo, **overrides))
    steps = args.steps or hl.ppo.total_steps
    specs = split_ids(dataset, "train")
    make_world = world_factory(dataset, specs, cand, vcfg, hl, shuffle_seed=hl.ppo.seed)
    contract = policy_contract(hl, args.candidate_config.name)
    history = []
    started = time.perf_counter()

    rule = RulePolicy(hl)
    baseline = (lambda world: run_policy(world, rule)["return"]) if args.rule_baseline else None
    env = sb3.make_vec_env(make_world, hl.buffer.slots, n_envs=args.envs, normalize=not args.no_normalize,
                           baseline=baseline, teacher=rule if args.teacher_bc else None)
    model = sb3.new_model(env, hl.ppo)
    if args.imitation_episodes:
        data = collect_teacher(make_world, rule, args.imitation_episodes,
                               gamma=hl.ppo.gamma, workers=args.envs, relative=args.rule_baseline)
        hist = sb3.imitate(model, data, epochs=args.imitation_epochs, batch_size=hl.ppo.batch_size,
                           lr=args.imitation_lr)
        history += hist
        print("imitation", json.dumps(hist[-1]), flush=True)

    from stable_baselines3.common.callbacks import BaseCallback, CallbackList

    val_specs = split_ids(dataset, "val")
    val_world = world_factory(dataset, val_specs, cand, vcfg, hl, shuffle_seed=777)
    val_episodes = len(val_specs) * args.val_passes
    val_rule = None
    if args.val_every:
        val_rule = _val_mean(val_world, rule, val_episodes, args.envs)
        print(json.dumps({"val_rule_pallet_eq": val_rule, "val_episodes": val_episodes}), flush=True)
    best = {"value": float("inf"), "steps": None}
    meta = {"split": "train", "scenarios": [s.scenario_id for s in specs], "steps": steps,
            "imitation_episodes": args.imitation_episodes, "imitation_lr": args.imitation_lr,
            "normalize": not args.no_normalize, "gamma": hl.ppo.gamma, "rule_baseline": args.rule_baseline,
            "teacher_bc": args.teacher_bc, "val_every": args.val_every, "val_rule_pallet_eq": val_rule}

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
                rel = [e["return"] - e["baseline_return"] for e in recent if "baseline_return" in e]
                if rel:
                    row["recent_return_minus_rule"] = sum(rel) / len(rel)
            for key in ("train/entropy_loss", "train/approx_kl", "train/value_loss",
                        "bc/coef", "bc/loss", "bc/agreement"):
                if key in self.logger.name_to_value:
                    row[key.split("/")[1] if key.startswith("train") else key.replace("/", "_")] = \
                        float(self.logger.name_to_value[key])
            if args.val_every and self.num_timesteps - self.last_val >= args.val_every:
                self.last_val = self.num_timesteps
                value = _val_mean(val_world, sb3.chooser(self._frozen()), val_episodes, args.envs)
                row["val_pallet_eq"] = value
                row["val_minus_rule"] = value - val_rule
                if value < best["value"]:
                    best.update(value=value, steps=self.num_timesteps)
                    sb3.save(self.model, args.output, hl.buffer.slots, contract,
                             extra={"trained_on": {**meta, "selected_at_steps": self.num_timesteps,
                                                   "val_pallet_eq": value}})
                    row["saved_best"] = True
            history.append(row)
            print(json.dumps(row), flush=True)

        def _frozen(self):
            norm = self.model.get_vec_normalize_env()
            self.model.pac_obs_norm = None if norm is None else {
                "mean": norm.obs_rms.mean.tolist(), "var": norm.obs_rms.var.tolist(),
                "clip": float(norm.clip_obs), "eps": float(norm.epsilon)}
            return self.model

    log_cb = Log()
    log_cb.last_val = 0
    callbacks = [log_cb]
    if args.teacher_bc:
        callbacks.insert(0, sb3.teacher_bc_callback(*args.teacher_bc, batch_size=hl.ppo.batch_size))
    model.learn(total_timesteps=steps, callback=CallbackList(callbacks))
    env.close()
    last = args.output.with_name(args.output.stem + "_last.zip") if args.val_every else args.output
    sb3.save(model, last, hl.buffer.slots, contract, extra={
        "trained_on": {**meta, "best_val_pallet_eq": best["value"] if args.val_every else None,
                       "best_at_steps": best["steps"]},
        "history": history,
    })
    if args.log:
        args.log.write_text("\n".join(json.dumps(r) for r in history), encoding="utf-8")
    print("saved", last, "best", args.output if args.val_every else "-", json.dumps(best))


if __name__ == "__main__":
    main()
