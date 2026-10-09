"""Run WAVE / baselines in the team PalletizingWorld; CEM-tune WAVE weights.

    python3 tune.py eval  --cand <yaml> --split test --weights dblf|<json> --out x.json
    python3 tune.py tune  --cand <yaml> --iters 8 --pop 24 --episodes 12 --out tuned.json
"""
import argparse, json, multiprocessing as mp, statistics, sys, time
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import wave  # noqa: E402  (bootstraps team paths)
from pac_candidates import load_candidate_config
from pac_highlevel import load_highlevel_config, RulePolicy, GreedyPolicy
from pac_highlevel.trainer import run_policy
from virtual_data import load_virtual_config
from virtual_data.scenario_source import load_dataset
from virtual_data.highlevel import split_ids, world_factory

REPO = wave.REPO
import os
DATA = Path(os.environ.get("WAVE_DATA", str(Path(__file__).resolve().parents[1] / "lookahead/data/dataset80_x10")))
G = {}

def setup(cand_path, split, hl_path=None):
    ds = load_dataset(DATA)
    G["ds"] = ds
    G["cand"] = load_candidate_config(cand_path)
    G["vcfg"] = load_virtual_config(REPO / "config/taehyeon/virtual_data.yaml")
    G["hl"] = load_highlevel_config(hl_path or REPO / "config/taehyeon/highlevel.yaml")
    G["specs"] = split_ids(ds, split)
    G["ranges"] = {k: (r.weight_min_kg, r.weight_max_kg) for k, r in ds.sku_ranges.items()}

def episode(job):
    weights, i, policy = job
    nf = len(wave.FEATURES)
    if weights is not None and policy == "wave" and len(weights) == nf + 1:   # 9 features + 2 thetas (v1)
        weights = list(weights[:nf - 1]) + [0.0] + list(weights[nf - 1:])
    placer = None if weights is None else wave.WavePlacer(weights[:nf], G["ranges"])
    make = world_factory(G["ds"], G["specs"], G["cand"], G["vcfg"], G["hl"], shuffle_seed=12345, placer=placer)
    world = make(i)
    if policy == "wave":
        chooser = wave.WavePolicy(placer, weights[nf], weights[nf + 1])
    else:
        chooser = RulePolicy(G["hl"]) if policy == "rule" else GreedyPolicy()
    t = time.perf_counter()
    out = run_policy(world, chooser)
    out.pop("pallets", None)
    out["wall_s"] = time.perf_counter() - t
    out["decisions_n"] = out.get("decisions")
    return out

def evaluate(pool, weights, episodes, policy="rule"):
    rows = pool.map(episode, [(weights, i, policy) for i in episodes], chunksize=1)
    return rows

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=("eval", "tune"))
    ap.add_argument("--cand", type=Path, required=True)
    ap.add_argument("--hl", type=Path)
    ap.add_argument("--split", default="train")
    ap.add_argument("--weights", default="dblf")
    ap.add_argument("--policy", default="rule")
    ap.add_argument("--passes", type=int, default=3)
    ap.add_argument("--iters", type=int, default=8)
    ap.add_argument("--pop", type=int, default=24)
    ap.add_argument("--episodes", type=int, default=12)
    ap.add_argument("--workers", type=int, default=20)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    setup(a.cand, a.split, a.hl)
    pool = mp.get_context("fork").Pool(a.workers)
    if a.cmd == "eval":
        if a.weights == "none":
            w = None                       # team default placer (DBLF)
        elif a.weights == "dblf":
            w = wave.DBLF_LIKE.tolist()
        else:
            w = json.loads(Path(a.weights).read_text())["mean"]
        eps = list(range(len(G["specs"]) * a.passes))
        rows = evaluate(pool, w, eps, a.policy)
        keys = ("pallet_equivalents", "pallets_used", "fill_per_pallet_used", "placed", "ng", "safety_issues", "time_s", "wall_s")
        summ = {k: statistics.fmean(r[k] for r in rows) for k in keys}
        summ["decision_ms_mean"] = statistics.fmean(1000 * r["wall_s"] / max(1, r["decisions"]) for r in rows)
        a.out.write_text(json.dumps({"weights": w, "split": a.split, "episodes": len(eps), "policy": a.policy,
                                     "summary": summ, "per_episode": rows}, indent=1, default=float))
        print(json.dumps(summ))
        return
    # CEM (noisy, Szita & Lorincz 2006) over feature weights; objective = mean pallet_equivalents on train episodes
    rng = np.random.default_rng(a.seed)
    init = json.loads(Path(a.weights).read_text())["mean"] if a.weights not in ("dblf", "none") else wave.DBLF_LIKE.tolist()
    nf = len(wave.FEATURES)
    if a.policy == "wave" and len(init) == nf - 1:
        init = list(init) + [0.0, 1.0, 0.1]
    elif a.policy == "wave" and len(init) == nf + 1:
        init = list(init[:nf - 1]) + [0.0] + list(init[nf - 1:])
    elif a.policy == "wave" and len(init) == nf:
        init = list(init) + [1.0, 0.1]
    mean = np.asarray(init, dtype=float)
    d = len(mean)
    std = np.full(d, 3.0)
    eps = list(range(min(a.episodes, len(G["specs"]) * a.passes)))
    hist = []
    best = (float("inf"), mean.tolist())
    for it in range(a.iters):
        pop = [mean.tolist()] + [(mean + std * rng.standard_normal(d)).tolist() for _ in range(a.pop - 1)]
        jobs = [(w, i, a.policy) for w in pop for i in eps]
        rows = pool.map(episode, jobs, chunksize=1)
        scores = []
        for k in range(len(pop)):
            r = rows[k * len(eps):(k + 1) * len(eps)]
            scores.append(statistics.fmean(x["pallet_equivalents"] for x in r))
        order = np.argsort(scores)
        elite = np.array([pop[j] for j in order[: max(2, a.pop // 4)]])
        mean = elite.mean(axis=0)
        std = elite.std(axis=0) + max(0.05, 1.0 * (1 - it / a.iters))   # noise term decays
        if scores[order[0]] < best[0]:
            best = (scores[order[0]], pop[order[0]])
        hist.append({"iter": it, "best": scores[order[0]], "pop_mean_score": statistics.fmean(scores),
                     "incumbent_score": scores[0], "mean": mean.tolist()})
        print(json.dumps(hist[-1]), flush=True)
        a.out.write_text(json.dumps({"features": wave.FEATURES, "mean": mean.tolist(), "best": best[1],
                                     "best_score": best[0], "history": hist}, indent=1))

if __name__ == "__main__":
    main()
