"""CEM with physics in the loop: objective = pallet_equivalents + LAMBDA * boxes
displaced > 20 mm by 0.2 g lateral pulses (unsecured load) in PyBullet.
Fast physics: final layout per pallet placed bottom-up with short settles.
1.1 x 1.1 pallet (simulator pallet), train split only.

    python3 tune_phys.py --init tuned_perbox_wave.json --iters 6 --pop 16 --episodes 12 --out tuned_phys.json
"""
import argparse, json, math, multiprocessing as mp, statistics, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import tune, wave, physics_check as pc  # noqa: E402
import pybullet as p  # noqa: E402
from pac_candidates.geometry import rotated_dims  # noqa: E402

LAMBDA = 0.15


def fast_moved(boxes):
    cfg, _ = pc.load_sim_config(pc.REPO / "config/ahead_simulator.yaml")
    sim = pc.AheadLiveSimulator(cfg)
    hz = cfg.physics.physics_hz
    L, W = cfg.pallet.length_m, cfg.pallet.width_m
    ids = []
    for k, (box, pose) in enumerate(boxes):
        dx, dy, dz = rotated_dims(box.size, pose.yaw)
        bid = f"b{k:03d}"
        sim.place_box(pc.BoxSpec(bid, (box.size.x, box.size.y, box.size.z), box.weight_kg,
                                 (pose.x + dx / 2 - L / 2, pose.y + dy / 2 - W / 2, pose.z + dz / 2), pose.yaw,
                                 source="wave_tune"))
        ids.append(bid)
        sim.step(int(0.1 * hz))
    sim.step(int(0.5 * hz))
    before = {b["id"]: np.array(b["position_m"]) for b in sim.snapshot()["boxes"] if b["id"] in ids}
    cid, g = sim.world.client_id, cfg.physics.gravity_m_s2
    for ax, ay in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        p.setGravity(ax * pc.LAT_G * g, ay * pc.LAT_G * g, -g, physicsClientId=cid)
        sim.step(int(0.5 * hz))
        p.setGravity(0, 0, -g, physicsClientId=cid)
        sim.step(int(0.3 * hz))
    after = {b["id"]: np.array(b["position_m"]) for b in sim.snapshot()["boxes"] if b["id"] in ids}
    sim.close()
    return sum(1 for k, v in before.items() if k in after and np.linalg.norm(after[k] - v) > 0.02)


def episode(job):
    weights, i = job
    out, finals = pc.record_episode(weights, "rule" if weights is None else "wave", i)
    moved = 0
    for v in finals:
        try:
            moved += fast_moved(sorted(v, key=lambda t: (round(t[1].z, 4), t[1].y, t[1].x)))
        except ValueError:          # physical overlap on rebuild -> count every box as moved
            moved += len(v)
    return {"pallet_equivalents": out["pallet_equivalents"], "moved": moved, "time_s": out["time_s"]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--init", type=Path, required=True)
    ap.add_argument("--split", default="train")
    ap.add_argument("--iters", type=int, default=6)
    ap.add_argument("--pop", type=int, default=16)
    ap.add_argument("--episodes", type=int, default=12)
    ap.add_argument("--workers", type=int, default=20)
    ap.add_argument("--seed", type=int, default=2)
    ap.add_argument("--eval", action="store_true", help="evaluate --init mean on --split, no tuning")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    C = Path(__file__).resolve().parents[1] / "lookahead/configs"
    import os
    tune.setup(Path(os.environ.get("WAVE_CAND", str(C / "perbox.yaml"))), a.split)
    nf = len(wave.FEATURES)
    init = None if str(a.init) == "none" else json.loads(a.init.read_text())["mean"]
    if init is not None and len(init) == nf + 1:
        init = list(init[:nf - 1]) + [0.0] + list(init[nf - 1:])
    pool = mp.get_context("fork").Pool(a.workers)
    if a.eval:
        eps = list(range(len(tune.G["specs"])))
        rows = pool.map(episode, [(init, i) for i in eps], chunksize=1)
        s = {k: statistics.fmean(r[k] for r in rows) for k in rows[0]}
        s["moved_total"] = sum(r["moved"] for r in rows)
        a.out.write_text(json.dumps({"weights": init, "summary": s, "per_episode": rows}, indent=1))
        print(json.dumps(s)); return
    rng = np.random.default_rng(a.seed)
    mean = np.asarray(init, dtype=float)
    d = len(mean)
    std = np.full(d, 1.5)
    eps = list(range(a.episodes))
    hist = []
    for it in range(a.iters):
        pop = [mean.tolist()] + [(mean + std * rng.standard_normal(d)).tolist() for _ in range(a.pop - 1)]
        rows = pool.map(episode, [(w, i) for w in pop for i in eps], chunksize=1)
        scores, parts = [], []
        for k in range(len(pop)):
            r = rows[k * len(eps):(k + 1) * len(eps)]
            pe = statistics.fmean(x["pallet_equivalents"] for x in r)
            mv = statistics.fmean(x["moved"] for x in r)
            scores.append(pe + LAMBDA * mv); parts.append((pe, mv))
        order = np.argsort(scores)
        elite = np.array([pop[j] for j in order[: max(2, a.pop // 4)]])
        mean = elite.mean(axis=0)
        std = elite.std(axis=0) + max(0.05, 0.6 * (1 - it / a.iters))
        hist.append({"iter": it, "best": scores[order[0]], "best_parts": parts[order[0]],
                     "incumbent": scores[0], "incumbent_parts": parts[0], "mean": mean.tolist()})
        print(json.dumps({k: v for k, v in hist[-1].items() if k != "mean"}), flush=True)
        a.out.write_text(json.dumps({"features": list(wave.FEATURES) + ["theta_buffer", "theta_retrieve"],
                                     "lambda": LAMBDA, "mean": mean.tolist(), "history": hist}, indent=1))


if __name__ == "__main__":
    main()
