"""Calibrate and check hard-mask step 13 (lateral stability, pac_candidates/lateral.py).

collect: run team episodes (rule policy, DBLF placer, run_eval data and seeds)
    with the planner config ``--cand`` (default: step 13 on, so every state
    the samples start from is itself stable). At every placement, evaluate the
    chosen candidate and ``--samples`` random candidates that pass hard-mask
    steps 1-12 for the same box and state: geometric and LP critical
    accelerations (all directions), the short tilt test (stage 3) and the slow
    reference tilt test of the whole pallet
    (pac_candidates.lateral_sim.reference_tilt_test, the "truth").
analyze: emulate the cascade for a grid of (band_low_g, band_high_g) and
    report missed collapses (OE: passed, reference fails), needless
    rejections (UE) and how often each stage runs. Rows after a chosen
    placement that failed the reference (state no longer stable) are dropped.

    python3 tools/donghan/lateral_calibrate.py collect --episodes 7 --planner-borderline reject --out cal.jsonl
    python3 tools/donghan/lateral_calibrate.py collect --cand config/taehyeon/candidates.yaml --samples 0 --out off.jsonl
    python3 tools/donghan/lateral_calibrate.py analyze cal.jsonl
Linux/WSL (fork workers); needs scipy and pybullet.
"""
import argparse
import json
import multiprocessing as mp
import statistics
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools/highlevel/scripts"))
import _common  # noqa: E402,F401  (team path bootstrap)

DATA = REPO / "tools/prototypes/lookahead/data/dataset80_x10"
TEAM_CAND = REPO / "config/taehyeon/candidates.yaml"
LATERAL_CAND = REPO / "config/donghan/candidates_lateral.yaml"
G = {}


def _setup(a):
    from pac_candidates import load_candidate_config
    from pac_highlevel import load_highlevel_config
    from virtual_data import load_virtual_config
    from virtual_data.highlevel import split_ids
    from virtual_data.scenario_source import load_dataset

    ds = load_dataset(Path(a.data))
    from dataclasses import replace

    cand = load_candidate_config(a.cand)
    if a.planner_borderline:
        lat = replace(cand.constraints.lateral, borderline=a.planner_borderline)
        cand = replace(cand, constraints=replace(cand.constraints, lateral=lat))
    G.update(ds=ds, specs=split_ids(ds, a.split), cand=cand,
             lat=load_candidate_config(a.lateral_config), team=load_candidate_config(TEAM_CAND),
             samples=a.samples,
             vcfg=load_virtual_config(REPO / "config/taehyeon/virtual_data.yaml"),
             hl=load_highlevel_config(REPO / "config/taehyeon/highlevel.yaml"))


def _evaluate(rec, lat_cfg):
    from pac_candidates import lateral as L
    from pac_candidates import lateral_sim as S
    from pac_candidates.geometry import footprint, rotated_dims
    from pac_candidates.pallet_model import PalletModel, box_tolerance

    state, ctx, box, pose = rec
    model = PalletModel(state, ctx, lat_cfg)
    dx, dy, dz = rotated_dims(box.size, pose.yaw)
    uncertain = box.box_id in model.uncertain_ids
    tol = box_tolerance(lat_cfg.uncertainty, uncertain)
    cfg = lat_cfg.constraints.lateral
    t0 = time.perf_counter()
    asm = L.assess(model, box, pose, footprint(pose.x, pose.y, dx, dy), dz, tol, uncertain, lp_all=True)
    t1 = time.perf_counter()
    short = S.short_tilt_test(asm.scene, asm.system, L.worst_directions(asm, cfg.sim_directions), cfg)
    t2 = time.perf_counter()
    ref = S.reference_tilt_test(asm.scene, asm.system, cfg, asm.directions)
    t3 = time.perf_counter()
    keep = ("passed", "reason", "direction", "max_disp_m", "max_tilt_deg")
    return {
        "n": len(asm.scene.bodies), "system": len(asm.system), "on_floor": pose.z < 1e-6, "top_m": pose.z + dz,
        "a_geom": [round(x, 4) for x in asm.a_geom], "a_lp": [round(x, 4) for x in asm.a_lp],
        "single_supported": asm.single_supported, "side_contacts": asm.side_contacts, "mu_limit": asm.mu_limit,
        "lp_status": asm.lp_status, "short": {k: short[k] for k in keep}, "ref": {k: ref[k] for k in keep},
        "t_assess_ms": 1000 * (t1 - t0), "t_short_ms": 1000 * (t2 - t1), "t_ref_ms": 1000 * (t3 - t2),
    }


def _episode(i):
    from pac_highlevel import RulePolicy
    from pac_highlevel.trainer import run_policy
    from pac_highlevel.world import PalletizingWorld
    from virtual_data.highlevel import world_factory

    import random
    from pac_candidates import CandidateBackend

    records = []
    orig = PalletizingWorld._place

    def _place(self, arrival, candidate):
        records.append((self.pallet_index, self.state(), self.context(), arrival.box, candidate.target_pose))
        return orig(self, arrival, candidate)

    PalletizingWorld._place = _place
    try:
        world = world_factory(G["ds"], G["specs"], G["cand"], G["vcfg"], G["hl"], shuffle_seed=12345,
                              vary_pallet=False)(i)
        t = time.perf_counter()
        out = run_policy(world, RulePolicy(G["hl"]))
        wall = time.perf_counter() - t
    finally:
        PalletizingWorld._place = orig
    print(f"episode {i}: planned {len(records)} placements in {wall:.0f} s, evaluating", flush=True)
    rows = []
    rng = random.Random(f"lateral-cal:{i}")
    for step, (pallet, state, ctx, box, pose) in enumerate(records):
        if step and step % 20 == 0:
            print(f"episode {i}: evaluated {step}/{len(records)} placements", flush=True)
        poses = [("chosen", pose)]
        if G["samples"]:
            team = CandidateBackend(ctx, G["team"]).candidate_set(box, state)
            others = [c.target_pose for c in team.valid if c.target_pose != pose]
            poses += [("sample", q) for q in rng.sample(others, min(G["samples"], len(others)))]
        for kind, q in poses:
            row = _evaluate((state, ctx, box, q), G["lat"])
            row.update(episode=i, pallet=pallet, step=step, kind=kind)
            rows.append(row)
    summary = {"episode": i, "pallets_used": out["pallets_used"], "placed": out["placed"], "ng": out["ng"],
               "decisions": out["decisions"], "decision_ms": 1000 * wall / max(1, out["decisions"])}
    return rows, summary


def collect(a):
    _setup(a)
    with mp.get_context("fork").Pool(a.workers, maxtasksperchild=1) as pool, a.out.open("w") as fh:
        for rows, summary in pool.imap_unordered(_episode, range(a.first, a.first + a.episodes), chunksize=1):
            for r in rows:
                fh.write(json.dumps(r, default=float) + "\n")
            fh.write(json.dumps({"summary": summary}) + "\n")
            fh.flush()
            chosen = [r for r in rows if r["kind"] == "chosen"]
            fails = sum(not r["ref"]["passed"] for r in chosen)
            print(f"episode {summary['episode']}: {len(chosen)} placements ({len(rows)} rows), chosen reference "
                  f"fails {fails}, pallets {summary['pallets_used']}, {summary['decision_ms']:.0f} ms/decision",
                  flush=True)


def cascade(row, lo, hi, accel=0.25):
    """Stage and verdict the hard mask would give for this placement."""
    geom = row["a_geom"]
    if min(geom) >= hi:
        return "geom", True
    if row["single_supported"]:
        a = min(min(geom), row["mu_limit"])
        if a >= accel or not row["side_contacts"]:
            return "exact", a >= accel
    lp = min(x for x, g in zip(row["a_lp"], geom) if g < hi)
    if lp >= hi:
        return "lp", True
    if lp < lo:
        return "lp", False
    return "sim", row["short"]["passed"]


def _score(rows, lo, hi):
    oe = ue = sim = lp = 0
    for r in rows:
        stage, ok = cascade(r, lo, hi)
        truth = r["ref"]["passed"]
        oe += ok and not truth
        ue += truth and not ok
        sim += stage == "sim"
        lp += stage in ("lp", "sim")
    return oe, ue, sim, lp


def analyze(a):
    rows, summaries = [], []
    for path in a.paths:
        for line in path.read_text().splitlines():
            d = json.loads(line)
            (summaries.append(d["summary"]) if "summary" in d else rows.append(d))
    # a chosen placement that fails the reference leaves an unstable state:
    # later rows of that pallet start from it and are dropped
    first_bad = {}
    for r in rows:
        if r["kind"] == "chosen" and not r["ref"]["passed"]:
            key = (r["episode"], r["pallet"])
            first_bad[key] = min(first_bad.get(key, r["step"]), r["step"])
    chosen = [r for r in rows if r["kind"] == "chosen"]
    total = len(rows)
    rows = [r for r in rows if r["step"] <= first_bad.get((r["episode"], r["pallet"]), 10 ** 9)]
    n = len(rows)
    fails = sum(not r["ref"]["passed"] for r in rows)
    bad = sum(not r["ref"]["passed"] for r in chosen)
    print(f"chosen placements {len(chosen)}: reference failures {bad} ({100 * bad / max(1, len(chosen)):.2f} %)")
    print(f"rows from stable states {n} (dropped {total - n}): reference failures at "
          f"0.25 g {fails} ({100 * fails / max(1, n):.1f} %)")
    if summaries:
        print(f"episodes {len(summaries)}, pallets/episode {statistics.fmean(s['pallets_used'] for s in summaries):.2f}, "
              f"decision {statistics.fmean(s['decision_ms'] for s in summaries):.0f} ms")
    acc = lambda pred: (sum(p and not r["ref"]["passed"] for p, r in zip(pred, rows)),
                        sum(not p and r["ref"]["passed"] for p, r in zip(pred, rows)))
    accel = 0.25
    for name, pred in (("geom >= 0.25", [min(r["a_geom"]) >= accel for r in rows]),
                       ("LP >= 0.25", [min(r["a_lp"]) >= accel for r in rows]),
                       ("short tilt test", [r["short"]["passed"] for r in rows])):
        oe, ue = acc(pred)
        print(f"  {name:16s} alone: OE {oe} ({100 * oe / n:.2f} %), UE {ue} ({100 * ue / n:.2f} %)")
    print("\nband_low | band_high | OE | UE | to LP | to tilt test")
    grid = []
    for lo100 in range(10, 26):
        for hi100 in range(25, 61):
            lo, hi = lo100 / 100, hi100 / 100
            oe, ue, sim, lp = _score(rows, lo, hi)
            grid.append((lo, hi, oe, ue, sim, lp))
    best = sorted((g for g in grid if g[2] <= a.max_oe * n), key=lambda g: (g[3] + g[4] * a.sim_cost, g[4]))
    for lo, hi, oe, ue, sim, lp in (best[:8] if best else sorted(grid, key=lambda g: g[2])[:8]):
        print(f"{lo:.2f} | {hi:.2f} | {oe} ({100 * oe / n:.2f} %) | {ue} ({100 * ue / n:.2f} %) | "
              f"{100 * lp / n:.0f} % | {100 * sim / n:.1f} %")
    if best:
        lo, hi = best[0][:2]
        print(f"\nper stage at band {lo:.2f} / {hi:.2f}: stage | rows | OE | UE")
        per = {}
        for r in rows:
            stage, ok = cascade(r, lo, hi)
            s = per.setdefault(stage, [0, 0, 0])
            s[0] += 1
            s[1] += ok and not r["ref"]["passed"]
            s[2] += r["ref"]["passed"] and not ok
        for stage, (cnt, oe, ue) in sorted(per.items()):
            print(f"  {stage} | {cnt} | {oe} | {ue}")
    times = {k: statistics.fmean(r[k] for r in rows) for k in ("t_assess_ms", "t_short_ms", "t_ref_ms")}
    print("\nmean time per placement: " + ", ".join(f"{k[2:-3]} {v:.0f} ms" for k, v in times.items()))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("collect")
    c.add_argument("--episodes", type=int, default=10)
    c.add_argument("--first", type=int, default=0)
    c.add_argument("--workers", type=int, default=7)
    c.add_argument("--split", default="test")
    c.add_argument("--data", default=str(DATA))
    c.add_argument("--lateral-config", type=Path, default=LATERAL_CAND, help="lateral parameters for the checks")
    c.add_argument("--cand", type=Path, default=LATERAL_CAND, help="planner candidate config")
    c.add_argument("--samples", type=int, default=4, help="extra steps 1-12 valid candidates per placement")
    c.add_argument("--planner-borderline", choices=("simulate", "reject", "accept"),
                   help="override the planner's borderline policy (reject: no simulation while planning)")
    c.add_argument("--out", type=Path, required=True)
    z = sub.add_parser("analyze")
    z.add_argument("paths", type=Path, nargs="+")
    z.add_argument("--max-oe", type=float, default=0.005, help="allowed missed collapses (share of placements)")
    z.add_argument("--sim-cost", type=float, default=0.2, help="weight of a tilt-test call vs one needless rejection")
    a = ap.parse_args()
    collect(a) if a.cmd == "collect" else analyze(a)
