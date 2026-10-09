"""Evaluate look-ahead variants vs the team baseline on the team world (1.1 x 1.1 pallet).
Metrics: pallets, pallet equivalents, robot time (assumed), decision time, interlock
ratio, and boxes displaced by 0.2 g lateral pulses in PyBullet (unsecured load).

    python3 run_eval.py VARIANT [VARIANT ...] --episodes 18 --out results.json
VARIANT: team | fast | la_k{K}  (look-ahead with K weighed boxes known, m=2 upstream)
"""
import argparse, json, multiprocessing as mp, statistics, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path.home() / "AHEAD/audit_20261009/08_prototype"))
import lookahead as LA
import tune_phys
import shock_test
from pac_candidates import load_candidate_config
from pac_highlevel import load_highlevel_config, RulePolicy
from pac_highlevel.rollout import run_policy
from pac_highlevel.world import PalletizingWorld
from virtual_data import load_virtual_config
from virtual_data.scenario_source import load_dataset
from virtual_data.highlevel import split_ids, world_factory

REPO = LA.REPO
DATA = Path.home() / "AHEAD/audit_20261009/03_planner/highlevel_work/dataset80_x10"
CAND = Path.home() / "AHEAD/audit_20261009/07_ai_role/sweep/configs/perbox.yaml"
G = {}


class IC:
    """Same rule as 08_prototype/interlock_check.py (copied: that file runs on import)."""
    @staticmethod
    def supporters(box, pose, layout):
        from pac_candidates.geometry import rotated_dims
        dx, dy, _ = rotated_dims(box.size, pose.yaw)
        out = []
        for b, p in layout:
            if b is box:
                continue
            bx, by, bz = rotated_dims(b.size, p.yaw)
            if abs(p.z + bz - pose.z) > 0.004:
                continue
            ox = min(pose.x + dx, p.x + bx) - max(pose.x, p.x)
            oy = min(pose.y + dy, p.y + by) - max(pose.y, p.y)
            if ox > 0.005 and oy > 0.005:
                out.append(ox * oy / (dx * dy))
        return out

def setup(split):
    ds = load_dataset(DATA)
    G.update(ds=ds, cand=load_candidate_config(CAND), vcfg=load_virtual_config(REPO / "config/taehyeon/virtual_data.yaml"),
             hl=load_highlevel_config(REPO / "config/taehyeon/highlevel.yaml"), specs=split_ids(ds, split),
             ranges={k: (r.weight_min_kg, r.weight_max_kg) for k, r in ds.sku_ranges.items()})

def make_planner(variant, i):
    if variant == "team":
        return None, None
    p = LA.Params(seed=i)
    if variant == "fast":
        p.lookahead = False
    elif variant == "fast_v1":
        p.lookahead = False
    elif variant.startswith("la_k"):
        p.preview_k = int(variant[4:])
        p.horizon = max(4, p.preview_k + 2)
    pl = LA.Planner(p, G["ranges"])
    return pl, pl.policy()

def episode(job):
    variant, i = job
    finals = []
    oc, of = PalletizingWorld._close_pallet, PalletizingWorld._finish
    def snap(self):
        if self.placed: finals.append([(pb, pb.pose) for pb in self.placed])
    PalletizingWorld._close_pallet = lambda self, *a, **k: (snap(self), oc(self, *a, **k))[1]
    PalletizingWorld._finish = lambda self, *a, **k: (snap(self), of(self, *a, **k))[1]
    try:
        placer, policy = make_planner(variant, i)
        make = world_factory(G["ds"], G["specs"], G["cand"], G["vcfg"], G["hl"], shuffle_seed=12345,
                             vary_pallet=False, placer=placer)
        world = make(i)
        if placer is not None:
            placer.world = world
        chooser = policy or RulePolicy(G["hl"])
        t = time.perf_counter()
        out = run_policy(world, chooser)
        wall = time.perf_counter() - t
    finally:
        PalletizingWorld._close_pallet, PalletizingWorld._finish = oc, of
    up = bonded = 0
    for layout in finals:
        for b, p in layout:
            if p.z < 1e-6: continue
            up += 1; bonded += len(IC.supporters(b, p, layout)) >= 2
    shock, rebuild_err = [], 0
    if G.get("shock"):
        for layout in finals:
            res = shock_test.shock_test(layout)
            if res["passed_g"] is None:
                rebuild_err += 1
            else:
                shock.append(res["passed_g"])
    moved = 0
    for layout in finals:
        try:
            moved += tune_phys.fast_moved(sorted(layout, key=lambda t: (round(t[1].z, 4), t[1].y, t[1].x)))
        except ValueError:
            moved += len(layout)
    return {"variant": variant, "episode": i, "pallets_used": out["pallets_used"],
            "pallet_equivalents": out["pallet_equivalents"], "placed": out["placed"], "ng": out["ng"],
            "time_s": out["time_s"], "decision_ms": 1000 * wall / max(1, out["decisions"]),
            "boxes_above_deck": up, "bonded": bonded, "lateral_moved": moved, "shock_passed_g": shock, "shock_rebuild_errors": rebuild_err,
            "counts": dict(out.get("counts", {}))}

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("variants", nargs="+")
    ap.add_argument("--episodes", type=int, default=18)
    ap.add_argument("--workers", type=int, default=20)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--shock", action="store_true", help="staircase shock test per finished pallet")
    a = ap.parse_args()
    setup("test")
    G["shock"] = a.shock
    jobs = [(v, i) for v in a.variants for i in range(a.episodes)]
    with mp.get_context("fork").Pool(a.workers) as pool:
        rows = pool.map(episode, jobs, chunksize=1)
    summ = {}
    for v in a.variants:
        r = [x for x in rows if x["variant"] == v]
        up = sum(x["boxes_above_deck"] for x in r)
        summ[v] = {k: round(statistics.fmean(x[k] for x in r), 3) for k in
                   ("pallets_used", "pallet_equivalents", "placed", "ng", "time_s", "decision_ms", "lateral_moved")}
        summ[v]["interlock_ratio"] = round(sum(x["bonded"] for x in r) / max(1, up), 3)
        sg = [g for x in r for g in x["shock_passed_g"]]
        if sg:
            summ[v]["shock_g_median"] = statistics.median(sg)
            summ[v]["shock_g_min"] = min(sg)
            summ[v]["share_pallets_ge_0.3g"] = round(sum(g >= 0.3 for g in sg) / len(sg), 3)
            summ[v]["share_pallets_ge_0.5g"] = round(sum(g >= 0.5 for g in sg) / len(sg), 3)
            summ[v]["shock_pallets"] = len(sg)
            summ[v]["shock_rebuild_errors"] = sum(x["shock_rebuild_errors"] for x in r)
        print(v, json.dumps(summ[v]), flush=True)
    a.out.write_text(json.dumps({"summary": summ, "rows": rows}, indent=1, default=float))
