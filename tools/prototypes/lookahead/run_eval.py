"""Evaluate look-ahead variants vs the team baseline on the team world (1.1 x 1.1 pallet).
Metrics: pallets, pallet equivalents, robot time (assumed), decision time, interlock
ratio, and boxes displaced by 0.2 g lateral pulses in PyBullet (unsecured load).

    python3 run_eval.py VARIANT [VARIANT ...] --episodes 18 --out results.json
VARIANT: team | fast | la_k{K}  (look-ahead with K weighed boxes known, m=2 upstream)
"""
import argparse, json, multiprocessing as mp, statistics, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "wave"))
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
import os
DATA = Path(os.environ.get("LA_DATA", str(Path(__file__).resolve().parent / "data/dataset80_x10")))
CAND = Path(__file__).resolve().parent / "configs/perbox.yaml"
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

def parse_variant(variant):
    """'la_k3;a_target_g=0.2;hl.repack.min_gain=0.05' -> (base, planner overrides, highlevel overrides)."""
    base, *kvs = variant.split(";")
    po, ho = {}, {}
    for kv in kvs:
        k, v = kv.split("=")
        try:
            val = float(v) if ("." in v or "e" in v) else int(v)
        except ValueError:
            val = v
        (ho if k.startswith("hl.") else po)[k[3:] if k.startswith("hl.") else k] = val
    return base, po, ho


def hl_for(ho):
    from dataclasses import replace as dc_replace
    hl = G["hl"]
    for path, val in ho.items():
        sec, field = path.split(".")
        hl = dc_replace(hl, **{sec: dc_replace(getattr(hl, sec), **{field: val})})
    return hl


def make_planner(variant, i):
    base, po, _ = parse_variant(variant)
    if base == "team":
        return None, None
    p = LA.Params(seed=i)
    if base == "fast":
        p.lookahead = False
    elif base.startswith("la_k"):
        p.preview_k = int(base[4:])
        p.horizon = max(4, p.preview_k + 2)
    for k, v in po.items():
        if k in ("cap", "cand"):
            continue
        setattr(p, k, v)
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
        _, po_, ho_ = parse_variant(variant)
        hl = hl_for(ho_)
        import virtual_data.highlevel as VH
        from pac_candidates import load_candidate_config
        cand_cfg = G["cand"]
        if "cand" in po_:
            cand_cfg = load_candidate_config(Path(__file__).resolve().parent / "configs" / f"{po_['cand']}.yaml")
        if "cap" in po_:
            sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "site_params"))
            import capacity as CAP
            VH.build_catalog = CAP.patched_build_catalog(po_["cap"])
        make = world_factory(G["ds"], G["specs"], cand_cfg, G["vcfg"], hl, shuffle_seed=12345,
                             vary_pallet=False, placer=placer)
        world = make(i)
        if placer is not None:
            placer.world = world
        chooser = policy or RulePolicy(hl)
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
    # true crush check against the hidden carton strength of this scenario
    from virtual_data.episode import select_strength
    from virtual_data.strength import true_overloads
    from pac_common import SystemState, PalletState, InventoryState, Size3D
    sid = world.arrivals[0].box.box_id.split("-")[0] if world.arrivals else None
    spec = next((sp for sp in G["ds"].scenarios if sp.scenario_id == sid), None)
    over_n, over_max = 0, 0.0
    if spec is not None:
        strength = select_strength(spec, G["vcfg"], [s_.scenario_id for s_ in G["ds"].scenarios].index(sid))
        if strength is not None:
            for layout in finals:
                st_ = SystemState(0, 0.0, PalletState("P", world.pallet_size, tuple(b for b, _ in layout)), InventoryState({}, {}))
                ov, mx = true_overloads(st_, strength, cand_cfg)
                over_n += len(ov); over_max = max(over_max, mx)
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
            "true_overloaded_boxes": over_n, "true_max_load_ratio": over_max,
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
    # incremental + resumable: every finished episode is appended to <out>.jsonl
    part = a.out.with_suffix(".jsonl")
    done_rows = []
    if part.exists():
        for line in part.read_text().splitlines():
            if line.strip():
                done_rows.append(json.loads(line))
    done = {(r["variant"], r["episode"]) for r in done_rows}
    jobs = [(v, i) for v in a.variants for i in range(a.episodes) if (v, i) not in done]
    print(f"resume: {len(done)} done, {len(jobs)} to run", flush=True)
    rows = [r for r in done_rows if r["variant"] in a.variants]
    with mp.get_context("fork").Pool(a.workers, maxtasksperchild=1) as pool, part.open("a") as fh:
        for r in pool.imap_unordered(episode, jobs, chunksize=1):
            fh.write(json.dumps(r, default=float) + "\n"); fh.flush()
            rows.append(r)
    summ = {}
    for v in a.variants:
        r = [x for x in rows if x["variant"] == v]
        up = sum(x["boxes_above_deck"] for x in r)
        summ[v] = {k: round(statistics.fmean(x[k] for x in r), 3) for k in
                   ("pallets_used", "pallet_equivalents", "placed", "ng", "time_s", "decision_ms", "lateral_moved")}
        summ[v]["interlock_ratio"] = round(sum(x["bonded"] for x in r) / max(1, up), 3)
        summ[v]["true_overloaded_per_ep"] = round(statistics.fmean(x.get("true_overloaded_boxes", 0) for x in r), 3)
        summ[v]["true_max_load_ratio"] = round(max(x.get("true_max_load_ratio", 0.0) for x in r), 3)
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
