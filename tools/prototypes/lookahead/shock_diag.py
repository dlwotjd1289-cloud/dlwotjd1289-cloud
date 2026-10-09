"""Which boxes fail the shock test? Re-run a few episodes and describe moved boxes."""
import json, sys, math, statistics
from collections import Counter
from pathlib import Path
import numpy as np, pybullet as p
sys.path.insert(0, str(Path(__file__).parent))
import run_eval as R, shock_test as ST, lookahead as LA
from pac_candidates.geometry import rotated_dims
from pac_highlevel.trainer import run_policy
from pac_highlevel.world import PalletizingWorld
from pac_highlevel import RulePolicy
from virtual_data.highlevel import world_factory

def finals_of(variant, i):
    finals = []
    oc, of = PalletizingWorld._close_pallet, PalletizingWorld._finish
    def snap(self):
        if self.placed: finals.append([(pb, pb.pose) for pb in self.placed])
    PalletizingWorld._close_pallet = lambda self, *a, **k: (snap(self), oc(self, *a, **k))[1]
    PalletizingWorld._finish = lambda self, *a, **k: (snap(self), of(self, *a, **k))[1]
    try:
        placer, policy = R.make_planner(variant, i)
        w = world_factory(R.G["ds"], R.G["specs"], R.G["cand"], R.G["vcfg"], R.G["hl"], shuffle_seed=12345, vary_pallet=False, placer=placer)(i)
        if placer: placer.world = w
        run_policy(w, policy or RulePolicy(R.G["hl"]))
    finally:
        PalletizingWorld._close_pallet, PalletizingWorld._finish = oc, of
    return finals

def diag(layout):
    cfg, _ = ST.pc.load_sim_config(ST.pc.REPO / "config/ahead_simulator.yaml")
    sim = ST.pc.AheadLiveSimulator(cfg); hz = cfg.physics.physics_hz; L, W = cfg.pallet.length_m, cfg.pallet.width_m
    order = sorted(layout, key=lambda t: (round(t[1].z, 4), t[1].y, t[1].x)); info = {}
    for k, (box, pose) in enumerate(order):
        dx, dy, dz = rotated_dims(box.size, pose.yaw); bid = f"b{k:03d}"
        sim.place_box(ST.pc.BoxSpec(bid, (box.size.x, box.size.y, box.size.z), box.weight_kg, (pose.x+dx/2-L/2, pose.y+dy/2-W/2, pose.z+dz/2), pose.yaw, source="d"))
        sup = R.IC.supporters(box, pose, layout)
        info[bid] = {"z": round(pose.z, 3), "top": round(pose.z+dz, 3), "h": round(dz, 2), "kg": round(box.weight_kg, 1), "n_sup": len(sup), "on_floor": pose.z < 1e-6}
        sim.step(int(0.1*hz))
    sim.step(int(0.5*hz)); ref = ST._poses(sim, list(info)); cid, g = sim.world.client_id, cfg.physics.gravity_m_s2
    for a in ST.LEVELS:
        for ax, ay in ((1,0),(-1,0),(0,1),(0,-1)):
            p.setGravity(ax*a*g, ay*a*g, -g, physicsClientId=cid); sim.step(int(0.4*hz)); p.setGravity(0,0,-g, physicsClientId=cid); sim.step(int(0.3*hz))
        for gz, dur in ((1.6,0.15),(0.4,0.15)):
            p.setGravity(0,0,-gz*g, physicsClientId=cid); sim.step(int(dur*hz))
        p.setGravity(0,0,-g, physicsClientId=cid); sim.step(int(0.3*hz))
        now = ST._poses(sim, list(info))
        bad = [k for k,(pos,_) in ref.items() if np.linalg.norm(now[k][0]-pos) > 0.02 or math.degrees(max(abs(now[k][1][0]), abs(now[k][1][1]))) > 10]
        if bad:
            sim.close(); return a, [info[k] for k in bad], len(info)
    sim.close(); return None, [], len(info)

R.setup("test")
out = {}
for variant in ("team", "la_k3"):
    fails, firstfail = [], Counter()
    for i in range(6):
        for layout in finals_of(variant, i):
            a, bad, n = diag(layout)
            if a is not None and a <= 0.3:
                firstfail[a] += 1
                fails.extend(bad)
    agg = {"failed_pallets_by_level": dict(firstfail), "moved_boxes": len(fails)}
    if fails:
        agg["share_on_floor"] = round(sum(f["on_floor"] for f in fails)/len(fails), 2)
        agg["share_single_support"] = round(sum(f["n_sup"] == 1 for f in fails)/len(fails), 2)
        agg["median_top_m"] = statistics.median(f["top"] for f in fails)
        agg["median_kg"] = statistics.median(f["kg"] for f in fails)
        agg["examples"] = fails[:6]
    out[variant] = agg
    print(variant, json.dumps(agg, ensure_ascii=False), flush=True)
Path("shock_diag.json").write_text(json.dumps(out, indent=1, ensure_ascii=False))
