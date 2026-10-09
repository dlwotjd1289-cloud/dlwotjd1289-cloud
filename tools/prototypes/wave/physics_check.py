"""Rebuild every closed pallet of WAVE vs team-DBLF episodes in the team PyBullet
simulator (pac_simulation.ahead_sim), box by box, then apply lateral pulses.

Static: after each placement settle 1.0 s; record max drift of earlier boxes,
tilt, and Bullet moving boxes. Dynamic (unsecured load, no stretch wrap):
gravity tilted by a = 0.2 g for 0.5 s in +x, -x, +y, -y; count boxes displaced
> 20 mm. Only 1.1 x 1.1 pallets (the simulator pallet), vary_pallet off.
"""
import json, math, multiprocessing as mp, statistics, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import tune, wave  # noqa: E402
import pybullet as p  # noqa: E402
from pac_highlevel import RulePolicy  # noqa: E402
from pac_highlevel.trainer import run_policy  # noqa: E402
from pac_highlevel.world import PalletizingWorld  # noqa: E402
from pac_candidates.geometry import rotated_dims  # noqa: E402
from virtual_data.highlevel import world_factory  # noqa: E402

REPO = wave.REPO
sys.path.insert(0, str(REPO / "scripts"))
from pac_simulation.ahead_sim import AheadLiveSimulator, BoxSpec  # noqa: E402
from pac_simulation.ahead_sim.config import load_config as load_sim_config  # noqa: E402

LAT_G = 0.2


def record_episode(weights, policy, i):
    """Final layout of every pallet (after any PARTIAL_REPACK moves), captured
    when the world closes or finishes a pallet."""
    finals = []
    orig_close, orig_finish = PalletizingWorld._close_pallet, PalletizingWorld._finish

    def snap(self):
        if self.placed:
            finals.append([(pb, pb.pose) for pb in self.placed])

    def _close(self, *a, **k):
        snap(self)
        return orig_close(self, *a, **k)

    def _finish(self, *a, **k):
        snap(self)
        return orig_finish(self, *a, **k)

    PalletizingWorld._close_pallet, PalletizingWorld._finish = _close, _finish
    try:
        nf = len(wave.FEATURES)
        placer = None if weights is None else wave.WavePlacer(weights[:nf], tune.G["ranges"])
        make = world_factory(tune.G["ds"], tune.G["specs"], tune.G["cand"], tune.G["vcfg"], tune.G["hl"],
                             shuffle_seed=12345, vary_pallet=False, placer=placer)
        chooser = wave.WavePolicy(placer, weights[nf], weights[nf + 1]) if policy == "wave" else RulePolicy(tune.G["hl"])
        out = run_policy(make(i), chooser)
    finally:
        PalletizingWorld._close_pallet, PalletizingWorld._finish = orig_close, orig_finish
    return out, finals


def simulate_pallet(boxes):
    cfg, _ = load_sim_config(REPO / "config/ahead_simulator.yaml")
    sim = AheadLiveSimulator(cfg)
    hz = cfg.physics.physics_hz
    L, W = cfg.pallet.length_m, cfg.pallet.width_m
    placed, max_drift, max_tilt = {}, 0.0, 0.0
    for k, (box, pose) in enumerate(boxes):
        dx, dy, dz = rotated_dims(box.size, pose.yaw)
        # pallet corner frame (team planner) -> simulator frame (pallet centre at origin, deck top z = 0)
        cx, cy, cz = pose.x + dx / 2 - L / 2, pose.y + dy / 2 - W / 2, pose.z + dz / 2
        bid = f"b{k:03d}"
        sim.place_box(BoxSpec(bid, (box.size.x, box.size.y, box.size.z), box.weight_kg, (cx, cy, cz), pose.yaw,
                              source="wave_check"))
        placed[bid] = np.array([cx, cy, cz])
        sim.step(int(1.0 * hz))
        snap = sim.snapshot()
        for b in snap["boxes"]:
            if b["id"] in placed:
                max_drift = max(max_drift, float(np.linalg.norm(np.array(b["position_m"]) - placed[b["id"]])))
                r, pt, _ = b["euler_rad"]
                max_tilt = max(max_tilt, math.degrees(max(abs(r), abs(pt))))
    static = {"n": len(boxes), "max_drift_mm": 1000 * max_drift, "max_tilt_deg": max_tilt}
    before = {b["id"]: np.array(b["position_m"]) for b in sim.snapshot()["boxes"] if b["id"] in placed}
    cid, g = sim.world.client_id, cfg.physics.gravity_m_s2
    moved = set()
    for ax, ay in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        p.setGravity(ax * LAT_G * g, ay * LAT_G * g, -g, physicsClientId=cid)
        sim.step(int(0.5 * hz))
        p.setGravity(0, 0, -g, physicsClientId=cid)
        sim.step(int(0.5 * hz))
    after = {b["id"]: np.array(b["position_m"]) for b in sim.snapshot()["boxes"] if b["id"] in placed}
    for bid, pos in before.items():
        if bid in after and np.linalg.norm(after[bid] - pos) > 0.02:
            moved.add(bid)
    sim.close()
    static["lateral_moved_gt20mm"] = len(moved)
    return static


def job(args):
    weights, policy, i = args
    out, finals = record_episode(weights, policy, i)
    # rebuild bottom-up (final layout; the true sequence may differ after repacks)
    res = [simulate_pallet(sorted(v, key=lambda t: (round(t[1].z, 4), t[1].y, t[1].x))) for v in finals]
    return {"episode": i, "pallets": len(res), "boxes": sum(r["n"] for r in res),
            "max_drift_mm": max(r["max_drift_mm"] for r in res), "max_tilt_deg": max(r["max_tilt_deg"] for r in res),
            "pallets_drift_gt5mm": sum(r["max_drift_mm"] > 5 for r in res),
            "lateral_moved_boxes": sum(r["lateral_moved_gt20mm"] for r in res), "per_pallet": res}


if __name__ == "__main__":
    C = Path.home() / "AHEAD/audit_20261009/07_ai_role/sweep/configs"
    tune.setup(C / "perbox.yaml", "test")
    wv = json.loads((Path(__file__).parent / "tuned_perbox_wave.json").read_text())["mean"]
    variants = {"team_dblf_rule": (None, "rule"), "wave_full": (wv, "wave")}
    eps = list(range(len(tune.G["specs"])))          # 9 test scenarios, 1.1 x 1.1 pallet
    out = {}
    with mp.get_context("fork").Pool(18) as pool:
        for name, (w, pol) in variants.items():
            rows = pool.map(job, [(w, pol, i) for i in eps], chunksize=1)
            s = {"episodes": len(rows), "pallets": sum(r["pallets"] for r in rows), "boxes": sum(r["boxes"] for r in rows),
                 "pallets_drift_gt5mm": sum(r["pallets_drift_gt5mm"] for r in rows),
                 "worst_drift_mm": max(r["max_drift_mm"] for r in rows),
                 "worst_tilt_deg": max(r["max_tilt_deg"] for r in rows),
                 "lateral_0.2g_moved_boxes": sum(r["lateral_moved_boxes"] for r in rows)}
            out[name] = {"summary": s, "episodes": rows}
            print(name, json.dumps(s), flush=True)
    (Path(__file__).parent / "physics_check.json").write_text(json.dumps(out, indent=1, default=float))
