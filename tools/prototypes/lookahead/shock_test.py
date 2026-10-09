"""Shock robustness of finished pallets in PyBullet (team simulator, unsecured load).

Staircase: lateral levels 0.1 .. 0.6 g. At each level gravity is tilted by a in
+x, -x, +y, -y for 0.4 s each (settle 0.3 s between); then a vertical bump
(gravity x 1.6 for 0.15 s, then x 0.4 for 0.15 s) imitating a forklift lift /
set-down. A level is passed when no box moved > 20 mm from its pre-test
position and no box tilted > 10 deg. Result per pallet: highest level passed.

No stretch wrap is modelled, so this is stricter than EUMOS 40509 (0.5 g with
the real load securing).
"""
import math
import sys
from pathlib import Path

import numpy as np
import pybullet as p

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "wave"))
import physics_check as pc  # noqa: E402  (team simulator imports)
from pac_candidates.geometry import rotated_dims  # noqa: E402

LEVELS = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
MOVE_TOL_M = 0.02
TILT_TOL_DEG = 10.0


def _poses(sim, ids):
    snap = sim.snapshot()
    return {b["id"]: (np.array(b["position_m"]), b["euler_rad"]) for b in snap["boxes"] if b["id"] in ids}


def shock_test(layout):
    """layout: [(PlacedBox, pose)] of one pallet. Returns dict with the highest g passed."""
    cfg, _ = pc.load_sim_config(pc.REPO / "config/ahead_simulator.yaml")
    sim = pc.AheadLiveSimulator(cfg)
    hz = cfg.physics.physics_hz
    L, W = cfg.pallet.length_m, cfg.pallet.width_m
    ids = []
    for k, (box, pose) in enumerate(sorted(layout, key=lambda t: (round(t[1].z, 4), t[1].y, t[1].x))):
        dx, dy, dz = rotated_dims(box.size, pose.yaw)
        bid = f"b{k:03d}"
        centre = [pose.x + dx / 2 - L / 2, pose.y + dy / 2 - W / 2, pose.z + dz / 2]
        for lift in (0.0, 0.015, 0.03):          # earlier boxes may have settled a few mm
            try:
                sim.place_box(pc.BoxSpec(bid, (box.size.x, box.size.y, box.size.z), box.weight_kg,
                                         (centre[0], centre[1], centre[2] + lift), pose.yaw, source="shock"))
                break
            except ValueError as exc:
                err = str(exc)
        else:
            sim.close()
            return {"boxes": len(ids), "passed_g": None, "failed_at_g": None, "moved": 0, "tilted": 0,
                    "rebuild_error": err}
        ids.append(bid)
        sim.step(int(0.1 * hz))
    sim.step(int(0.5 * hz))
    ref = _poses(sim, ids)
    cid, g = sim.world.client_id, cfg.physics.gravity_m_s2
    passed = 0.0
    for a in LEVELS:
        for ax, ay in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            p.setGravity(ax * a * g, ay * a * g, -g, physicsClientId=cid)
            sim.step(int(0.4 * hz))
            p.setGravity(0, 0, -g, physicsClientId=cid)
            sim.step(int(0.3 * hz))
        for gz, dur in ((1.6, 0.15), (0.4, 0.15)):
            p.setGravity(0, 0, -gz * g, physicsClientId=cid)
            sim.step(int(dur * hz))
        p.setGravity(0, 0, -g, physicsClientId=cid)
        sim.step(int(0.3 * hz))
        now = _poses(sim, ids)
        moved = sum(1 for k, (pos, _) in ref.items() if k in now and np.linalg.norm(now[k][0] - pos) > MOVE_TOL_M)
        tilted = sum(1 for k, (_, e) in now.items()
                     if math.degrees(max(abs(e[0]), abs(e[1]))) > TILT_TOL_DEG)
        if moved or tilted:
            sim.close()
            return {"boxes": len(ids), "passed_g": passed, "failed_at_g": a, "moved": moved, "tilted": tilted}
        passed = a
    sim.close()
    return {"boxes": len(ids), "passed_g": passed, "failed_at_g": None, "moved": 0, "tilted": 0}
