#!/usr/bin/env python3
"""Rebuild the pallets made by the 1 -> 8 runtime loop in jaesung's PyBullet
simulator, box by box in placement order, with TRUE sizes and TRUE poses.

After each drop the world is simulated; a pallet fails if any box moves more
than ``--max-shift`` or tilts more than ``--max-tilt`` (same criteria as the
5-2 physics cross-check). Optionally sends one pallet to a running AHEAD
live simulator (``/api/place``, pallet-centre origin, box centre) to look at
it in 3D:

    python tools/runtime/scripts/physics_replay.py --dataset tools/highlevel/output/dataset80 \
        --split test --report docs/taehyeon/reports/runtime_physics.json
    # 3D view: start run_ahead_simulator.py (jaesung / pac2026-ahead), then
    python tools/runtime/scripts/physics_replay.py ... --scenarios 1 --live http://127.0.0.1:4173 --live-pallet 0
"""

import argparse
import json
import math
from pathlib import Path
import sys
import time
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "highlevel" / "scripts"))
from _common import REPO, add_common_args, load_all, load_lookahead  # noqa: E402
import team_paths  # noqa: E402

from pac_common import Size3D  # noqa: E402
from pac_highlevel import load_policy  # noqa: E402
from pac_robot_check import RobotFeasibility, load_robot_check_config  # noqa: E402
from pac_runtime import RuntimeLoop, load_runtime_config  # noqa: E402
from virtual_data.highlevel import family_indices, split_ids  # noqa: E402
from virtual_data.runtime_cell import runtime_cell  # noqa: E402


def centre(box, pallet):
    """pallet frame min corner -> simulator frame (pallet centre origin) box centre."""
    x, y, z, yaw = box["pose"]
    sx, sy, sz = box["size"]
    if int(round(yaw / (math.pi / 2))) % 2:
        sx, sy = sy, sx
    return [x + sx / 2 - pallet[0] / 2, y + sy / 2 - pallet[1] / 2, z + sz / 2]


def replay(sim, pallet, max_shift, max_tilt, step_s=0.5):
    import pybullet as p
    from pac_simulation.ahead_sim.world import BulletPalletWorld

    cfg = sim.SimulatorConfig(pallet=sim.PalletConfig(collision_model="solid", length_m=pallet["pallet_size"][0],
                                                      width_m=pallet["pallet_size"][1]))
    world = BulletPalletWorld(cfg)
    hz = world.config.physics.physics_hz
    placed, worst, failed_at = {}, 0.0, None
    try:
        for k, b in enumerate(pallet["layout"]):
            c = centre(b, pallet["pallet_size"])
            body = world.add_box(sim.BoxSpec(box_id=b["box_id"], size_m=tuple(b["size"]),
                                             mass_kg=max(0.05, b["mass_kg"] or 1.0), target_position_m=tuple(c),
                                             yaw_rad=b["pose"][3], source="taehyeon_runtime_replay"))
            placed[b["box_id"]] = (body, c)
            world.step(int(step_s * hz))
            for bid, (bd, target) in placed.items():
                pos, quat = p.getBasePositionAndOrientation(bd, physicsClientId=world.client_id)
                roll, pitch, _ = p.getEulerFromQuaternion(quat)
                shift = math.dist(pos, target)
                tilt = math.degrees(max(abs(roll), abs(pitch)))
                worst = max(worst, shift)
                if (shift > max_shift or tilt > max_tilt) and failed_at is None:
                    failed_at = {"step": k, "box": bid, "shift_m": round(shift, 4), "tilt_deg": round(tilt, 2)}
        world.step(int(2.0 * hz))
        for bid, (bd, target) in placed.items():
            pos, _ = p.getBasePositionAndOrientation(bd, physicsClientId=world.client_id)
            worst = max(worst, math.dist(pos, target))
    finally:
        world.close()
    return {"pallet_id": pallet["pallet_id"], "boxes": len(pallet["layout"]), "stable": failed_at is None,
            "worst_shift_m": round(worst, 4), "failed_at": failed_at}


def post(url, path, payload):
    req = urllib.request.Request(url.rstrip("/") + path, data=json.dumps(payload).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=10) as res:
        return json.loads(res.read().decode())


def main():
    parser = argparse.ArgumentParser()
    add_common_args(parser)
    parser.add_argument("--split", default="test")
    parser.add_argument("--scenarios", type=int, default=0, help="first N scenarios (0 = all)")
    parser.add_argument("--max-shift", type=float, default=0.01)
    parser.add_argument("--max-tilt", type=float, default=2.0)
    parser.add_argument("--live", help="URL of a running AHEAD live simulator (3D view)")
    parser.add_argument("--live-pallet", type=int, default=0)
    parser.add_argument("--live-delay", type=float, default=0.6)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    dataset, cand, vcfg, hl = load_all(args)
    rt = load_runtime_config(REPO / "config/taehyeon/runtime.yaml")
    la = load_lookahead(args)
    robot = RobotFeasibility(load_robot_check_config(REPO / "config/taehyeon/robot_check.yaml"))
    specs = split_ids(dataset, args.split)
    if args.scenarios:
        specs = specs[: args.scenarios]
    fam = family_indices(dataset)
    pos = {s.scenario_id: i for i, s in enumerate(dataset.scenarios)}
    pallets = []
    for spec in specs:
        cell = runtime_cell(spec, dataset, cand, vcfg, family_index=fam[spec.scenario_id],
                            scenario_index=pos[spec.scenario_id])
        out = RuntimeLoop(cell, cand, hl, rt, robot, load_policy("lookahead", la, config=hl), log_events=False).run()
        for p in out["pallet_list"]:
            pallets.append({"scenario": spec.scenario_id, **p})
    if args.live:
        pal = pallets[args.live_pallet]
        post(args.live, "/api/reset", {})
        for b in pal["layout"]:
            post(args.live, "/api/place", {"id": b["box_id"], "size_m": b["size"], "mass_kg": b["mass_kg"] or 1.0,
                                          "target_position_m": centre(b, pal["pallet_size"]),
                                          "yaw_rad": b["pose"][3]})
            time.sleep(args.live_delay)
        print("sent", pal["pallet_id"], len(pal["layout"]), "boxes to", args.live)
        return
    team_paths.add_optional("pac_simulation")
    import pac_simulation.ahead_sim as sim

    results = [{"scenario": p["scenario"], **replay(sim, p, args.max_shift, args.max_tilt)} for p in pallets]
    stable = sum(r["stable"] for r in results)
    print(f"stable pallets {stable}/{len(results)}, boxes {sum(r['boxes'] for r in results)}")
    for r in results:
        if not r["stable"]:
            print("  unstable", r)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps({"split": args.split, "max_shift_m": args.max_shift,
                                           "max_tilt_deg": args.max_tilt, "pallets": results,
                                           "layouts": pallets}, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
