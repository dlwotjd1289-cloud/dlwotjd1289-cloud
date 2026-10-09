#!/usr/bin/env python3
"""Pallet change in the Gazebo V4.4 workcell (simulated AGV / forklift / operator, not a robot move).

The full pallet is taken out of the cell: a copy of the pallet model is created in the outbound
lane and every box of the finished pallet is moved onto it (same relative pose, bottom layer
first); pallet_main stays in place as the new, empty pallet.

  python3 scripts/pallet_swap_v44.py --state RUN/pallet_1_state.json --index 1
"""
import argparse
import json
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
from moveit_pick_place_v44 import gz_world_poses  # noqa: E402

WORLD = "ahead_workcell_v4_2_physical_scale"
WORLD_FILE = os.path.join(ROOT, "ros2_ws/src/pac_simulation/worlds/ahead_workcell_v4_4_suction.sdf")
PALLET_MAIN = (0.0, 1.20)
OUTBOUND = [(2.70, 1.20), (2.70, -0.15), (2.70, -1.50)]   # finished pallets (floor x <= 3.5)


def pallet_sdf(name: str) -> str:
    s = open(WORLD_FILE).read()
    a = s.index('<model name="pallet_main">')
    b = s.index("</model>", a) + len("</model>")
    m = s[a:b].replace('<model name="pallet_main">', f'<model name="{name}">', 1)
    m = re.sub(r"<pose>[^<]*</pose>", "<pose>0 0 0 0 0 0</pose>", m, count=1)   # model pose (first)
    return f'<?xml version="1.0"?>\n<sdf version="1.9">\n{m}\n</sdf>\n'


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", required=True, help="planner state of the finished pallet")
    ap.add_argument("--index", type=int, required=True, help="number of the finished pallet (1, 2, ...)")
    a = ap.parse_args()
    boxes = [b["box_id"] for b in json.load(open(a.state))["placed"]]
    ox, oy = OUTBOUND[(a.index - 1) % len(OUTBOUND)]
    name = f"pallet_done_{a.index}"
    path = f"/tmp/{name}.sdf"
    open(path, "w").write(pallet_sdf(name))
    create = os.path.join(subprocess.run(["ros2", "pkg", "prefix", "ros_gz_sim"], capture_output=True, text=True).stdout.strip(),
                          "lib/ros_gz_sim/create")
    subprocess.run([create, "-name", name, "-file", path, "-x", str(ox), "-y", str(oy), "-z", "0"],
                   capture_output=True, timeout=30)
    snap = gz_world_poses()
    dx, dy = ox - PALLET_MAIN[0], oy - PALLET_MAIN[1]
    moved = 0
    for box in sorted((b for b in boxes if b in snap), key=lambda b: snap[b][0][2]):
        (x, y, z), (qx, qy, qz, qw) = snap[box]
        req = (f'name: "{box}" position {{x: {x + dx:.4f} y: {y + dy:.4f} z: {z + 0.002:.4f}}} '
               f'orientation {{x: {qx} y: {qy} z: {qz} w: {qw}}}')
        r = subprocess.run(["ign", "service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "ignition.msgs.Pose",
                            "--reptype", "ignition.msgs.Boolean", "--timeout", "3000", "--req", req],
                           capture_output=True, text=True, timeout=10)
        moved += "data: true" in r.stdout
    print(f"PALLET SWAP OK: pallet {a.index} ({moved}/{len(boxes)} boxes) moved out to ({ox:.2f}, {oy:.2f}) as {name}; "
          f"pallet_main is empty")
    return 0 if moved == len(boxes) else 1


if __name__ == "__main__":
    sys.exit(main())
