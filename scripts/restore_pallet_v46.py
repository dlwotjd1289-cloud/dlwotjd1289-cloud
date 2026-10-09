#!/usr/bin/env python3
"""Restore the boxes already placed in an earlier generator-cycle run (so a verification can resume
at the first unverified box instead of re-running the whole scenario).

For every box in RUN/planner_state.json the box model RUN/boxes/box_NN.sdf is created at the pose
the CCTV measured after the placement (RUN/box_NN_placed_pallet.json), bottom layer first, and the
gripper's spawn-time attach is released at once (the arm must stand still, as in the cycle runner).
Then run the cycle with RESUME_DIR=RUN: placed boxes are skipped, the next box is fed normally.

  python3 scripts/restore_pallet_v46.py --from logs/v44_generator_cycle/S0001_20261010_020616
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
from moveit_pick_place_v44 import gz_world_poses  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--from", dest="run", required=True, help="earlier generator-cycle run directory")
    a = ap.parse_args()
    state = json.load(open(os.path.join(a.run, "planner_state.json")))
    create = os.path.join(subprocess.run(["ros2", "pkg", "prefix", "ros_gz_sim"], capture_output=True,
                                         text=True).stdout.strip(), "lib/ros_gz_sim/create")
    boxes = []
    for b in state["placed"]:
        box = b["box_id"]
        m = json.load(open(os.path.join(a.run, f"{box}_placed_pallet.json")))
        boxes.append((m["z"], box, m))
    present = gz_world_poses()
    made = []
    for _, box, m in sorted(boxes):
        if box in present:
            print(f"  {box}: already in the world, kept")
            continue
        sdf = os.path.join(a.run, "boxes", f"{box}.sdf")
        subprocess.run([create, "-name", box, "-file", sdf, "-x", f"{m['x']:.4f}", "-y", f"{m['y']:.4f}",
                        "-z", f"{m['z'] + 0.003:.4f}", "-Y", f"{m['yaw']:.4f}"], capture_output=True, timeout=30)
        made.append(box)
        time.sleep(0.5)
    time.sleep(1.0)
    for box in made:   # DetachableJoint attaches a new box_NN on appearance: release it
        for _ in range(3):
            subprocess.run(["ign", "topic", "-t", f"/pac/gripper/{box}/detach", "-m", "ignition.msgs.Empty",
                            "-p", " "], capture_output=True, timeout=10)
    time.sleep(2.0)
    now = gz_world_poses()
    worst = 0.0
    for _, box, m in sorted(boxes):
        if box not in now:
            print(f"RESTORE FAIL: {box} not in the world")
            return 1
        d = math.hypot(now[box][0][0] - m["x"], now[box][0][1] - m["y"])
        worst = max(worst, d)
    print(f"RESTORE OK: {len(boxes)} boxes from {os.path.basename(a.run)} "
          f"(created {len(made)}, worst settle offset {worst * 1000:.1f} mm)")
    return 0 if worst < 0.01 else 1


if __name__ == "__main__":
    sys.exit(main())
