#!/usr/bin/env python3
"""Replay a generator-cycle run (planner commands) on the robot cell of the AHEAD live simulator.

For every box that the planner placed in the Gazebo run (RUN_DIR/box_NN_plan.json), the same
command (size from the arrival list, mass from the V4.3 scale log, planned centre and yaw) is
sent to POST /api/robot_place: the PyBullet HDR50-22 picks it at PICK and releases it at the
target, and Bullet decides how it settles. Prints per-box result vs. the commanded target.

  python3 scripts/replay_to_robot_cell_v44.py logs/v44_generator_cycle/S0001_... [--url ...] [--reset]
"""
import argparse
import glob
import json
import os
import re
import sys
import time
import urllib.request

GZ_TO_SIM = (0.0, -1.20, -0.15)


def call(url, path, payload=None):
    data = json.dumps(payload).encode() if payload is not None else None
    req = urllib.request.Request(url + path, data=data, headers={"Content-Type": "application/json"},
                                 method="POST" if payload is not None else "GET")
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read().decode())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dirs", nargs="+", help="generator cycle run dir(s); later dirs win")
    ap.add_argument("--url", default="http://127.0.0.1:4173")
    ap.add_argument("--reset", action="store_true", help="POST /api/reset first")
    ap.add_argument("--timeout", type=float, default=1800)
    a = ap.parse_args()
    plans, arrivals, masses = {}, {}, {}
    for d in a.run_dirs:
        for f in glob.glob(os.path.join(d, "box_*_plan.json")):
            plans[os.path.basename(f)[:6]] = json.load(open(f))
        tsv = os.path.join(d, "arrivals.tsv")
        if os.path.exists(tsv):
            for line in open(tsv):
                k, sku, x, y, z, m, gid = line.rstrip("\n").split("\t")
                arrivals[f"box_{int(k):02d}"] = (sku, (float(x), float(y), float(z)), float(m))
        for f in glob.glob(os.path.join(d, "box_*_v43.log")):
            mm = re.search(r"V4\.3 PASS: ([0-9.]+)", open(f).read())
            if mm:
                masses[os.path.basename(f)[:6]] = float(mm.group(1))
    if a.reset:
        call(a.url, "/api/reset", {})
    order = sorted(plans)
    print(f"replaying {len(order)} planner commands on the robot cell")
    for box in order:
        pl = plans[box]
        sku, size, gen_mass = arrivals[box]
        sw = pl["slot_world"]
        target = [round(sw[i] + GZ_TO_SIM[i], 4) for i in range(3)]
        cmd = {"id": box, "size_m": list(size), "mass_kg": masses.get(box, gen_mass),
               "target_position_m": target, "yaw_rad": pl["yaw_rad"], "source": f"planner:{pl['candidate_id']}"}
        print(f"  {box} {sku} -> {target} yaw {pl['yaw_rad']:.2f}: {call(a.url, '/api/robot_place', cmd)}")
    t0 = time.time()
    while time.time() - t0 < a.timeout:
        r = call(a.url, "/api/state")["robot"]
        if not r["queue"] and r["state"] == "READY":
            break
        time.sleep(2)
    r = call(a.url, "/api/state")["robot"]
    done = {c["id"]: c for c in r["completed"]}
    print("\nresult (robot release in Bullet vs. commanded target):")
    for box in order:
        c = done.get(box)
        print(f"  {box}: " + (f"xy {c['xy_error_mm']} mm, z {c['z_error_mm']} mm, tilt {c['tilt_deg']} deg" if c else "NOT PLACED"))
    failed = [l for l in r["log"] if "FAILED" in l]
    print(f"placed {len(done)}/{len(order)}; failures in log: {failed or 'none'}")
    return 0 if len(done) == len(order) else 1


if __name__ == "__main__":
    sys.exit(main())
