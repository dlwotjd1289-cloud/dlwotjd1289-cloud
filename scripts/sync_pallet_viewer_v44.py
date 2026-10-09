#!/usr/bin/env python3
"""Send boxes placed in Gazebo to the AHEAD live pallet simulator (PyBullet + Three.js viewer).

Frame conversion: Gazebo pallet_main center (0, 1.20), deck top z = 0.15
-> AHEAD sim pallet frame (origin = pallet center, z = 0 at deck top).
Uses the box's *actual* Gazebo pose and the mass measured on the scale.

  python3 scripts/sync_pallet_viewer_v44.py --box box_01 --mass 5.000 [--url http://127.0.0.1:4173]
"""
import argparse
import json
import math
import sys
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from moveit_pick_place_v44 import BOX_SIZE, PALLET_TOP_Z, PLACE_XY, gz_model_pose  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--box", required=True)
    ap.add_argument("--mass", type=float, required=True, help="measured mass [kg] from the V4.3 scale")
    ap.add_argument("--url", default="http://127.0.0.1:4173")
    ap.add_argument("--size", type=float, nargs=3, default=list(BOX_SIZE), metavar=("X", "Y", "Z"))
    ap.add_argument("--robot", action="store_true",
                    help="let the viewer's HDR50-22 pick and release the box (/api/robot_place) instead of "
                         "inserting it at the measured pose (/api/place)")
    a = ap.parse_args()
    pose = gz_model_pose(a.box)
    if pose is None:
        print(f"SYNC FAIL: {a.box} not found in Gazebo")
        return 1
    (x, y, z), (qx, qy, qz, qw) = pose
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    payload = {"id": a.box, "size_m": list(a.size), "mass_kg": round(a.mass, 3),
               "target_position_m": [round(x - PLACE_XY[0], 4), round(y - PLACE_XY[1], 4), round(z - PALLET_TOP_Z, 4)],
               "yaw_rad": round(yaw, 4)}
    req = urllib.request.Request(a.url + ("/api/robot_place" if a.robot else "/api/place"), data=json.dumps(payload).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            body = json.loads(r.read().decode())
    except (urllib.error.URLError, OSError) as exc:
        print(f"SYNC SKIPPED: viewer not reachable at {a.url} ({exc}); payload={json.dumps(payload)}")
        return 2
    print(f"SYNC OK: {json.dumps(payload)} -> {body.get('ok')}")
    return 0 if body.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
