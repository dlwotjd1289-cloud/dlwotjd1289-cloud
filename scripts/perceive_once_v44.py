#!/usr/bin/env python3
"""Wait for a stable fixed-CCTV detection of the box at PICK and write it as JSON.

  python3 scripts/perceive_once_v44.py --box box_03 --size 0.35 0.25 0.15 --out P.json [--timeout 20]
Publishes the target (name + SKU size, needed for the box-top plane) to the perception node.
Stable = two consecutive OK detections (from images newer than this call) within 5 mm / 1 deg.
"""
import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--box", default="v43_scale_box_5kg")
    ap.add_argument("--size", type=float, nargs=3, default=[0.40, 0.30, 0.25])
    a = ap.parse_args()
    rclpy.init()
    node = Node("pac_perceive_once_v44")
    seen = []
    node.create_subscription(String, "/pac/perception/far/box_info", lambda m: seen.append(json.loads(m.data)), 10)
    target = node.create_publisher(String, "/pac/perception/target", 10)
    target_msg = String(data=json.dumps({"box": a.box, "size": a.size}))
    end = time.monotonic() + a.timeout
    prev, last_status = None, None
    try:
        while time.monotonic() < end:
            target.publish(target_msg)
            rclpy.spin_once(node, timeout_sec=0.1)
            while seen:
                info = seen.pop(0)
                if info.get("box") != a.box:
                    continue
                last_status = info.get("status")
                if last_status != "OK":
                    prev = None
                    continue
                if prev and math.hypot(info["x"] - prev["x"], info["y"] - prev["y"]) < 0.005 \
                        and abs(info["yaw"] - prev["yaw"]) < math.radians(1.0):
                    open(a.out, "w").write(json.dumps(info, indent=2))
                    print(f"PERCEPTION OK: ({info['x']:.3f}, {info['y']:.3f}) yaw {math.degrees(info['yaw']):.1f} deg, "
                          f"footprint {info['length']:.3f} x {info['width']:.3f} m")
                    return 0
                prev = info
        print(f"PERCEPTION FAIL: no stable detection in {a.timeout:.0f} s (last status {last_status})")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
