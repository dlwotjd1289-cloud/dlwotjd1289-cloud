#!/usr/bin/env python3
"""Slot plan for run_stack_v44.sh -> lines "x y z yaw" (world frame, box centre, yaw rad).

Default: placeholder pattern for 0.40 x 0.30 x 0.25 m boxes, 2 x 3 per layer, 20 mm gaps,
rows far from the robot first (the arm never reaches over placed boxes to a farther slot),
optionally alternating 90 deg layers is NOT used because 0.30 x 0.40 does not tile 3 x 2 the
same way; keep yaw 0.

--plan FILE: JSON list in the AHEAD live viewer /api/place format
  [{"id": "...", "target_position_m": [x, y, z], "yaw_rad": 0.0, "size_m": [...]}, ...]
  (pallet frame: origin pallet centre, z = 0 at the deck top, box centre) - e.g. AHEAD output.
"""
import argparse
import json
import sys

PALLET_CENTER_XY = (0.0, 1.20)   # pallet_main in the Gazebo workcell
PALLET_TOP_Z = 0.15
BOX_SIZE = (0.40, 0.30, 0.25)    # test box SKU in Gazebo


def default_slots(n):
    out = []
    for layer in range(4):
        for dy in (0.32, 0.0, -0.32):
            for dx in (-0.21, 0.21):
                out.append((dx, dy, BOX_SIZE[2] / 2 + BOX_SIZE[2] * layer, 0.0))
    return out[:n]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("count", type=int)
    ap.add_argument("--plan", help="JSON plan in the /api/place format (pallet frame)")
    a = ap.parse_args()
    if a.plan:
        items = json.load(open(a.plan))
        slots = []
        for it in items[: a.count]:
            size = it.get("size_m", BOX_SIZE)
            if any(abs(s - t) > 1e-3 for s, t in zip(sorted(size), sorted(BOX_SIZE))):
                sys.exit(f"plan item {it.get('id')}: size {size} differs from the Gazebo test box {BOX_SIZE}")
            x, y, z = it["target_position_m"]
            slots.append((x, y, z, float(it.get("yaw_rad", 0.0))))
    else:
        slots = default_slots(a.count)
    if len(slots) < a.count:
        sys.exit(f"plan has only {len(slots)} slots for {a.count} boxes")
    for x, y, z, yaw in slots:
        print(f"{x + PALLET_CENTER_XY[0]:.4f} {y + PALLET_CENTER_XY[1]:.4f} {z + PALLET_TOP_Z:.4f} {yaw:.4f}")


if __name__ == "__main__":
    main()
