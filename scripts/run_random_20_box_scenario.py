#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import random
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


PALLET_L = 1.10
PALLET_W = 1.10
GRID = 0.05  # 50 mm planning grid
NX = round(PALLET_L / GRID)
NY = round(PALLET_W / GRID)

# Dimensions are multiples of the 50 mm planning grid.
L_CHOICES = [0.20, 0.25, 0.30, 0.35, 0.40, 0.45]
W_CHOICES = [0.20, 0.25, 0.30, 0.35, 0.40]
H_CHOICES = [0.15, 0.20, 0.25, 0.30]


def http_json(
    base_url: str,
    method: str,
    path: str,
    payload: Optional[dict] = None,
) -> Tuple[int, dict]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        base_url + path,
        data=data,
        headers={"Content-Type": "application/json"},
        method=method,
    )
    try:
        with urllib.request.urlopen(req, timeout=10) as res:
            return res.status, json.loads(res.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def generate_boxes(count: int, rng: random.Random) -> List[dict]:
    boxes = []
    for i in range(count):
        # Favor realistic mixed sizes rather than tiny cubes.
        length = rng.choice(L_CHOICES)
        width = rng.choice(W_CHOICES)
        height = rng.choice(H_CHOICES)

        volume = length * width * height

        # Development-only randomized mass model:
        # use a broad density-equivalent range to create light/heavy cartons.
        density_equiv = rng.uniform(90.0, 360.0)  # kg/m^3
        mass = max(2.0, min(25.0, volume * density_equiv))

        boxes.append(
            {
                "id": f"R{i+1:02d}",
                "size_m": [length, width, height],
                "mass_kg": round(mass, 2),
                "color": random_carton_color(rng),
                "source": "random20_baseline",
            }
        )
    return boxes


def random_carton_color(rng: random.Random) -> str:
    palette = [
        "#c79058",
        "#b97b45",
        "#d39a62",
        "#a96c3c",
        "#d2aa78",
        "#bc8758",
        "#c69b6d",
    ]
    return rng.choice(palette)


def cells_for_size(length: float, width: float) -> Tuple[int, int]:
    return round(length / GRID), round(width / GRID)


def candidate_placements(
    heights: List[List[float]],
    length: float,
    width: float,
    height: float,
    max_height: float,
) -> List[Tuple[float, float, float, float, int, int, int, int]]:
    """Return candidates sorted by low height then near pallet center.

    tuple:
      (score, x_center, y_center, z_center, ix, iy, nx, ny)

    We require a flat support region in this baseline packer.  Dynamic
    stability itself is still determined by PyBullet after placement.
    """
    results = []

    for rotated in (False, True):
        l = width if rotated else length
        w = length if rotated else width
        nx, ny = cells_for_size(l, w)

        if nx > NX or ny > NY:
            continue

        for ix in range(NX - nx + 1):
            for iy in range(NY - ny + 1):
                footprint = [
                    heights[x][y]
                    for x in range(ix, ix + nx)
                    for y in range(iy, iy + ny)
                ]
                z0 = max(footprint)

                # Baseline placement uses a fully flat underlying footprint.
                # This avoids injecting deliberately unstable placements here.
                if max(footprint) - min(footprint) > 1e-9:
                    continue

                top = z0 + height
                if top > max_height + 1e-9:
                    continue

                x0 = -PALLET_L / 2.0 + ix * GRID
                y0 = -PALLET_W / 2.0 + iy * GRID
                x_center = x0 + l / 2.0
                y_center = y0 + w / 2.0
                z_center = z0 + height / 2.0

                center_penalty = math.hypot(x_center, y_center)
                edge_penalty = (
                    abs(x_center) / (PALLET_L / 2.0)
                    + abs(y_center) / (PALLET_W / 2.0)
                )

                # Lowest available layer dominates; center bias breaks ties.
                score = 100.0 * z0 + center_penalty + 0.05 * edge_penalty
                yaw = math.pi / 2.0 if rotated else 0.0
                results.append(
                    (
                        score,
                        x_center,
                        y_center,
                        z_center,
                        ix,
                        iy,
                        nx,
                        ny,
                        yaw,
                    )
                )

    results.sort(key=lambda c: c[0])
    return results


def update_heightmap(
    heights: List[List[float]],
    ix: int,
    iy: int,
    nx: int,
    ny: int,
    new_top: float,
) -> None:
    for x in range(ix, ix + nx):
        for y in range(iy, iy + ny):
            heights[x][y] = new_top


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Generate 20 reproducible random cartons and place them one by one "
            "into the running AHEAD PyBullet simulator using a simple "
            "height-map baseline packer."
        )
    )
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--seed", type=int, default=20261008)
    parser.add_argument("--delay", type=float, default=0.75)
    parser.add_argument("--settle", type=float, default=2.0)
    parser.add_argument("--url", default="http://127.0.0.1:4173")
    parser.add_argument(
        "--no-reset",
        action="store_true",
        help="do not reset the simulator before the run",
    )
    parser.add_argument(
        "--max-candidates",
        type=int,
        default=80,
        help="maximum candidate poses to try per box",
    )
    args = parser.parse_args()

    rng = random.Random(args.seed)

    status, initial = http_json(args.url, "GET", "/api/state")
    if status != 200:
        raise SystemExit("simulator state API is unavailable")

    max_height = float(initial["pallet"]["max_height_m"])

    if not args.no_reset:
        http_json(args.url, "POST", "/api/reset", {})

    heights = [[0.0 for _ in range(NY)] for _ in range(NX)]
    boxes = generate_boxes(args.count, rng)

    placed_records: List[dict] = []
    rejected_records: List[dict] = []

    print("===== RANDOM 20-BOX PHYSICS SCENARIO =====")
    print(f"seed       : {args.seed}")
    print(f"boxes      : {args.count}")
    print(f"pallet     : {PALLET_L:.2f} x {PALLET_W:.2f} m")
    print(f"height max : {max_height:.2f} m")
    print()

    for index, box in enumerate(boxes, start=1):
        L, W, H = box["size_m"]
        candidates = candidate_placements(
            heights, L, W, H, max_height
        )

        accepted = None
        last_error = None

        for c in candidates[: args.max_candidates]:
            (
                score,
                x,
                y,
                z,
                ix,
                iy,
                nx,
                ny,
                yaw,
            ) = c

            payload = dict(box)
            payload["target_position_m"] = [
                round(x, 5),
                round(y, 5),
                round(z, 5),
            ]
            payload["yaw_rad"] = yaw

            status, response = http_json(
                args.url, "POST", "/api/place", payload
            )

            if status == 200 and response.get("ok"):
                accepted = {
                    "payload": payload,
                    "grid": [ix, iy, nx, ny],
                    "planned_top_m": round(z + H / 2.0, 5),
                    "baseline_score": score,
                }
                update_heightmap(
                    heights,
                    ix,
                    iy,
                    nx,
                    ny,
                    z + H / 2.0,
                )
                break

            last_error = response.get("error", str(response))

        if accepted is None:
            rejected_records.append(
                {
                    "box": box,
                    "reason": last_error or "no feasible flat candidate",
                }
            )
            print(
                f"[{index:02d}/{args.count:02d}] "
                f"{box['id']}  REJECTED  "
                f"{box['size_m']} m  {box['mass_kg']:.2f} kg"
            )
            continue

        placed_records.append(accepted)

        print(
            f"[{index:02d}/{args.count:02d}] "
            f"{box['id']}  "
            f"{L:.2f}x{W:.2f}x{H:.2f} m  "
            f"{box['mass_kg']:5.2f} kg  ->  "
            f"xyz={accepted['payload']['target_position_m']}  "
            f"yaw={math.degrees(accepted['payload']['yaw_rad']):.0f}°"
        )

        time.sleep(max(0.0, args.delay))

    print()
    print(f"waiting {args.settle:.1f} s for final settling...")
    time.sleep(max(0.0, args.settle))

    _, final_state = http_json(args.url, "GET", "/api/state")
    metrics = final_state["metrics"]

    artifacts = Path.cwd() / "artifacts"
    artifacts.mkdir(parents=True, exist_ok=True)

    scenario_path = artifacts / f"random20_seed{args.seed}_scenario.json"
    final_path = artifacts / f"random20_seed{args.seed}_final_state.json"

    scenario_payload = {
        "seed": args.seed,
        "requested_box_count": args.count,
        "generated_boxes": boxes,
        "placed": placed_records,
        "rejected": rejected_records,
    }
    scenario_path.write_text(
        json.dumps(scenario_payload, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    final_path.write_text(
        json.dumps(final_state, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    print()
    print("===== FINAL RESULT =====")
    print(
        f"placed             : "
        f"{metrics['on_pallet_box_count']} / {args.count}"
    )
    print(f"mass on pallet     : {metrics['on_pallet_mass_kg']:.2f} kg")
    print(
        f"stack height       : "
        f"{metrics['current_height_m']:.3f} / "
        f"{metrics['height_limit_m']:.3f} m"
    )
    print(
        f"allowed-volume util: "
        f"{metrics['allowed_volume_utilization']*100:.2f} %"
    )
    print(
        f"current-stack util : "
        f"{metrics['current_stack_utilization']*100:.2f} %"
    )
    print(f"CoM XY offset      : {metrics['com_xy_offset_m']*1000:.1f} mm")
    print(f"moving boxes       : {metrics['moving_box_count']}")
    print(f"outside pallet     : {metrics['outside_pallet_box_count']}")
    print(f"max tilt           : {metrics['max_tilt_deg']:.2f} deg")

    overload_count = int(
        final_state.get("strength", {}).get("overloaded_box_count", 0)
    )
    print(f"overloaded boxes   : {overload_count}")
    print()
    print(f"scenario log       : {scenario_path}")
    print(f"final state log    : {final_path}")


if __name__ == "__main__":
    main()
