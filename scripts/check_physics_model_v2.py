#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PAC_SIM_SRC = ROOT / "ros2_ws" / "src" / "pac_simulation"
sys.path.insert(0, str(PAC_SIM_SRC))

from pac_simulation.ahead_sim.config import load_config


def main() -> None:
    cfg, _ = load_config(ROOT / "config" / "ahead_simulator.yaml")
    box, pallet = cfg.physics.bullet_body_frictions()

    print("===== AHEAD Physics Model V2 =====")
    print(
        f"pallet      : {cfg.pallet.length_m*1000:.0f} x "
        f"{cfg.pallet.width_m*1000:.0f} mm"
    )
    print(f"collision   : {cfg.pallet.collision_model}")
    print(f"top boards  : {cfg.pallet.top_board_count}")
    print()
    print("Target combined friction")
    print(
        f"  box-box    : {cfg.physics.target_box_box_friction:.4f}"
    )
    print(
        f"  box-pallet : {cfg.physics.target_box_pallet_friction:.4f}"
    )
    print("Bullet body friction")
    print(f"  box body   : {box:.4f}")
    print(f"  pallet body: {pallet:.4f}")
    print("Recovered combined")
    print(f"  box-box    : {box*box:.4f}")
    print(f"  box-pallet : {box*pallet:.4f}")
    print()
    print(
        f"contact averaging window: "
        f"{cfg.contact_force_average_window_s:.3f} s"
    )


if __name__ == "__main__":
    main()
