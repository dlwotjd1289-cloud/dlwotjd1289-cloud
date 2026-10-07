#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import threading
import webbrowser
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PAC_SIM_SRC = PROJECT_ROOT / "ros2_ws" / "src" / "pac_simulation"
if str(PAC_SIM_SRC) not in sys.path:
    sys.path.insert(0, str(PAC_SIM_SRC))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="PAC2026 AHEAD live PyBullet pallet simulator"
    )
    parser.add_argument(
        "--config",
        default=str(PROJECT_ROOT / "config" / "ahead_simulator.yaml"),
    )
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument(
        "--max-height",
        type=float,
        default=None,
        help="override pallet max_height_m",
    )
    parser.add_argument("--no-browser", action="store_true")
    return parser.parse_args()


def main() -> None:
    try:
        from aiohttp import web
        from pac_simulation.ahead_sim.config import load_config
        from pac_simulation.ahead_sim.models import (
            PalletConfig,
            SimulatorConfig,
        )
        from pac_simulation.ahead_sim.server import AheadWebServer
        from pac_simulation.ahead_sim.simulator import AheadLiveSimulator
    except ImportError as exc:
        print(f"[ERROR] missing dependency: {exc}")
        print(
            "Install with:\n"
            "  python3 -m pip install --user "
            "-r requirements-ahead-sim.txt"
        )
        raise SystemExit(2)

    args = parse_args()
    config_path = Path(args.config)
    cfg, raw = load_config(config_path)

    if args.max_height is not None:
        cfg = SimulatorConfig(
            pallet=PalletConfig(
                length_m=cfg.pallet.length_m,
                width_m=cfg.pallet.width_m,
                deck_height_m=cfg.pallet.deck_height_m,
                max_height_m=args.max_height,
            ),
            physics=cfg.physics,
            load_grid_size=cfg.load_grid_size,
            contact_display_min_force_n=cfg.contact_display_min_force_n,
        )

    server_cfg = raw.get("server", {})
    host = args.host or str(server_cfg.get("host", "127.0.0.1"))
    port = args.port or int(server_cfg.get("port", 4173))

    sim = AheadLiveSimulator(cfg)
    server = AheadWebServer(sim, PROJECT_ROOT)
    app = server.make_app()

    print("===== PAC2026 AHEAD Live Physics Simulator =====")
    print("physics     : PyBullet (always ON)")
    print(
        "pallet      : "
        f"{cfg.pallet.length_m*1000:.0f} x "
        f"{cfg.pallet.width_m*1000:.0f} mm"
    )
    print(f"height limit: {cfg.pallet.max_height_m:.3f} m")
    print(f"physics rate: {cfg.physics.physics_hz} Hz")
    print(f"viewer rate : {cfg.physics.viewer_hz} Hz")
    print(f"viewer      : http://{host}:{port}")
    print()
    print("AHEAD placement API: POST /api/place")

    if not args.no_browser:
        threading.Timer(
            1.0, lambda: webbrowser.open(f"http://{host}:{port}")
        ).start()

    web.run_app(app, host=host, port=port, print=None)


if __name__ == "__main__":
    main()
