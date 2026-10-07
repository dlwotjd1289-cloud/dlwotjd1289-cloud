#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Convert a measured critical static tilt angle to an approximate "
            "Coulomb friction coefficient: mu ~= tan(theta)."
        )
    )
    parser.add_argument(
        "angle_deg",
        type=float,
        help="critical tilt angle in degrees at first sustained sliding",
    )
    args = parser.parse_args()

    if not (0.0 <= args.angle_deg < 89.0):
        raise SystemExit("angle_deg must be in [0, 89)")

    mu = math.tan(math.radians(args.angle_deg))
    print(f"critical angle : {args.angle_deg:.3f} deg")
    print(f"mu estimate    : {mu:.6f}")
    print()
    print("Use this as a measured TARGET COMBINED contact coefficient,")
    print("not directly as a Bullet body friction value.")


if __name__ == "__main__":
    main()
