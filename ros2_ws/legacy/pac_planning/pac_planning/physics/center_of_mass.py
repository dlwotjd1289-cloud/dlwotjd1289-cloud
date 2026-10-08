import math

import numpy as np

from pac_common.models import PalletState, Pose3D, Size3D


def combined_com(
    pallet: PalletState,
    candidate_size: Size3D,
    candidate_weight_kg: float,
    candidate_pose: Pose3D,
) -> tuple[float, float, float, float]:
    """Return total mass and mass-weighted CoM including the candidate."""
    masses = [b.weight_kg for b in pallet.boxes] + [candidate_weight_kg]
    centres = [[b.pose.x, b.pose.y, b.pose.z] for b in pallet.boxes]
    centres.append([candidate_pose.x, candidate_pose.y, candidate_pose.z])

    total = float(sum(masses))
    if total <= 0.0:
        return 0.0, 0.0, 0.0, 0.0

    xyz = np.asarray(centres, dtype=float)
    w = np.asarray(masses, dtype=float)
    com = np.average(xyz, axis=0, weights=w)
    return total, float(com[0]), float(com[1]), float(com[2])


def com_xy_metrics(pallet: PalletState, x_m: float, y_m: float) -> tuple[float, bool]:
    offset = math.hypot(x_m, y_m)
    inside = (
        abs(x_m) <= pallet.size.x / 2.0
        and abs(y_m) <= pallet.size.y / 2.0
    )
    return offset, inside
