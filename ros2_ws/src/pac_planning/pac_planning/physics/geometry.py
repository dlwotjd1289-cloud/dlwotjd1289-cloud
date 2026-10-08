import math
from typing import Protocol

from shapely import affinity
from shapely.geometry import Polygon, box as shapely_box

from pac_common.models import PalletState, PlacementCandidate, Pose3D, Size3D


class SizedPose(Protocol):
    size: Size3D
    pose: Pose3D


def pallet_footprint(pallet: PalletState) -> Polygon:
    """Pallet top footprint.

    Planning convention used by Physics V1:
    - frame: ``pallet``
    - x/y origin: pallet geometric centre
    - z=0: pallet top surface
    - box poses represent box centres
    """
    hx = pallet.size.x / 2.0
    hy = pallet.size.y / 2.0
    return shapely_box(-hx, -hy, hx, hy)


def footprint(size: Size3D, pose: Pose3D) -> Polygon:
    """Horizontal rectangular footprint for a box centre pose."""
    hx = size.x / 2.0
    hy = size.y / 2.0
    poly = shapely_box(pose.x - hx, pose.y - hy, pose.x + hx, pose.y + hy)
    if not math.isclose(pose.yaw, 0.0, abs_tol=1e-12):
        poly = affinity.rotate(poly, pose.yaw, origin=(pose.x, pose.y), use_radians=True)
    return poly


def z_interval(size: Size3D, pose: Pose3D) -> tuple[float, float]:
    return pose.z - size.z / 2.0, pose.z + size.z / 2.0


def candidate_footprint(size: Size3D, candidate: PlacementCandidate) -> Polygon:
    return footprint(size, candidate.target_pose)


def inside_pallet(candidate_poly: Polygon, pallet: PalletState, tol_m: float) -> bool:
    # A tiny outward buffer only absorbs floating-point noise at exact boundaries.
    return pallet_footprint(pallet).buffer(tol_m).covers(candidate_poly)


def collides_with_existing(
    size: Size3D,
    pose: Pose3D,
    pallet: PalletState,
    tol_m: float,
) -> bool:
    cand_poly = footprint(size, pose)
    cand_bottom, cand_top = z_interval(size, pose)

    for placed in pallet.boxes:
        other_bottom, other_top = z_interval(placed.size, placed.pose)
        vertical_overlap = min(cand_top, other_top) - max(cand_bottom, other_bottom)
        if vertical_overlap <= tol_m:
            continue
        if cand_poly.intersection(footprint(placed.size, placed.pose)).area > tol_m * tol_m:
            return True
    return False
