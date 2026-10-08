import math

from shapely.ops import unary_union

from pac_common.models import PalletState, Pose3D, Size3D

from .geometry import footprint, z_interval
from .types import SupportContact


PALLET_SUPPORT_ID = "__PALLET__"


def evaluate_support(
    size: Size3D,
    pose: Pose3D,
    pallet: PalletState,
    z_tol_m: float,
) -> tuple[float, tuple[SupportContact, ...]]:
    """Return support area ratio and support contributors.

    V1 assumes horizontal box top / bottom faces. A first-layer box is supported
    by the pallet when its bottom face is on z=0 in the pallet planning frame.
    Higher boxes are supported by existing boxes whose top face is coplanar with
    the candidate bottom face.
    """
    cand_poly = footprint(size, pose)
    base_area = cand_poly.area
    if base_area <= 0.0:
        return 0.0, ()

    cand_bottom, _ = z_interval(size, pose)

    if math.isclose(cand_bottom, 0.0, abs_tol=z_tol_m):
        return 1.0, (SupportContact(PALLET_SUPPORT_ID, base_area, 1.0),)

    intersections: list[tuple[str, object]] = []
    for placed in pallet.boxes:
        _, placed_top = z_interval(placed.size, placed.pose)
        if not math.isclose(placed_top, cand_bottom, abs_tol=z_tol_m):
            continue
        inter = cand_poly.intersection(footprint(placed.size, placed.pose))
        if inter.area > 0.0:
            intersections.append((placed.box_id, inter))

    if not intersections:
        return 0.0, ()

    supported_geom = unary_union([geom for _, geom in intersections])
    support_ratio = max(0.0, min(1.0, supported_geom.area / base_area))

    # Existing boxes are expected not to overlap at the same layer. We still
    # normalise individual areas so the load-share contract remains bounded.
    area_sum = sum(geom.area for _, geom in intersections)
    contacts = tuple(
        SupportContact(box_id, geom.area, (geom.area / area_sum if area_sum > 0.0 else 0.0))
        for box_id, geom in intersections
    )
    return support_ratio, contacts
