from shapely.geometry import box as shapely_box

from pac_common.models import PalletState, Pose3D, Size3D

from .geometry import footprint


def projected_quadrant_loads(
    pallet: PalletState,
    candidate_size: Size3D,
    candidate_weight_kg: float,
    candidate_pose: Pose3D,
) -> tuple[tuple[float, float, float, float], float]:
    """Projected mass balance over four pallet quadrants.

    This is intentionally a fast heuristic for current-layout balance. It is
    *not* contact-force propagation. Structural load propagation is a later
    support-graph layer.
    """
    hx = pallet.size.x / 2.0
    hy = pallet.size.y / 2.0
    quadrants = (
        shapely_box(0.0, 0.0, hx, hy),       # +x +y
        shapely_box(-hx, 0.0, 0.0, hy),      # -x +y
        shapely_box(-hx, -hy, 0.0, 0.0),     # -x -y
        shapely_box(0.0, -hy, hx, 0.0),      # +x -y
    )

    loads = [0.0, 0.0, 0.0, 0.0]
    items = [(b.size, b.weight_kg, b.pose) for b in pallet.boxes]
    items.append((candidate_size, candidate_weight_kg, candidate_pose))

    for size, mass, pose in items:
        poly = footprint(size, pose)
        area = poly.area
        if area <= 0.0 or mass <= 0.0:
            continue
        for i, quadrant in enumerate(quadrants):
            fraction = poly.intersection(quadrant).area / area
            loads[i] += mass * fraction

    total = sum(loads)
    if total <= 0.0:
        return tuple(loads), 1.0

    ideal = total / 4.0
    l1_from_ideal = sum(abs(v - ideal) for v in loads)
    # Maximum L1 distance for four bins occurs when all load is in one bin:
    # |M-.25M| + 3*|0-.25M| = 1.5M.
    normalised_imbalance = min(1.0, l1_from_ideal / (1.5 * total))
    return tuple(float(v) for v in loads), 1.0 - normalised_imbalance
