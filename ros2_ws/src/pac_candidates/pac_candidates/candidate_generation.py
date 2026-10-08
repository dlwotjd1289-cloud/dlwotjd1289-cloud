"""Stage 5-1: placement candidate generation.

Candidate set = EMS anchors U Extreme Points, for every allowed yaw.

* EMS (heightmap based): Empty Maximal Spaces of the coordinate-compressed
  heightmap, computed in the delta-inflated plane so that every anchored box
  automatically keeps the required clearance from its neighbours.
  Each EMS gives 5 reference points: 4 corners + centre.
* Extreme Points: corner points created by the placed boxes (right/front
  neighbours, their projections toward the origin walls, top-face corner).
* Orientation: ``config.yaw_set_rad`` (1st stage: 0 / 90 deg) intersected
  with ``box.allowed_yaws_rad``; yaws with the same AABB are kept once.
* Duplicates: candidates of the same yaw within ``dedup_distance_m`` in x
  and y (and the same level) are merged (80 mm by default).

``z`` is never taken from the EMS: it is the drop height of the inflated
footprint on the current heightmap, so a candidate always rests on
something. Feasibility is decided by the hard mask (5-2), not here.
"""

from dataclasses import dataclass
import math

from .geometry import LEN_EPS, Rect, rotated_dims, same_yaw
from .pallet_model import box_tolerance


@dataclass(frozen=True)
class RawCandidate:
    x: float
    y: float
    z: float
    yaw: float
    dims: tuple  # rotated (dx, dy, dz)
    source: str  # "EMS" | "EP"
    anchor: str  # corner_ll ... center | ep_* name
    ems: object  # Ems or None (EMS of origin / containing EMS)
    expanded: Rect
    support_est: float = 1.0  # vectorised pre-mask support ratio estimate
    proxy_ok: bool = True  # passes the cheap pre-mask estimate
    rank: tuple = ()  # precomputed priority key

    def priority(self):
        """Deepest-bottom-left-fill order; corners before centres."""
        return self.rank or _rank(self.x, self.y, self.z, self.anchor, self.yaw)


def _rank(x, y, z, anchor, yaw):
    # micrometre integer keys: exact ties, cheaper than round()
    return (
        int(z * 1e6 + 0.5),
        int(y * 1e6 + 0.5),
        int(x * 1e6 + 0.5),
        1 if anchor == "center" else 0,
        yaw,
    )


def candidate_yaws(box, config):
    """Allowed yaws in configured order; equal-footprint yaws deduplicated."""
    result = []
    footprints = set()
    for yaw in config.generation.yaw_set_rad:
        if not any(same_yaw(yaw, allowed) for allowed in box.allowed_yaws_rad):
            continue
        try:
            dims = rotated_dims(box.size, yaw)
        except ValueError:
            continue
        key = (round(dims[0], 9), round(dims[1], 9))
        if key in footprints:
            continue
        footprints.add(key)
        result.append(yaw)
    return result


def _ems_raw(model, box, yaw, dims, margin, anchors):
    out = []
    width = dims[0] + 2.0 * margin
    depth = dims[1] + 2.0 * margin
    for ems in model.ems():
        r = ems.rect
        if r.width + LEN_EPS < width or r.depth + LEN_EPS < depth:
            continue
        if ems.level + dims[2] > ems.top + LEN_EPS:
            continue
        cx, cy = r.center
        points = {
            "corner_ll": (r.x0, r.y0),
            "corner_lr": (r.x1 - width, r.y0),
            "corner_ul": (r.x0, r.y1 - depth),
            "corner_ur": (r.x1 - width, r.y1 - depth),
            "center": (cx - 0.5 * width, cy - 0.5 * depth),
        }
        for anchor in anchors:
            ex, ey = points[anchor]
            out.append((ex, ey, "EMS", anchor, ems))
    return out


def _project(model, x, y, z, axis):
    """Slide (x, y) toward the origin wall along ``axis`` (0 = x, 1 = y)
    until it meets an inflated box whose top is above ``z``."""
    bound = model.expanded_bounds
    best = bound.x0 if axis == 0 else bound.y0
    for g in model.boxes:
        if g.top <= z + model.height_tol:
            continue
        e = g.expanded
        if axis == 0:
            if e.y0 <= y + LEN_EPS and y < e.y1 - LEN_EPS and e.x1 <= x + LEN_EPS:
                best = max(best, e.x1)
        else:
            if e.x0 <= x + LEN_EPS and x < e.x1 - LEN_EPS and e.y1 <= y + LEN_EPS:
                best = max(best, e.y1)
    return best


def extreme_points(model):
    """Inflated-plane extreme points (anchor name, x, y); cached per model."""
    if model._extreme_points is None:
        model._extreme_points = tuple(_compute_extreme_points(model))
    return model._extreme_points


def _compute_extreme_points(model):
    bound = model.expanded_bounds
    points = [("ep_origin", bound.x0, bound.y0)]
    for g in model.boxes:
        e = g.expanded
        z = g.bottom
        points.append(("ep_right", e.x1, e.y0))
        points.append(("ep_front", e.x0, e.y1))
        points.append(("ep_right_proj", e.x1, _project(model, e.x1, e.y0, z, 1)))
        points.append(("ep_front_proj", _project(model, e.x0, e.y1, z, 0), e.y1))
        points.append(("ep_top", e.x0, e.y0))
    return points


def _ep_raw(model, box, dims, margin):
    width = dims[0] + 2.0 * margin
    depth = dims[1] + 2.0 * margin
    bound = model.expanded_bounds
    out = []
    for anchor, ex, ey in extreme_points(model):
        if ex + width > bound.x1 + LEN_EPS or ey + depth > bound.y1 + LEN_EPS:
            continue
        if ex < bound.x0 - LEN_EPS or ey < bound.y0 - LEN_EPS:
            continue
        out.append((ex, ey, "EP", anchor, None))
    return out


def containing_ems(model, expanded, z):
    """Largest EMS at ``z``'s level that contains the inflated footprint."""
    best = None
    for ems in model.ems():
        if abs(ems.level - z) > model.height_tol:
            continue
        if ems.rect.contains_rect(expanded, tol=1e-7):
            if best is None or ems.rect.area > best.rect.area:
                best = ems
    return best


def raw_candidates(model, box, config):
    """All candidates before deduplication, sorted by priority."""
    gen = config.generation
    uncertain = box.box_id in model.uncertain_ids
    tol = box_tolerance(config.uncertainty, uncertain)
    margin = model.half_gap + tol
    seen = set()
    result = []
    for yaw in candidate_yaws(box, config):
        dims = rotated_dims(box.size, yaw)
        anchors = []
        if gen.use_ems:
            anchors.extend(_ems_raw(model, box, yaw, dims, margin, gen.ems_anchors))
        if gen.use_extreme_points:
            anchors.extend(_ep_raw(model, box, dims, margin))
        if not anchors:
            continue
        w = dims[0] + 2.0 * margin
        d = dims[1] + 2.0 * margin
        rects = [(ex, ey, ex + w, ey + d) for ex, ey, _, _, _ in anchors]
        zs_arr = model.resting_z_many(rects)
        actual = [(r[0] + margin, r[1] + margin, r[2] - margin, r[3] - margin) for r in rects]
        sup_arr, heavy_arr = model.support_estimates(
            actual, zs_arr, box.weight_kg, config.constraints.heavy_on_light
        )
        ok_arr = (
            (sup_arr >= config.constraints.min_support_ratio - 1e-9)
            & ~heavy_arr
            & (zs_arr + dims[2] + tol <= model.pallet_size.z + 1e-9)
        )
        zs = zs_arr.tolist()
        for (ex, ey, source, anchor, ems), z, sup, ok in zip(
            anchors, zs, sup_arr.tolist(), ok_arr.tolist()
        ):
            expanded = Rect(ex, ey, ex + w, ey + d)
            x = ex + margin
            y = ey + margin
            rank = _rank(x, y, z, anchor, yaw)
            key = rank[:3] + (yaw,)
            if key in seen:
                continue
            seen.add(key)
            if ems is not None and abs(ems.level - z) > model.height_tol:
                ems = None  # resolved lazily for kept candidates only
            result.append(
                RawCandidate(
                    x, y, float(z), yaw, dims, source, anchor, ems, expanded,
                    float(sup), bool(ok), rank,
                )
            )
    result.sort(key=RawCandidate.priority)
    return result


def deduplicate(raws, distance, height_tol, is_valid=None):
    """Greedy duplicate removal within ``distance`` (Chebyshev in x/y).

    ``raws`` must be in priority order. The first candidate of a cluster is
    kept. With ``is_valid`` (mask-aware mode) a kept candidate that fails the
    hard mask is replaced by a later duplicate that passes it, so dedup never
    hides the only valid neighbour. Validity is evaluated lazily, only for
    candidates that actually collide. Returns (kept, validity-by-index).
    """
    if distance <= 0.0:
        return list(raws), {}
    validity = {}

    def valid(i):
        if i not in validity:
            validity[i] = bool(is_valid(raws[i]))
        return validity[i]

    kept = set()
    buckets = {}
    location = {}

    def conflicts(c, bx, by):
        for gx in (bx - 1, bx, bx + 1):
            for gy in (by - 1, by, by + 1):
                for j in buckets.get((gx, gy), ()):
                    o = raws[j]
                    if (
                        abs(o.x - c.x) < distance - 1e-12
                        and abs(o.y - c.y) < distance - 1e-12
                        and abs(o.z - c.z) <= height_tol
                        and same_yaw(o.yaw, c.yaw)
                    ):
                        yield j

    for i, c in enumerate(raws):
        bx = math.floor(c.x / distance)
        by = math.floor(c.y / distance)
        hits = list(conflicts(c, bx, by))
        if hits:
            if is_valid is None or not valid(i):
                continue
            invalid_hits = [j for j in hits if not valid(j)]
            if len(invalid_hits) != len(hits):
                continue  # a valid representative already exists
            for j in invalid_hits:
                kept.discard(j)
                buckets[location[j]].remove(j)
        kept.add(i)
        location[i] = (bx, by)
        buckets.setdefault((bx, by), []).append(i)
    return [raws[i] for i in sorted(kept)], validity
