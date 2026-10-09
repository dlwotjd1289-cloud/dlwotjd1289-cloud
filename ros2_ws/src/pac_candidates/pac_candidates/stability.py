"""Stack-level stability checks on a whole layout (pallet frame, metres).

Three analytic checks, each a function of the layout only:

* ``lateral``: the team's 0.3 g check (decided 2026-10-09, ported from
  ``tools/prototypes/lookahead/lookahead.py`` ``stack_lateral_ok``). For every
  box, the composite centre of mass of the box and everything resting on it
  (transitively) must stay inside the box's support polygon when shifted by
  ``a_g * h`` (h = height of that centre above the box bottom) along +-x and
  +-y. Pallet edges and neighbours are not walls.
* ``sme``: static mechanical equilibrium, the same test at ``a_g = 0``: the
  point of gravitational force must be supported at every level of the stack
  (Krebs & Ehmke 2021, as compared in Mazur et al. 2025, ITOR 34(1):501-527).
* ``pbs``: partial base support, supported base area / base area >= alpha
  (Mazur et al. 2025 section 2.1; the hard mask uses 0.70).
* ``unit_tipping``: a wrapped load unit as one rigid body (boxes and pallet
  held together): it stays upright while ``a_g * h_cg`` is smaller than the
  distance from its centre of mass to the pallet edge, along +-x / +-y.

Support polygon = convex hull of the contact rectangles with the boxes
directly below (Ramos et al. 2016a), or the box footprint on the pallet.
"""

from dataclasses import dataclass
import math

Z_TOL_M = 0.004          # bottom / top faces closer than this are in contact (as the prototype)
MIN_CONTACT_M = 1e-3     # contact rectangles narrower than this are ignored
AXES = ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0))


@dataclass(frozen=True)
class StackBox:
    """Axis-aligned box: min corner (x, y, z), extents (dx, dy, dz), mass.

    ``com_offset`` is the centre of mass relative to the geometric centre
    (Mazur et al. 2025 "displaced centre of mass"); zero = uniform box.
    """

    box_id: str
    x: float
    y: float
    z: float
    dx: float
    dy: float
    dz: float
    mass_kg: float
    com_offset: tuple = (0.0, 0.0, 0.0)

    @property
    def com(self):
        ox, oy, oz = self.com_offset
        return (self.x + 0.5 * self.dx + ox, self.y + 0.5 * self.dy + oy, self.z + 0.5 * self.dz + oz)

    @property
    def top(self):
        return self.z + self.dz


@dataclass(frozen=True)
class SupportGraph:
    below: tuple    # below[i] = ((j, (x0, y0, x1, y1)), ...) boxes directly under i
    above: tuple    # above[j] = (i, ...) boxes directly on j
    on_pallet: tuple


def support_graph(boxes, z_tol=Z_TOL_M):
    n = len(boxes)
    below = [[] for _ in range(n)]
    on_pallet = [b.z <= z_tol for b in boxes]
    for i, b in enumerate(boxes):
        if on_pallet[i]:
            continue
        for j, s in enumerate(boxes):
            if j == i or abs(s.top - b.z) > z_tol:
                continue
            x0, x1 = max(b.x, s.x), min(b.x + b.dx, s.x + s.dx)
            y0, y1 = max(b.y, s.y), min(b.y + b.dy, s.y + s.dy)
            if x1 - x0 > MIN_CONTACT_M and y1 - y0 > MIN_CONTACT_M:
                below[i].append((j, (x0, y0, x1, y1)))
    above = [[] for _ in range(n)]
    for i in range(n):
        for j, _ in below[i]:
            above[j].append(i)
    return SupportGraph(tuple(map(tuple, below)), tuple(map(tuple, above)), tuple(on_pallet))


def _hull(points):
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def inside_distance(poly, point):
    """Signed distance from ``point`` to the boundary of convex ``poly``
    (counter-clockwise): > 0 inside, <= 0 on or outside."""
    if len(poly) < 3:
        return -math.inf
    best = math.inf
    px, py = point
    for k in range(len(poly)):
        (ax, ay), (bx, by) = poly[k], poly[(k + 1) % len(poly)]
        ex, ey = bx - ax, by - ay
        length = math.hypot(ex, ey)
        if length <= 0.0:
            continue
        best = min(best, (ex * (py - ay) - ey * (px - ax)) / length)
    return best


def support_polygon(boxes, graph, i):
    b = boxes[i]
    if graph.on_pallet[i]:
        return _hull([(b.x, b.y), (b.x + b.dx, b.y), (b.x + b.dx, b.y + b.dy), (b.x, b.y + b.dy)])
    pts = []
    for _, (x0, y0, x1, y1) in graph.below[i]:
        pts += [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return _hull(pts)


def _carried(graph, i):
    group, todo = set(), [i]
    while todo:
        q = todo.pop()
        if q not in group:
            group.add(q)
            todo.extend(graph.above[q])
    return group


def _load_path(graph, i):
    path, todo = [], [i]
    while todo:
        k = todo.pop()
        if k not in path:
            path.append(k)
            todo.extend(j for j, _ in graph.below[k])
    return path


def box_margin(boxes, graph, k, a_g):
    """Smallest inside distance (m) of the shifted composite CoG over the
    four axes for box ``k`` and everything it carries."""
    group = _carried(graph, k)
    m = sum(boxes[q].mass_kg for q in group) or 1e-9
    cx = sum(boxes[q].mass_kg * boxes[q].com[0] for q in group) / m
    cy = sum(boxes[q].mass_kg * boxes[q].com[1] for q in group) / m
    cz = sum(boxes[q].mass_kg * boxes[q].com[2] for q in group) / m
    poly = support_polygon(boxes, graph, k)
    h = cz - boxes[k].z
    if a_g <= 0.0:
        return inside_distance(poly, (cx, cy))
    return min(inside_distance(poly, (cx + ux * a_g * h, cy + uy * a_g * h)) for ux, uy in AXES)


@dataclass(frozen=True)
class CheckResult:
    ok: bool
    margin_m: float          # worst inside distance (m); < 0 = outside
    worst_box: str = ""


def column_check(boxes, a_g, graph=None, only=None):
    """``a_g`` > 0: lateral check; ``a_g`` = 0: static mechanical equilibrium.
    ``only``: index of a newly placed box -> check its load path only."""
    graph = graph or support_graph(boxes)
    indices = _load_path(graph, only) if only is not None else range(len(boxes))
    worst, who = math.inf, ""
    for k in indices:
        margin = box_margin(boxes, graph, k, a_g)
        if margin < worst:
            worst, who = margin, boxes[k].box_id
    if worst == math.inf:       # nothing to check (empty layout)
        return CheckResult(True, 0.0, "")
    return CheckResult(worst > 0.0, worst, who)


def lateral_check(boxes, a_g=0.3, graph=None, only=None):
    if a_g <= 0.0:
        raise ValueError("lateral acceleration must be > 0 g")
    return column_check(boxes, a_g, graph, only)


def sme_check(boxes, graph=None):
    return column_check(boxes, 0.0, graph)


def pbs_check(boxes, alpha=0.70, graph=None):
    """Worst supported-area ratio minus alpha (reported in ``margin_m``)."""
    graph = graph or support_graph(boxes)
    worst, who = math.inf, ""
    for i, b in enumerate(boxes):
        ratio = 1.0 if graph.on_pallet[i] else min(
            1.0, sum((x1 - x0) * (y1 - y0) for _, (x0, y0, x1, y1) in graph.below[i]) / (b.dx * b.dy))
        if ratio - alpha < worst:
            worst, who = ratio - alpha, b.box_id
    worst = 0.0 if worst == math.inf else worst
    return CheckResult(worst >= -1e-9, worst, who)


def unit_tipping_check(boxes, a_g, pallet_xy, deck_height_m):
    """Whole wrapped unit on its pallet base: margin = edge distance of the
    centre of mass - a_g x its height above the pallet bottom. The pallet's
    own mass is left out (it would lower the centre of mass)."""
    if not boxes:
        return CheckResult(True, 0.0, "")
    m = sum(b.mass_kg for b in boxes) or 1e-9
    cx, cy, cz = (sum(b.mass_kg * b.com[k] for b in boxes) / m for k in range(3))
    lx, ly = pallet_xy
    edge = min(cx, lx - cx, cy, ly - cy)
    margin = edge - a_g * (cz + deck_height_m)
    return CheckResult(margin > 0.0, margin, "unit")


def boxes_from_layout(layout):
    """``[{"box_id", "size": [x, y, z], "pose": [x, y, z, yaw], "mass_kg"}]``
    (pallet-frame min corner, the format of tools/runtime physics_replay)."""
    out = []
    for b in layout:
        sx, sy, sz = b["size"]
        x, y, z, yaw = b["pose"]
        if int(round(yaw / (math.pi / 2))) % 2:
            sx, sy = sy, sx
        ox, oy, oz = b.get("com_offset", (0.0, 0.0, 0.0))   # box frame -> pallet frame
        c, s = math.cos(yaw), math.sin(yaw)
        out.append(StackBox(str(b["box_id"]), x, y, z, sx, sy, sz, float(b.get("mass_kg") or 1.0),
                            (ox * c - oy * s, ox * s + oy * c, oz)))
    return out
