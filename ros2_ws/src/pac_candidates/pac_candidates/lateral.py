"""Hard-mask step 13: lateral-acceleration stability of the unwrapped load.

The load (every box the new box can affect) must stay at rest when the pallet
accelerates sideways by ``accel_g`` (default 0.25 g) in any of ``directions``.
Three stages, cheapest first (docs/donghan/lateral_stability.md):

1. geom  For the new box and every box on its load path, the CoG of that box
         plus everything resting on it, shifted by a * (CoG height above the
         contact), must stay inside its support polygon (Jaesung's stack check,
         no side support, no friction). a_geom >= band_high_g: pass.
2. lp    Force-equilibrium LP over the affected system (load path, everything
         on it, neighbours within ``side_contact_gap_m``, closed transitively).
         Bottom contacts push only, friction is a Coulomb octagon inside
         mu * normal (exact along the 8 test directions), side contacts push
         only and carry no friction. The LP maximises a per direction.
         a_lp >= band_high_g: pass, a_lp < band_low_g: reject.
3. sim   band_low_g <= a_lp < band_high_g: short PyBullet tilt test
         (lateral_sim.py) in the worst LP directions; without pybullet the
         ``borderline`` policy decides.

Every box carries its CoG uncertainty (pallet_model.cog_delta), shifted
towards the push direction. Units inside: metres, kilograms, forces in kgf.
"""

from dataclasses import dataclass, field
import math

import numpy as np

from .geometry import LEN_EPS, Rect, convex_hull
from .pallet_model import cog_delta

A_CAP_G = 1.0          # LP upper bound for a (keeps the problem bounded)
_OCT_K = math.cos(math.pi / 8.0)
# octagon facets between the 8 test directions -> exact mu along them
_OCT_ANGLES = tuple(math.pi / 8.0 + m * math.pi / 4.0 for m in range(8))


@dataclass(frozen=True)
class Body:
    box_id: str
    lo: tuple
    hi: tuple
    mass: float
    tol: float
    delta: tuple  # CoG uncertainty (x, y)

    @property
    def rect(self):
        return Rect(self.lo[0], self.lo[1], self.hi[0], self.hi[1])

    @property
    def center(self):
        return tuple(0.5 * (a + b) for a, b in zip(self.lo, self.hi))


@dataclass
class Assessment:
    """Raw numbers of one candidate (all directions); see ``decide``."""

    system: list                      # body indices of the affected system
    directions: list                  # unit 2D vectors
    a_geom: list                      # per direction
    a_lp: list = field(default_factory=list)   # per direction (None = not solved)
    lp_status: list = field(default_factory=list)


@dataclass(frozen=True)
class LateralResult:
    passed: bool
    stage: str        # geom | lp | sim | borderline_reject | borderline_accept
    a_max_g: float    # lowest estimate available at the deciding stage
    detail: dict


def directions(count):
    step = 2.0 * math.pi / count
    return [(round(math.cos(k * step), 12), round(math.sin(k * step), 12)) for k in range(count)]


def deck_rects(cfg, pallet_size):
    """Support regions of the pallet top (pallet frame, origin at the corner)."""
    if cfg.deck == "solid":
        return [Rect(0.0, 0.0, pallet_size.x, pallet_size.y)]
    w = cfg.deck_board_width_ratio * pallet_size.y
    n = cfg.deck_board_count
    span = pallet_size.y - w
    centres = [0.5 * pallet_size.y] if n == 1 else [
        0.5 * pallet_size.y - 0.5 * span + i * span / (n - 1) for i in range(n)
    ]
    return [Rect(0.0, c - 0.5 * w, pallet_size.x, c + 0.5 * w) for c in centres]


def _shrink(rect, t):
    if rect is None or rect.width <= 2 * t + LEN_EPS or rect.depth <= 2 * t + LEN_EPS:
        return None
    return Rect(rect.x0 + t, rect.y0 + t, rect.x1 - t, rect.y1 - t)


class Scene:
    """Placed boxes plus the candidate (last index) with neighbour queries."""

    def __init__(self, model, box, pose, rect, dz, tol, uncertain):
        unc = model.config.uncertainty
        bodies = [
            Body(g.box_id, g.lo, g.hi, g.weight_kg, g.tolerance_m,
                 cog_delta(unc, g.rect.width, g.rect.depth, g.uncertain))
            for g in model.boxes
        ]
        bodies.append(Body(box.box_id, (rect.x0, rect.y0, pose.z), (rect.x1, rect.y1, pose.z + dz),
                           float(box.weight_kg), tol, cog_delta(unc, rect.width, rect.depth, uncertain)))
        self.bodies = bodies
        self.new = len(bodies) - 1
        self.lo = np.array([b.lo for b in bodies], dtype=float)
        self.hi = np.array([b.hi for b in bodies], dtype=float)
        self.htol = model.height_tol
        self.cfg = model.config.constraints.lateral
        self.deck = deck_rects(self.cfg, model.pallet_size)

    def _xy_overlap(self, i):
        return ((np.minimum(self.hi[:, 0], self.hi[i, 0]) - np.maximum(self.lo[:, 0], self.lo[i, 0]) > LEN_EPS)
                & (np.minimum(self.hi[:, 1], self.hi[i, 1]) - np.maximum(self.lo[:, 1], self.lo[i, 1]) > LEN_EPS))

    def below(self, i):
        if self.lo[i, 2] <= self.htol:
            return []
        m = (np.abs(self.hi[:, 2] - self.lo[i, 2]) <= self.htol) & self._xy_overlap(i)
        m[i] = False
        return np.flatnonzero(m).tolist()

    def above(self, i):
        m = (np.abs(self.lo[:, 2] - self.hi[i, 2]) <= self.htol) & self._xy_overlap(i)
        m[i] = False
        return np.flatnonzero(m).tolist()

    def side(self, i):
        gap = self.cfg.side_contact_gap_m
        lo, hi = self.lo, self.hi
        zov = np.minimum(hi[:, 2], hi[i, 2]) - np.maximum(lo[:, 2], lo[i, 2]) > self.htol
        out = np.zeros(len(self.bodies), dtype=bool)
        for ax, other in ((0, 1), (1, 0)):
            ov = np.minimum(hi[:, other], hi[i, other]) - np.maximum(lo[:, other], lo[i, other]) > LEN_EPS
            g1 = lo[:, ax] - hi[i, ax]          # neighbour on the + side
            g2 = lo[i, ax] - hi[:, ax]          # neighbour on the - side
            near = ((g1 >= -LEN_EPS) & (g1 <= gap)) | ((g2 >= -LEN_EPS) & (g2 <= gap))
            out |= zov & ov & near
        out[i] = False
        return np.flatnonzero(out).tolist()

    def closure(self, start, relations):
        seen, todo = {start}, [start]
        while todo:
            i = todo.pop()
            for rel in relations:
                for j in rel(i):
                    if j not in seen:
                        seen.add(j)
                        todo.append(j)
        return sorted(seen)

    def system(self):
        return self.closure(self.new, (self.below, self.above, self.side))

    def bottom_regions(self, i):
        """[(Rect, supporter index or None for the deck, mu)] of body i."""
        b = self.bodies[i]
        if self.lo[i, 2] <= self.htol:
            regions = [_shrink(b.rect.intersection(d), b.tol) for d in self.deck]
            return [(r, None, self.cfg.mu_box_pallet) for r in regions if r is not None]
        out = []
        for j in self.below(i):
            r = _shrink(b.rect.intersection(self.bodies[j].rect), b.tol)
            if r is not None:
                out.append((r, j, self.cfg.mu_box_box))
        return out


def _shifted_cog(body, u):
    c = body.center
    sx = math.copysign(body.delta[0], u[0]) if abs(u[0]) > 1e-12 else 0.0
    sy = math.copysign(body.delta[1], u[1]) if abs(u[1]) > 1e-12 else 0.0
    return (c[0] + sx, c[1] + sy, c[2])


def _ray_exit(poly, point, u):
    """Distance from ``point`` along ``u`` to the boundary of a convex CCW polygon
    (<= 0 when the point is outside)."""
    if len(poly) < 3:
        return -1.0
    best = math.inf
    n = len(poly)
    for k in range(n):
        (x1, y1), (x2, y2) = poly[k], poly[(k + 1) % n]
        ex, ey = x2 - x1, y2 - y1
        length = math.hypot(ex, ey)
        if length <= 1e-12:
            continue
        nx, ny = ey / length, -ex / length            # outward normal (CCW)
        slack = nx * (x1 - point[0]) + ny * (y1 - point[1])
        if slack < 0:
            return -1.0
        nu = nx * u[0] + ny * u[1]
        if nu > 1e-12:
            best = min(best, slack / nu)
    return best


def geom_accel(scene, u):
    """Jaesung's stack check as a critical acceleration (no side support)."""
    path = scene.closure(scene.new, (scene.below,))
    worst = A_CAP_G
    for k in path:
        group = scene.closure(k, (scene.above,))
        mass = sum(scene.bodies[q].mass for q in group) or 1e-9
        cogs = [_shifted_cog(scene.bodies[q], u) for q in group]
        c = [sum(scene.bodies[q].mass * cg[d] for q, cg in zip(group, cogs)) / mass for d in range(3)]
        pts = []
        for r, _, _ in scene.bottom_regions(k):
            pts.extend(r.corners())
        poly = convex_hull(pts)
        h = c[2] - scene.lo[k, 2]
        d = _ray_exit(poly, c, u)
        if d <= 0:
            return 0.0
        worst = min(worst, d / h if h > 1e-9 else A_CAP_G)
    return worst


def lp_accel(scene, system, u):
    """Maximum lateral acceleration (g) along ``u`` that the system can carry.

    Returns (a, status): status "ok", "static_unstable" (no equilibrium even
    at a = 0) or "solver_error"."""
    from scipy.optimize import linprog
    from scipy.sparse import coo_matrix

    local = {b: k for k, b in enumerate(system)}
    cogs = {b: _shifted_cog(scene.bodies[b], u) for b in system}
    rows, cols, vals = [], [], []
    ub_rows, ub_cols, ub_vals = [], [], []
    n_ub = 0
    nvar = 1                     # x[0] = a
    lower, upper = [0.0], [A_CAP_G]

    def add_force(body, point, var_terms):
        """var_terms: [(var, (Fx, Fy, Fz) coefficients)] of a force at point on body."""
        if body is None:
            return
        base = 6 * local[body]
        c = cogs[body]
        rx, ry, rz = point[0] - c[0], point[1] - c[1], point[2] - c[2]
        for var, (fx, fy, fz) in var_terms:
            for r, v in ((0, fx), (1, fy), (2, fz),
                         (3, ry * fz - rz * fy), (4, rz * fx - rx * fz), (5, rx * fy - ry * fx)):
                if v:
                    rows.append(base + r)
                    cols.append(var)
                    vals.append(v)

    for i in system:
        z = scene.lo[i, 2]
        for rect, j, mu in scene.bottom_regions(i):
            for (x, y) in rect.corners():
                fn, fx, fy = nvar, nvar + 1, nvar + 2
                nvar += 3
                lower += [0.0, None, None]
                upper += [None, None, None]
                p = (x, y, z)
                add_force(i, p, [(fx, (1, 0, 0)), (fy, (0, 1, 0)), (fn, (0, 0, 1))])
                add_force(j, p, [(fx, (-1, 0, 0)), (fy, (0, -1, 0)), (fn, (0, 0, -1))])
                for th in _OCT_ANGLES:
                    ub_rows += [n_ub, n_ub, n_ub]
                    ub_cols += [fx, fy, fn]
                    ub_vals += [math.cos(th), math.sin(th), -mu * _OCT_K]
                    n_ub += 1
    # side contacts (compression only, no friction)
    for i in system:
        for j in scene.side(i):
            if j <= i or j not in local:
                continue
            bi, bj = scene.bodies[i], scene.bodies[j]
            for ax, other in ((0, 1), (1, 0)):
                if scene.hi[i, ax] <= scene.lo[j, ax] + LEN_EPS:
                    fi, fj, sign = scene.hi[i, ax], scene.lo[j, ax], 1.0   # j on the + side of i
                elif scene.hi[j, ax] <= scene.lo[i, ax] + LEN_EPS:
                    fi, fj, sign = scene.lo[i, ax], scene.hi[j, ax], -1.0
                else:
                    continue
                t = max(bi.tol, bj.tol)
                o0 = max(scene.lo[i, other], scene.lo[j, other]) + t
                o1 = min(scene.hi[i, other], scene.hi[j, other]) - t
                z0 = max(scene.lo[i, 2], scene.lo[j, 2]) + t
                z1 = min(scene.hi[i, 2], scene.hi[j, 2]) - t
                if o1 - o0 <= LEN_EPS or z1 - z0 <= LEN_EPS:
                    continue
                n = [0.0, 0.0, 0.0]
                n[ax] = sign
                for o in (o0, o1):
                    for zz in (z0, z1):
                        s = nvar
                        nvar += 1
                        lower.append(0.0)
                        upper.append(None)
                        pi, pj = [0.0, 0.0, zz], [0.0, 0.0, zz]
                        pi[ax], pj[ax] = fi, fj
                        pi[other] = pj[other] = o
                        add_force(j, pj, [(s, tuple(n))])
                        add_force(i, pi, [(s, tuple(-v for v in n))])
    # body forces: m * (a u_x, a u_y, -1)
    b_eq = np.zeros(6 * len(system))
    for b, k in local.items():
        m = scene.bodies[b].mass
        for r, v in ((0, m * u[0]), (1, m * u[1])):
            if v:
                rows.append(6 * k + r)
                cols.append(0)
                vals.append(v)
        b_eq[6 * k + 2] = m
    a_eq = coo_matrix((vals, (rows, cols)), shape=(6 * len(system), nvar)).tocsr()
    a_ub = coo_matrix((ub_vals, (ub_rows, ub_cols)), shape=(max(n_ub, 1), nvar)).tocsr()
    b_ub = np.zeros(max(n_ub, 1))
    cost = np.zeros(nvar)
    cost[0] = -1.0
    res = linprog(cost, A_ub=a_ub, b_ub=b_ub, A_eq=a_eq, b_eq=b_eq,
                  bounds=list(zip(lower, upper)), method="highs")
    if res.status == 0:
        return float(res.x[0]), "ok"
    if res.status == 2:
        return 0.0, "static_unstable"
    return 0.0, "solver_error"


def assess(model, box, pose, rect, dz, tol, uncertain, *, lp_all=False):
    """Geometric and (where needed) LP critical accelerations per direction."""
    cfg = model.config.constraints.lateral
    scene = Scene(model, box, pose, rect, dz, tol, uncertain)
    dirs = directions(cfg.directions)
    out = Assessment(system=[], directions=dirs, a_geom=[geom_accel(scene, u) for u in dirs])
    need = [k for k, g in enumerate(out.a_geom) if lp_all or g < cfg.band_high_g]
    out.a_lp = [None] * len(dirs)
    out.lp_status = [None] * len(dirs)
    if need:
        out.system = scene.system()
        for k in need:
            out.a_lp[k], out.lp_status[k] = lp_accel(scene, out.system, dirs[k])
    out.scene = scene
    return out


def decide(assessment, cfg):
    """Cascade verdict from an assessment (no simulation)."""
    lp = [a for a in assessment.a_lp if a is not None]
    if not lp:
        return LateralResult(True, "geom", min(assessment.a_geom), {})
    a = min(lp)
    if a >= cfg.band_high_g:
        return LateralResult(True, "lp", a, {})
    if a < cfg.band_low_g:
        return LateralResult(False, "lp", a, {"lp_status": sorted({s for s in assessment.lp_status if s})})
    return None   # borderline


def check(model, box, pose, rect, dz, tol, uncertain):
    """Hard-mask entry point: geom -> LP -> short tilt test."""
    cfg = model.config.constraints.lateral
    asm = assess(model, box, pose, rect, dz, tol, uncertain)
    verdict = decide(asm, cfg)
    if verdict is not None:
        return verdict
    a = min(x for x in asm.a_lp if x is not None)
    if cfg.borderline == "accept":
        return LateralResult(True, "borderline_accept", a, {})
    if cfg.borderline == "simulate":
        try:
            from . import lateral_sim
        except ImportError:          # pybullet missing -> conservative
            return LateralResult(False, "borderline_reject", a, {"reason": "pybullet unavailable"})
        res = lateral_sim.short_tilt_test(asm.scene, asm.system, worst_directions(asm, cfg.sim_directions), cfg)
        return LateralResult(res["passed"], "sim", a, res)
    return LateralResult(False, "borderline_reject", a, {})


def worst_directions(asm, count):
    """Lowest LP directions; ties (e.g. all 0) broken by the geometric value."""
    order = sorted((x if x is not None else math.inf, g, k)
                   for k, (x, g) in enumerate(zip(asm.a_lp, asm.a_geom)))
    return [asm.directions[k] for _, _, k in order[:count]]


__all__ = ["Assessment", "Body", "LateralResult", "Scene", "assess", "check", "decide",
           "deck_rects", "directions", "geom_accel", "lp_accel", "worst_directions"]
