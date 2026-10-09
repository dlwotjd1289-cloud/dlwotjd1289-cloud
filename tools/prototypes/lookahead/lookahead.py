"""Stability-first look-ahead palletizing planner (prototype).

Information tiers (what the planner may read):
  0  current box at PICK            measured size + weight (scale + CCTV label)
  1  next k boxes already weighed    exact size, weight and order (conveyor queue)
  2  next m boxes seen upstream      SKU/size and order, weight unknown -> sampled
                                     from the SKU weight range per scenario
  3  everything else                 only the remaining count per SKU -> sampled orders

Decision = rolling horizon: score every root action (place current at one of
the top candidates / retrieve a buffered box / park the current box), roll the
known queue (tiers 1-2) and sampled tier-3 boxes forward with the fast base
policy, keep the action with the lowest expected cost (mean + CVaR blend), and
execute only that action. The plan is recomputed whenever a new box is labelled
or the pallet state changes.

Scoring (lower cost = better) puts stability first, non-linearly:
  phi(x; t, a_lo, a_hi) = a_lo * min(x, t) + a_hi * max(0, x - t),  a_lo >> a_hi
so shortfall below the target t is expensive, surplus above t is worth little.
S = stability index in [0, 1] from support ratio, interlock (rests on >= 2
boxes), tilt margin (CoG inside the support polygon with gravity tilted by
0.2 g) and side support. L = load margin = 1 - max load ratio.

Safety is never decided here: every candidate comes from the team hard mask
(per_box heavy-on-light, crush, overlap, height, LBCP); this module only ranks.
"""
from __future__ import annotations

import math
import os
import random
import sys
from collections import Counter
from dataclasses import dataclass, field, replace
from pathlib import Path

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

REPO = Path.home() / "AHEAD/pac2026_integrated"
sys.path.insert(0, str(REPO / "tools/highlevel/scripts"))
import _common  # noqa: E402,F401  (team path bootstrap)

import numpy as np  # noqa: E402
from pac_common import BoxStatus, PlacedBox, SystemState, PalletState, InventoryState  # noqa: E402
from pac_candidates.geometry import rotated_dims  # noqa: E402
from pac_highlevel.actions import ActionType, HighLevelAction  # noqa: E402

CELL = 0.02
TILT_G = 0.2
EFF_FEATURES = ("z", "top", "rough", "void", "corner", "block", "floor")


@dataclass
class Params:
    # stability-first, non-linear (fixed by design, not tuned for pallet count)
    s_target: float = 0.75
    s_lo: float = 20.0          # cost per unit of S below target
    s_hi: float = 2.0           # cost per unit of S above target (diminishing)
    l_target: float = 0.5
    l_lo: float = 8.0
    l_hi: float = 0.5
    s_weights: tuple = (0.20, 0.25, 0.40, 0.15)   # support, interlock, shock, side
    a_target_g: float = 0.3     # lateral acceleration a box should survive (forklift handling)
    mu: float = 0.4             # carton-on-carton friction (sliding limit, g)
    # efficiency weights (tunable)
    eff: tuple = (2.0, 2.0, 2.0, 1.5, 4.0, 1.5, 0.5)
    # high-level / look-ahead
    theta_buffer: float = 0.5   # cost of parking the current box
    theta_retrieve: float = 0.1
    preview_k: int = 3          # tier 1
    preview_m: int = 2          # tier 2
    horizon: int = 4            # boxes rolled out after the root action
    scenarios: int = 4
    top_k: int = 4              # root placement candidates
    p_block: float = 6.0        # cost of a box with no valid place in a rollout
    cvar_kappa: float = 0.3     # cost = (1-k) mean + k * worst
    gamma: float = 0.6          # weight of the rolled-out future vs the root step
    lookahead: bool = True
    seed: int = 0


def _rect_overlap(a, b):
    return max(0.0, min(a[1], b[1]) - max(a[0], b[0]))


def _signed_dist_convex(poly, pt):
    """>0 inside: min distance from pt to the edges of a CCW/CW convex polygon."""
    n = len(poly)
    if n < 3:
        return -1.0
    area = sum(poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1] for i in range(n))
    sgn = 1.0 if area > 0 else -1.0
    best = math.inf
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        ex, ey = x2 - x1, y2 - y1
        L = math.hypot(ex, ey) or 1e-12
        d = sgn * (ex * (pt[1] - y1) - ey * (pt[0] - x1)) / L
        best = min(best, d)
    return best


class Scorer:
    """Fast per-candidate cost (also the base policy inside rollouts)."""

    def __init__(self, params: Params, sku_ranges):
        self.p = params
        self.ranges = sku_ranges

    # --- weight awareness from tiers 1-3 (never the hidden order) ---
    def heavier_share(self, w, pool_counts, known_weights):
        total, heavy = 0.0, 0.0
        for sku, n in pool_counts.items():
            lo, hi = self.ranges[sku]
            p = 1.0 if hi <= lo and hi > w + 0.5 else (0.0 if hi <= lo else min(1.0, max(0.0, (hi - (w + 0.5)) / (hi - lo))))
            total += n
            heavy += n * p
        for kw in known_weights:
            total += 1
            heavy += float(kw > w + 0.5)
        return heavy / total if total else 0.0

    def stability(self, cand, box, verdict, grid, pallet):
        m = verdict.details["metrics"]
        p = cand.target_pose
        dx, dy, dz = rotated_dims(box.size, p.yaw)
        support = float(m.get("support_ratio", 1.0))
        on_floor = bool(m.get("on_floor", p.z < 1e-6))
        shares = list((m.get("supporter_shares") or {}).values())
        if on_floor:
            interlock = 1.0
        elif len(shares) >= 2:
            interlock = min(1.0, 2.0 * (1.0 - max(shares)))   # two equal halves -> 1
        else:
            interlock = 0.0
        # side support per direction: walls or neighbours reaching half the box height
        nx, ny = grid.shape
        i0, j0 = max(0, int(round(p.x / CELL))), max(0, int(round(p.y / CELL)))
        i1, j1 = min(nx, max(i0 + 1, int(round((p.x + dx) / CELL)))), min(ny, max(j0 + 1, int(round((p.y + dy) / CELL))))
        half = p.z + 0.5 * dz
        def side_frac(s):
            return 1.0 if s is None else float(np.mean(s >= half))
        sides = {(-1, 0): side_frac(grid[i0 - 1, j0:j1] if i0 > 0 else None),
                 (1, 0): side_frac(grid[i1, j0:j1] if i1 < nx else None),
                 (0, -1): side_frac(grid[i0:i1, j0 - 1] if j0 > 0 else None),
                 (0, 1): side_frac(grid[i0:i1, j1] if j1 < ny else None)}
        side = float(np.mean(list(sides.values())))
        # critical lateral acceleration (g): tipping over the support polygon edge or sliding,
        # unless a wall / neighbour blocks that direction
        poly = m.get("support_polygon") or ()
        if on_floor or len(poly) < 3:
            poly = ((p.x, p.y), (p.x + dx, p.y), (p.x + dx, p.y + dy), (p.x, p.y + dy))
        cx, cy, h = p.x + dx / 2, p.y + dy / 2, dz / 2
        a_c = 1.0
        for (ux, uy), blocked in sides.items():
            if blocked >= 0.5:
                continue
            lo, hi = 0.0, 1.0
            if _signed_dist_convex(poly, (cx + ux * hi * h, cy + uy * hi * h)) > 0:
                a_tip = hi
            else:
                for _ in range(10):
                    mid = 0.5 * (lo + hi)
                    if _signed_dist_convex(poly, (cx + ux * mid * h, cy + uy * mid * h)) > 0:
                        lo = mid
                    else:
                        hi = mid
                a_tip = lo
            a_c = min(a_c, a_tip, self.p.mu)
        tilt = float(np.clip(a_c / self.p.a_target_g, 0.0, 1.0))   # "shock" component
        self.last_ac = a_c
        comps = (support, interlock, tilt, side)
        S = float(np.dot(self.p.s_weights, comps))
        L = float(np.clip(1.0 - float(m.get("max_load_ratio", 0.0)), 0.0, 1.0))
        return S, L, comps, (i0, i1, j0, j1)

    def cost(self, cand, box, verdict, grid, pallet, heavy_share):
        p = cand.target_pose
        dx, dy, dz = rotated_dims(box.size, p.yaw)
        H, Lx, Ly = pallet.z, pallet.x, pallet.y
        S, L, comps, (i0, i1, j0, j1) = self.stability(cand, box, verdict, grid, pallet)
        P = self.p
        phi_s = P.s_lo * min(S, P.s_target) + P.s_hi * max(0.0, S - P.s_target)
        phi_l = P.l_lo * min(L, P.l_target) + P.l_hi * max(0.0, L - P.l_target)
        under = grid[i0:i1, j0:j1]
        void = float(np.sum(np.clip(p.z - under, 0.0, None)) * CELL * CELL) / max(1e-9, dx * dy * dz)
        after = grid.copy()
        after[i0:i1, j0:j1] = p.z + dz
        top = p.z + dz
        area = dx * dy / (Lx * Ly)
        open_top = 1.0 if top < H - 0.09 else 0.0
        eff = np.array([
            p.z / H, top / H, float(after.std()) / H, void, (p.x + p.y) / (Lx + Ly),
            area * heavy_share * open_top, area * heavy_share * (1.0 if p.z < 1e-6 else 0.0),
        ])
        return float(np.dot(P.eff, eff) - phi_s - phi_l), {"S": S, "L": L, "comps": comps}


def grid_of(boxes, pallet):
    g = np.zeros((int(round(pallet.x / CELL)), int(round(pallet.y / CELL))))
    for b in boxes:
        dx, dy, dz = rotated_dims(b.size, b.pose.yaw)
        i0, j0 = max(0, int(round(b.pose.x / CELL))), max(0, int(round(b.pose.y / CELL)))
        i1, j1 = min(g.shape[0], max(i0 + 1, int(round((b.pose.x + dx) / CELL)))), min(g.shape[1], max(j0 + 1, int(round((b.pose.y + dy) / CELL))))
        np.maximum(g[i0:i1, j0:j1], b.pose.z + dz, out=g[i0:i1, j0:j1])
    return g


class Planner:
    """Placer (wants_context) + high-level policy for the team PalletizingWorld."""

    wants_context = True
    name = "lookahead"

    def __init__(self, params: Params, sku_ranges):
        self.p = params
        self.scorer = Scorer(params, sku_ranges)
        self.ranges = sku_ranges
        self.override = {}
        self.best = {}            # box_id -> (cost, candidate)
        self.rng = random.Random(params.seed)
        self.world = None
        self.stats = Counter()

    # ---------------- low level (placer) ----------------
    def _ranked(self, box, state, backend, pool_counts, known_weights, limit=None):
        cset = backend.candidate_set(box, state)
        if not cset.valid:
            return []
        grid = grid_of(state.pallet.boxes, state.pallet.size)
        hs = self.scorer.heavier_share(box.weight_kg, pool_counts, known_weights)
        out = []
        for c in cset.valid:
            v = backend.validate_constraints(box, c, state)
            if not v.success:
                continue
            cost, info = self.scorer.cost(c, box, v, grid, state.pallet.size, hs)
            out.append((cost, c.candidate_id, c, info))
        out.sort(key=lambda t: (t[0], t[1]))
        self.stats["mask_calls"] += 1
        return out[:limit] if limit else out

    def __call__(self, valid, box, state, backend):
        if not valid:
            return None
        o = self.override.pop(box.box_id, None)
        if o is not None and any(c.candidate_id == o.candidate_id for c in valid):
            return next(c for c in valid if c.candidate_id == o.candidate_id)
        pool, known = self._future_pool()
        r = self._ranked(box, state, backend, pool, known, limit=1)
        if not r:
            return None
        self.best[box.box_id] = (r[0][0], r[0][2])
        return r[0][2]

    # ---------------- information tiers ----------------
    def _tiers(self):
        w = self.world
        if w is None:
            return [], [], Counter()
        nxt = w.arrivals[w.next_arrival:]
        t1 = [a.box for a in nxt[: self.p.preview_k]]
        t2 = [a.box for a in nxt[self.p.preview_k: self.p.preview_k + self.p.preview_m]]
        pool = Counter({k: v for k, v in w.remaining.items() if v > 0})   # unseen = not yet pulled
        for b in t1 + t2:
            pool[b.sku_id] -= 1
        pool = Counter({k: v for k, v in pool.items() if v > 0})
        return t1, t2, pool

    def _future_pool(self):
        t1, t2, pool = self._tiers()
        pool = Counter(pool)
        for b in t2:
            pool[b.sku_id] += 1          # tier 2: weight unknown -> treated by range
        known = [b.weight_kg for b in t1]
        for e in (self.world.buffer if self.world else []):
            if e is not None:
                known.append(e.arrival.box.weight_kg)
        return pool, known

    def _sample_box(self, sku, idx):
        """A tier-3 box: SKU known from the remaining counts, weight sampled from its range."""
        lo, hi = self.ranges[sku]
        cat = self.world.catalog[sku]
        template = self.world.arrivals[0].box
        return replace(template, box_id=f"SIM{idx:04d}", sku_id=sku, size=cat.size,
                       weight_kg=round(self.rng.uniform(lo, hi), 3),
                       allowed_yaws_rad=tuple(cat.allowed_yaws_rad), status=BoxStatus.READY_FOR_PICK)

    def _scenario_queue(self, extra_first=()):
        """One sampled future: tier1 exact, tier2 with sampled weight, then tier3 by remaining counts."""
        t1, t2, pool = self._tiers()
        q = list(extra_first) + list(t1)
        for i, b in enumerate(t2):
            lo, hi = self.ranges[b.sku_id]
            q.append(replace(b, box_id=f"T2_{i}_{b.box_id}", weight_kg=round(self.rng.uniform(lo, hi), 3)))
        bag = [s for s, n in pool.items() for _ in range(n)]
        self.rng.shuffle(bag)
        for j, sku in enumerate(bag[: max(0, self.p.horizon - len(q))]):
            q.append(self._sample_box(sku, j))
        return q[: self.p.horizon]

    # ---------------- rollout ----------------
    def _hyp_state(self, base, placed, hyp_boxes, current=None):
        """Hypothetical snapshot: every placed box is tracked as PLACED at its pose."""
        tracked = dict(base.inventory.tracked_boxes)
        for pb in placed:
            src = tracked.get(pb.box_id) or hyp_boxes.get(pb.box_id)
            if src is not None and (src.status != BoxStatus.PLACED or src.pose != pb.pose):
                tracked[pb.box_id] = replace(src, pose=pb.pose, status=BoxStatus.PLACED)
        if current is not None:
            tracked[current.box_id] = current
        return SystemState(base.state_version, base.stamp_sec,
                           replace(base.pallet, boxes=tuple(placed)),
                           replace(base.inventory, tracked_boxes=tracked))

    def _rollout(self, base_state, placed, queue, backend, hyp_boxes):
        cost, blocked = 0.0, 0
        placed = list(placed)
        hyp = dict(hyp_boxes)
        pool, known = self._future_pool()
        for b in queue:
            hyp[b.box_id] = b
            st = self._hyp_state(base_state, placed, hyp, current=b)
            r = self._ranked(b, st, backend, pool, known, limit=1)
            if not r:
                blocked += 1
                continue
            c = r[0][2]
            cost += r[0][0]
            placed.append(PlacedBox(b.box_id, b.sku_id, b.size, b.weight_kg, c.target_pose))
        return cost + self.p.p_block * blocked

    def _evaluate(self, root_cost, placed_after, base_state, backend, extra_first=(), hyp_boxes=None):
        vals = []
        for _ in range(self.p.scenarios):
            q = self._scenario_queue(extra_first)
            vals.append(self._rollout(base_state, placed_after, q, backend, hyp_boxes or {}))
        mean = float(np.mean(vals))
        worst = float(np.max(vals))
        fut = (1 - self.p.cvar_kappa) * mean + self.p.cvar_kappa * worst
        return root_cost + self.p.gamma * fut

    # ---------------- high level (policy) ----------------
    def decide(self, world):
        self.world = world
        mask = world.action_mask()
        current, buffered = world.options()      # fills self.best via the placer
        state = world.state()
        backend = world.backend()
        placed = list(state.pallet.boxes)
        options = []
        for i, e in enumerate(world.buffer):      # stale buffer boxes first (team rule)
            if e is not None and mask[2 + i] and world.decisions - e.stored_at >= world.config.rules.max_buffer_age:
                return HighLevelAction(ActionType.RETRIEVE_BUFFER, i)
        cur = world.current.box if world.current is not None else None
        pool, known = self._future_pool()
        if mask[0] and cur is not None:
            ranked = self._ranked(cur, state, backend, pool, known, limit=self.p.top_k)
            for cost, _, c, _ in ranked:
                pb = PlacedBox(cur.box_id, cur.sku_id, cur.size, cur.weight_kg, c.target_pose)
                q = cost if not self.p.lookahead else self._evaluate(cost, placed + [pb], state, backend,
                                                                      hyp_boxes={cur.box_id: cur})
                options.append((q, "place", c))
        for i, e in enumerate(world.buffer):
            if e is None or not mask[2 + i]:
                continue
            b = e.arrival.box
            ranked = self._ranked(b, state, backend, pool, known, limit=1)
            if not ranked:
                continue
            cost, _, c, _ = ranked[0]
            pb = PlacedBox(b.box_id, b.sku_id, b.size, b.weight_kg, c.target_pose)
            extra = (cur,) if cur is not None else ()
            q = cost + self.p.theta_retrieve if not self.p.lookahead else \
                self._evaluate(cost + self.p.theta_retrieve, placed + [pb], state, backend, extra,
                               hyp_boxes={b.box_id: b})
            options.append((q, ("retrieve", i), c))
        if mask[1] and cur is not None:
            # park the current box: it comes back later, right after the next known box
            if self.p.lookahead:
                # the parked box re-enters the rollout after the next known box
                q = self._evaluate(self.p.theta_buffer, placed, state, backend, ())
            else:
                q = self.p.theta_buffer + (self.best.get(cur.box_id, (0.0,))[0] if mask[0] else 0.0)
            options.append((q, "buffer", None))
        if not options:
            raise RuntimeError("empty mask")
        q, kind, c = min(options, key=lambda t: (t[0], str(t[1])))
        self.stats[kind if isinstance(kind, str) else "retrieve"] += 1
        if kind == "place":
            self.override[cur.box_id] = c
            world._options = None
            return HighLevelAction(ActionType.PLACE_CURRENT)
        if kind == "buffer":
            return HighLevelAction(ActionType.BUFFER_CURRENT)
        slot = kind[1]
        self.override[world.buffer[slot].arrival.box.box_id] = c
        world._options = None
        return HighLevelAction(ActionType.RETRIEVE_BUFFER, slot)

    def policy(self):
        return lambda world: self.decide(world)
