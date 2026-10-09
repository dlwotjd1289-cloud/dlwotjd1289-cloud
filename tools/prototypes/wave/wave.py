"""WAVE prototype: Weight-Aware Value Evaluator placer for the team's
PalletizingWorld (taehyeon pac_highlevel) on the team's hard mask.

Only the low-level choice among hard-mask-valid candidates changes; candidate
generation, the hard mask (safety), the high-level RulePolicy (buffer / close)
and the world are the team's code, unchanged. Scores are a linear sum of
interpretable features (Tetris-style evaluator); weights are tuned offline by
the cross-entropy method in tune.py.

Known-in-advance information used (mission brief): SKU sizes and weight ranges
and the remaining count per SKU. The arrival order is never read.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

REPO = Path.home() / "AHEAD/pac2026_integrated"
sys.path.insert(0, str(REPO / "tools/highlevel/scripts"))
import _common  # noqa: E402  (team path bootstrap)

import numpy as np  # noqa: E402
from pac_candidates.geometry import rotated_dims  # noqa: E402

FEATURES = ("z", "top", "rough", "block", "floor", "contact", "corner", "support", "void", "exposed")
CELL = 0.02


def _footprint(grid, x, y, dx, dy):
    nx, ny = grid.shape
    i0, j0 = max(0, int(round(x / CELL))), max(0, int(round(y / CELL)))
    i1, j1 = min(nx, max(i0 + 1, int(round((x + dx) / CELL)))), min(ny, max(j0 + 1, int(round((y + dy) / CELL))))
    return i0, i1, j0, j1


class WavePlacer:
    wants_context = True

    def __init__(self, weights, sku_ranges, tol_kg=0.5):
        w = list(weights)
        if len(w) < len(FEATURES):                          # older weight files: new features off
            w = w + [0.0] * (len(FEATURES) - len(w))
        self.w = np.asarray(w, dtype=float)
        self.ranges = sku_ranges            # sku -> (min_kg, max_kg), known from the catalog
        self.H = 1.5
        self.tol = tol_kg
        self.last_features = None
        self.last_score = {}

    # probability that one unseen box of this SKU weighs more than w (uniform in range)
    def _p_heavier(self, sku, w):
        lo, hi = self.ranges[sku]
        if hi <= lo:
            return float(hi > w + self.tol)
        return float(min(1.0, max(0.0, (hi - (w + self.tol)) / (hi - lo))))

    def _heavier_share(self, box, state):
        rem = state.inventory.remaining_by_sku or {}
        total, heavy = 0.0, 0.0
        for sku, n in rem.items():
            total += n
            heavy += n * self._p_heavier(sku, box.weight_kg)
        for b in state.inventory.tracked_boxes.values():   # buffered boxes: weights are measured
            if getattr(b, "status", None) is not None and b.status.name == "BUFFERED":
                total += 1
                heavy += float(b.weight_kg > box.weight_kg + self.tol)
        return heavy / total if total else 0.0

    def features(self, cand, box, state, grid, heavy_share):
        p = cand.target_pose
        dx, dy, dz = rotated_dims(box.size, p.yaw)
        Lx, Ly = state.pallet.size.x, state.pallet.size.y
        i0, i1, j0, j1 = _footprint(grid, p.x, p.y, dx, dy)
        under = grid[i0:i1, j0:j1]
        support = float(np.mean(np.abs(under - p.z) < 0.004)) if under.size else 1.0
        void = float(np.sum(np.clip(p.z - under, 0.0, None)) * CELL * CELL) / max(1e-9, dx * dy * dz)
        after = grid.copy()
        after[i0:i1, j0:j1] = p.z + dz
        top = p.z + dz
        area = dx * dy / (Lx * Ly)
        open_top = 1.0 if top < self.H - 0.09 else 0.0       # 0.09 m = smallest box height
        # side contact: walls or neighbours reaching at least half the box height
        lo_h = p.z + 0.5 * dz
        nx, ny = grid.shape
        sides = []
        for side in (
            (grid[i0 - 1, j0:j1] if i0 > 0 else None),
            (grid[i1, j0:j1] if i1 < nx else None),
            (grid[i0:i1, j0 - 1] if j0 > 0 else None),
            (grid[i0:i1, j1] if j1 < ny else None),
        ):
            sides.append(1.0 if side is None else float(np.mean(side >= lo_h)))
        contact = float(np.mean(sides))
        return np.array([
            p.z / self.H,
            top / self.H,
            float(after.std()) / self.H,
            area * heavy_share * open_top,                  # top face that heavier future boxes cannot use
            area * heavy_share * (1.0 if p.z < 1e-6 else 0.0),  # floor used by a box lighter than what is coming
            -contact,
            (p.x + p.y) / (Lx + Ly),
            -support,
            void,
            (top / self.H) * (1.0 - contact),               # high box with open sides: lateral risk
        ])

    def __call__(self, valid, box, state, backend):
        if not valid:
            return None
        self.H = state.pallet.size.z
        grid = np.zeros((int(round(state.pallet.size.x / CELL)), int(round(state.pallet.size.y / CELL))))
        for b in state.pallet.boxes:
            ddx, ddy, ddz = rotated_dims(b.size, b.pose.yaw)
            i0, i1, j0, j1 = _footprint(grid, b.pose.x, b.pose.y, ddx, ddy)
            np.maximum(grid[i0:i1, j0:j1], b.pose.z + ddz, out=grid[i0:i1, j0:j1])
        hs = self._heavier_share(box, state)
        best, best_s = None, math.inf
        for c in valid:
            s = float(self.w @ self.features(c, box, state, grid, hs))
            if s < best_s - 1e-12 or (abs(s - best_s) <= 1e-12 and best is not None and c.candidate_id < best.candidate_id):
                best, best_s = c, s
        self.last_score[box.box_id] = best_s
        return best


# DBLF-equivalent start point: lowest z, then corner
DBLF_LIKE = np.array([10.0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0])


class WavePolicy:
    """High-level choice with the same evaluator: place current / retrieve a
    buffered box / park the current box, whichever has the lowest cost.
    theta_buf = cost of parking the current box, theta_ret = extra cost of
    retrieving (travel). Old buffered boxes are forced out like RulePolicy."""
    name = "wave"

    def __init__(self, placer, theta_buf, theta_ret, max_age=12):
        self.placer, self.tb, self.tr, self.max_age = placer, theta_buf, theta_ret, max_age

    def __call__(self, world):
        from pac_highlevel.actions import ActionType, HighLevelAction
        mask = world.action_mask()
        current, buffered = world.options()
        last = self.placer.last_score
        for i, e in enumerate(world.buffer):
            if e is not None and mask[2 + i] and world.decisions - e.stored_at >= self.max_age:
                return HighLevelAction(ActionType.RETRIEVE_BUFFER, i)
        best = None
        if mask[0]:
            best = (last.get(world.current.box.box_id, 0.0), HighLevelAction(ActionType.PLACE_CURRENT))
        for i, e in enumerate(world.buffer):
            if e is not None and mask[2 + i]:
                c = last.get(e.arrival.box.box_id, 0.0) + self.tr
                if best is None or c < best[0]:
                    best = (c, HighLevelAction(ActionType.RETRIEVE_BUFFER, i))
        if mask[1]:
            c = (last.get(world.current.box.box_id, 0.0) if mask[0] else 0.0) + self.tb
            if best is None or c < best[0]:
                best = (c, HighLevelAction(ActionType.BUFFER_CURRENT))
        if best is None:
            raise RuntimeError("empty mask")
        return best[1]
