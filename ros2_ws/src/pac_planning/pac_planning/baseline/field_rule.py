"""COMPARISON BASELINE, not the AHEAD algorithm: field-practice rules, no look-ahead, no search, no learning.

Stage 4 (what to do with the box), ``FieldRulePolicy``:
    1. the current box has a safe spot           -> PLACE_CURRENT
    2. otherwise a buffered box has a safe spot  -> RETRIEVE_BUFFER, oldest first (FIFO)
    3. otherwise a buffer slot is free           -> BUFFER_CURRENT
    4. nothing is possible (buffer full)         -> the world closes the pallet and
       starts a new one (``field_rule_highlevel_config``: no partial repack, no
       early close)

Stage 5-3 (where), ``FieldRulePlacer``, among the 5-2-valid candidates:
    1. lowest spot (tops within the 5-2 height tolerance count as one level)
    2. row farthest from the robot, so the arm never reaches over placed boxes
       (row = the footprint's far edge, within ``row_tol_m``)
    3. most side contact: share of the footprint perimeter touching a
       neighbouring box or the pallet edge
    4. left-most seen from the robot, then candidate id (deterministic)

Safety is not part of the rules: the candidates come from the team's 5-1
generator and 5-2 Hard Mask unchanged; the rules only order safe options.

Offline (``pac_highlevel.PalletizingWorld``)::

    hl = field_rule_highlevel_config(hl)
    world = PalletizingWorld(arrivals, pallet, catalog, cand_config, hl, placer=FieldRulePlacer())
    out = run_policy(world, FieldRulePolicy())

Runtime (``pac_runtime.RuntimeCore``, stage 6 checks the ranked list)::

    placer = FieldRulePlacer(config)
    core = RuntimeCore(cell, cand, field_rule_highlevel_config(hl), rt, robot,
                       field_rule_loaded_policy(), ranker=placer.rank)
"""

from dataclasses import dataclass, replace
from pathlib import Path

from pac_candidates.geometry import rotated_dims

# facing direction of a robot standing on that side of the pallet (pallet frame)
_FACING = {"-y": (0.0, 1.0), "+y": (0.0, -1.0), "-x": (1.0, 0.0), "+x": (-1.0, 0.0)}
_DEFAULT_LEVEL_TOL_M = 0.003  # 5-2 height tolerance when no backend is given


@dataclass(frozen=True)
class FieldRuleConfig:
    robot_side: str = "-y"          # side of the pallet the robot stands on
    level_tol_m: float = 0.0        # same level; 0 = the 5-2 height tolerance (3 mm)
    row_tol_m: float = 0.01         # same row (far edges within this distance)
    contact_gap_m: float = 0.015    # faces this close touch (5-2 neighbour gap is 8 mm)

    def __post_init__(self):
        if self.robot_side not in _FACING:
            raise ValueError(f"robot_side must be one of {sorted(_FACING)}")
        for name in ("level_tol_m", "row_tol_m", "contact_gap_m"):
            v = getattr(self, name)
            if isinstance(v, bool) or not 0.0 <= v <= 0.1:
                raise ValueError(f"{name} must be in [0, 0.1] m")


def field_rule_config_from_dict(data):
    data = dict(data or {})
    unknown = set(data) - set(FieldRuleConfig.__dataclass_fields__)
    if unknown:
        raise ValueError(f"unknown baseline_field_rule keys: {sorted(unknown)}")
    return FieldRuleConfig(**data)


def load_field_rule_config(path=None):
    """``config/donghan/baseline/field_rule.yaml`` (top key ``baseline_field_rule``)."""
    if path is None:
        return FieldRuleConfig()
    import yaml

    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    return field_rule_config_from_dict(data.get("baseline_field_rule", data))


def field_rule_highlevel_config(hl):
    """Stage-4 world settings of the rule-based algorithm: no partial repack,
    and the pallet is closed only when nothing can be placed or buffered."""
    return replace(
        hl,
        repack=replace(hl.repack, enabled=False),
        close=replace(hl.close, mode="fill", fill_before_buffer=0.0),
    )


# ---------------------------------------------------------------- stage 4
class FieldRulePolicy:
    """``policy(world) -> HighLevelAction`` (same contract as ``RulePolicy``)."""

    name = "baseline_field_rule"

    def __call__(self, world):
        from pac_highlevel.actions import ActionType, HighLevelAction

        mask = world.action_mask()
        if world.current is not None and mask[0]:
            return HighLevelAction(ActionType.PLACE_CURRENT)
        fits = [(e.stored_at, i) for i, e in enumerate(world.buffer) if e is not None and mask[2 + i]]
        if fits:
            return HighLevelAction(ActionType.RETRIEVE_BUFFER, min(fits)[1])
        if world.current is not None and mask[1]:
            return HighLevelAction(ActionType.BUFFER_CURRENT)
        raise RuntimeError("world asked for a decision with an empty mask")


def field_rule_loaded_policy():
    """The policy wrapped for ``pac_highlevel.HighLevelDecider`` / ``RuntimeCore``."""
    from pac_highlevel.runtime import LoadedPolicy

    return LoadedPolicy(FieldRulePolicy.name, FieldRulePolicy())


# ---------------------------------------------------------------- stage 5-3
@dataclass(frozen=True)
class _Row:
    candidate: object
    z: float
    depth: float      # far edge along the robot's facing direction (larger = farther)
    contact: float    # touching share of the footprint perimeter
    left: float       # left edge seen from the robot (larger = more left)


def _footprint(size, pose):
    dx, dy, dz = rotated_dims(size, pose.yaw)
    return pose.x, pose.y, pose.x + dx, pose.y + dy, pose.z, pose.z + dz


def side_contact(size, pose, pallet, gap, level_tol):
    """Share of the footprint perimeter touching a placed box side or the pallet edge."""
    x0, y0, x1, y1, z0, z1 = _footprint(size, pose)
    X, Y = pallet.size.x, pallet.size.y
    # [x-faces at x0 / x1 run along y, y-faces at y0 / y1 run along x]
    touch = [y1 - y0 if x0 <= gap else 0.0, y1 - y0 if x1 >= X - gap else 0.0,
             x1 - x0 if y0 <= gap else 0.0, x1 - x0 if y1 >= Y - gap else 0.0]
    for b in pallet.boxes:
        bx0, by0, bx1, by1, bz0, bz1 = _footprint(b.size, b.pose)
        if min(z1, bz1) - max(z0, bz0) <= level_tol:
            continue  # below or above, not beside
        along_y = max(0.0, min(y1, by1) - max(y0, by0))
        along_x = max(0.0, min(x1, bx1) - max(x0, bx0))
        if -1e-9 <= x0 - bx1 <= gap:
            touch[0] += along_y
        if -1e-9 <= bx0 - x1 <= gap:
            touch[1] += along_y
        if -1e-9 <= y0 - by1 <= gap:
            touch[2] += along_x
        if -1e-9 <= by0 - y1 <= gap:
            touch[3] += along_x
    sides = (y1 - y0, y1 - y0, x1 - x0, x1 - x0)
    return sum(min(t, s) for t, s in zip(touch, sides)) / sum(sides)


class FieldRulePlacer:
    """World placer ``(valid, box, state, backend) -> candidate`` and runtime
    ranker ``rank(valid, box, state, backend) -> ordered list``."""

    wants_context = True
    name = "baseline_field_rule"

    def __init__(self, config=None):
        self.cfg = config or FieldRuleConfig()
        fx, fy = _FACING[self.cfg.robot_side]
        self._facing = (fx, fy)
        self._left = (-fy, fx)  # facing turned +90 deg

    def _level_tol(self, backend):
        if self.cfg.level_tol_m > 0:
            return self.cfg.level_tol_m
        unc = getattr(getattr(backend, "config", None), "uncertainty", None)
        return unc.height_tolerance_m if unc is not None else _DEFAULT_LEVEL_TOL_M

    def _rows(self, valid, box, state, level_tol):
        pallet = state.pallet
        (fx, fy), (lx, ly) = self._facing, self._left
        rows = []
        for c in valid:
            p = c.target_pose
            x0, y0, x1, y1, _, _ = _footprint(box.size, p)
            corners = ((x0, y0), (x0, y1), (x1, y0), (x1, y1))
            rows.append(_Row(
                c, p.z,
                max(fx * x + fy * y for x, y in corners),
                side_contact(box.size, p, pallet, self.cfg.contact_gap_m, level_tol),
                max(lx * x + ly * y for x, y in corners),
            ))
        return rows

    def _pick(self, rows, level_tol):
        z_min = min(r.z for r in rows)
        rows = [r for r in rows if r.z <= z_min + level_tol]
        far = max(r.depth for r in rows)
        rows = [r for r in rows if r.depth >= far - self.cfg.row_tol_m]
        most = max(r.contact for r in rows)
        rows = [r for r in rows if r.contact >= most - 1e-9]
        return min(rows, key=lambda r: (-r.left, r.candidate.candidate_id))

    def __call__(self, valid, box, state, backend=None):
        if not valid:
            return None
        tol = self._level_tol(backend)
        return self._pick(self._rows(valid, box, state, tol), tol).candidate

    def rank(self, valid, box, state, backend=None):
        """All candidates in rule order (best first), e.g. for stage 6 fallback."""
        tol = self._level_tol(backend)
        rows, ordered = self._rows(valid, box, state, tol), []
        while rows:
            best = self._pick(rows, tol)
            ordered.append(best.candidate)
            rows = [r for r in rows if r is not best]
        return ordered

    def explain(self, valid, box, state, backend=None):
        """Rule values per candidate in rule order (logs, documents)."""
        tol = self._level_tol(backend)
        rows = {r.candidate.candidate_id: r for r in self._rows(valid, box, state, tol)}
        return [{"candidate_id": c.candidate_id, "z": rows[c.candidate_id].z,
                 "far_edge": rows[c.candidate_id].depth, "contact": rows[c.candidate_id].contact,
                 "left_edge": rows[c.candidate_id].left}
                for c in self.rank(valid, box, state, backend)]


__all__ = [
    "FieldRuleConfig",
    "FieldRulePlacer",
    "FieldRulePolicy",
    "load_field_rule_config",
    "field_rule_config_from_dict",
    "field_rule_highlevel_config",
    "field_rule_loaded_policy",
    "side_contact",
]
