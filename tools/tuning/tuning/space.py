"""Search space: parameters of stages 4 and 5-1 that are rules, not physics.

Safety constraints (5-2 hard mask, stage 6) are deliberately NOT tunable:
the tuner may only change how the cell chooses among safe options.
"""

from dataclasses import dataclass, replace


@dataclass(frozen=True)
class Param:
    name: str
    kind: str          # float | int | bool
    low: float = 0.0
    high: float = 1.0
    meaning: str = ""


PARAMS = (
    Param("rules.good_support", "float", 0.6, 1.0,
          "Rule policy: place the current box only if its best spot has at least this support ratio; "
          "otherwise prefer a buffered box or buffer the current one."),
    Param("rules.retrieve_margin_m", "float", 0.0, 0.15,
          "Rule policy: a buffered box is placed instead of the current one when its spot ends this much lower (m)."),
    Param("rules.max_buffer_age", "int", 2, 40,
          "Rule policy: a buffered box older than this many decisions is retrieved first."),
    Param("close.fill_before_buffer", "float", 0.0, 0.7,
          "Close the pallet instead of buffering when nothing fits and the pallet is at least this full (0 = off)."),
    Param("repack.enabled", "bool", 0, 1, "Allow PARTIAL_REPACK (moving placed boxes) when nothing fits."),
    Param("repack.max_moves", "int", 1, 3, "Boxes moved per repack."),
    Param("generation.dedup_distance_m", "float", 0.02, 0.15,
          "5-1: candidates closer than this are merged (smaller = more candidates, slower)."),
    Param("generation.balance_anchors", "bool", 0, 1,
          "5-1: extra candidates where a heavy box's load is shared by several supporters."),
)
BY_NAME = {p.name: p for p in PARAMS}


def default_params(cand_config, hl_config):
    """Current values of every tunable parameter."""
    out = {}
    for p in PARAMS:
        section, key = p.name.split(".")
        src = cand_config.generation if section == "generation" else getattr(hl_config, section)
        out[p.name] = getattr(src, key)
    return out


def validate_params(params):
    """Clean a proposed parameter dict. Returns (clean dict, list of problems)."""
    clean, problems = {}, []
    for name, value in (params or {}).items():
        p = BY_NAME.get(name)
        if p is None:
            problems.append(f"unknown parameter {name!r}")
            continue
        try:
            if p.kind == "bool":
                v = bool(value) if not isinstance(value, str) else value.lower() in ("1", "true", "yes")
            elif p.kind == "int":
                v = int(round(float(value)))
            else:
                v = float(value)
        except (TypeError, ValueError):
            problems.append(f"{name}: not a {p.kind}")
            continue
        if p.kind != "bool" and not p.low <= v <= p.high:
            problems.append(f"{name}={v} outside [{p.low}, {p.high}]")
            continue
        clean[name] = v
    return clean, problems


def apply_params(cand_config, hl_config, params):
    """New (cand_config, hl_config) with ``params`` applied (others unchanged)."""
    gen = {}
    hl = {}
    for name, value in params.items():
        section, key = name.split(".")
        if section == "generation":
            gen[key] = value
        else:
            hl.setdefault(section, {})[key] = value
    if gen:
        cand_config = replace(cand_config, generation=replace(cand_config.generation, **gen))
    for section, kv in hl.items():
        hl_config = replace(hl_config, **{section: replace(getattr(hl_config, section), **kv)})
    return cand_config, hl_config


def describe():
    return [{"name": p.name, "type": p.kind, "range": None if p.kind == "bool" else [p.low, p.high],
             "meaning": p.meaning} for p in PARAMS]
