"""``config/donghan/reinspection_policy.yaml`` -> ``ReinspectionPolicy``.

Every number in the file is a test assumption (``assumed: true``), not a
measured value.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from .codes import REASONS

FAMILIES = ("label", "weigh", "frames")
KINDS = ("relabel", "reweigh", "rezero", "recapture", "frames", "separate", "second_view")


@dataclass(frozen=True)
class ReinspectionPolicy:
    reasons: dict
    retries: dict           # family -> max re-sensing attempts per box
    costs_s: dict           # reacquire kind -> assumed time
    total_time_s: float     # per-box reinspection budget, separate from operator_time_s
    max_verdict_changes: int
    assumed: bool = True

    def enabled(self, code: str) -> bool:
        return bool(self.reasons.get(code, {}).get("enabled", False))

    def param(self, code: str, name: str):
        return self.reasons[code][name]


def policy_from_dict(data: dict) -> ReinspectionPolicy:
    if data.get("schema_version") != 1:
        raise ValueError(f"unsupported schema_version: {data.get('schema_version')}")
    reasons = {k: dict(v or {}) for k, v in (data.get("reasons") or {}).items()}
    unknown = set(reasons) - set(REASONS)
    if unknown:
        raise ValueError(f"unknown RI codes: {sorted(unknown)}")
    for code, cfg in reasons.items():
        if cfg.get("enabled") and not REASONS[code].implemented_here:
            raise ValueError(f"{code} is not implemented here ({REASONS[code].feasibility})")
    retries = dict(data["retries"])
    costs = dict(data["costs_s"])
    if set(retries) != set(FAMILIES) or set(costs) != set(KINDS):
        raise ValueError(f"retries needs {FAMILIES}, costs_s needs {KINDS}")
    budget = data["budget"]
    numbers = list(retries.values()) + list(costs.values()) + [budget["total_time_s"], budget["max_verdict_changes"]]
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0 for v in numbers):
        raise ValueError("retries, costs and budget must be finite and >= 0")
    return ReinspectionPolicy(reasons, retries, costs, float(budget["total_time_s"]),
                              int(budget["max_verdict_changes"]), bool(data.get("assumed", True)))


def load_policy(path: str | Path) -> ReinspectionPolicy:
    return policy_from_dict(yaml.safe_load(Path(path).read_text(encoding="utf-8")))
