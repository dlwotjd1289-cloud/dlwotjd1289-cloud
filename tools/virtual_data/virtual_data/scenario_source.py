"""Read (or produce) jaesung's AHEAD dataset-generator output.

The generator is executed as a subprocess (never imported) so its module
layout cannot shadow the team ``pac_common``. Only its files are consumed:

* ``test_data/Sxxxx.json``      planner-safe initial state (no future order)
* ``ground_truth/Sxxxx.json``   true arrival order (simulator only)
* ``analysis/catalog.csv``      SKU sizes / weight ranges / yaws
* ``splits.json``               scenario-level train/val/test split
"""

import csv
from dataclasses import dataclass
import json
import os
from pathlib import Path
import subprocess
import sys

from pac_common import Size3D, SkuSpec
from pac_common.adapters import box_from_json

from pac_candidates.loads import mckee_capacity_n


@dataclass(frozen=True)
class SkuRange:
    sku_id: str
    size: Size3D
    weight_min_kg: float
    weight_max_kg: float
    allowed_yaws_rad: tuple


@dataclass(frozen=True)
class ScenarioSpec:
    scenario_id: str
    family: str
    seed: int
    pallet_id: str
    pallet_xy: tuple  # (x, y) m
    pallet_deck_m: float
    max_height_m: float
    max_load_kg: float | None
    arrivals: tuple  # BoxState in true arrival order


@dataclass(frozen=True)
class Dataset:
    root: Path
    scenarios: tuple
    sku_ranges: dict
    splits: dict
    manifest: dict

    def split_of(self, scenario_id):
        for name, ids in self.splits.items():
            if scenario_id in ids:
                return name
        return "unsplit"


def run_generator(
    generator_root,
    pac_common_src,
    output_dir,
    mode="sample",
    sample_per_family=2,
    seed=None,
    config_path=None,
):
    """Execute jaesung's generator CLI; returns its stdout line."""
    generator_root = Path(generator_root)
    cmd = [
        sys.executable,
        str(generator_root / "scripts" / "generate_dataset.py"),
        "--output",
        str(Path(output_dir).resolve()),
        "--mode",
        mode,
        "--sample-per-family",
        str(sample_per_family),
    ]
    if seed is not None:
        cmd += ["--seed", str(seed)]
    if config_path is not None:
        cmd += ["--config", str(Path(config_path).resolve())]
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        p for p in (str(pac_common_src), env.get("PYTHONPATH", "")) if p
    )
    done = subprocess.run(
        cmd, cwd=generator_root, env=env, capture_output=True, text=True, check=False
    )
    if done.returncode != 0:
        raise RuntimeError(f"generator failed:\n{done.stdout}\n{done.stderr}")
    return done.stdout.strip()


def load_sku_ranges(dataset_dir):
    rows = {}
    with (Path(dataset_dir) / "analysis" / "catalog.csv").open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            rows[row["sku_id"]] = SkuRange(
                row["sku_id"],
                Size3D(float(row["size_x_m"]), float(row["size_y_m"]), float(row["size_z_m"])),
                float(row["weight_min_kg"]),
                float(row["weight_max_kg"]),
                tuple(float(v) for v in json.loads(row["allowed_yaws_rad"])),
            )
    return rows


def load_dataset(dataset_dir):
    root = Path(dataset_dir)
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    splits = json.loads((root / "splits.json").read_text(encoding="utf-8"))
    scenarios = []
    for path in sorted((root / "test_data").glob("S*.json")):
        planner_safe = json.loads(path.read_text(encoding="utf-8"))
        truth = json.loads(
            (root / "ground_truth" / path.name).read_text(encoding="utf-8")
        )
        events = sorted(truth["arrival_events"], key=lambda e: e["arrival_index"])
        pallet = planner_safe["initial_system_state"]["pallet"]
        constraints = planner_safe.get("constraints", {})
        scenarios.append(
            ScenarioSpec(
                scenario_id=planner_safe["scenario_id"],
                family=planner_safe["scenario_family"],
                seed=int(planner_safe["seed"]),
                pallet_id=pallet["pallet_id"],
                pallet_xy=(float(pallet["size"]["x"]), float(pallet["size"]["y"])),
                pallet_deck_m=float(pallet["size"]["z"]),
                max_height_m=float(constraints.get("max_height_m", 1.5)),
                max_load_kg=constraints.get("max_load_kg"),
                arrivals=tuple(box_from_json(e["box_state"]) for e in events),
            )
        )
    return Dataset(root, tuple(scenarios), load_sku_ranges(root), splits, manifest)


def stack_height_limit(spec, config):
    if config.pallet.height_limit_includes_pallet:
        return spec.max_height_m - spec.pallet_deck_m
    return spec.max_height_m


def build_catalog(sku_ranges, config, load_model):
    """``SkuSpec`` per SKU; top-load capacity from the McKee model."""
    catalog = {}
    for sku_id, r in sorted(sku_ranges.items()):
        if config.catalog.nominal_weight == "max":
            weight = r.weight_max_kg
        else:
            weight = 0.5 * (r.weight_min_kg + r.weight_max_kg)
        capacity = mckee_capacity_n(
            r.size.x,
            r.size.y,
            load_model.ect_n_per_m,
            load_model.board_thickness_m,
            load_model.safety_factor,
        )
        catalog[sku_id] = SkuSpec(
            sku_id, r.size, round(weight, 6), r.allowed_yaws_rad, round(capacity, 3)
        )
    return catalog
