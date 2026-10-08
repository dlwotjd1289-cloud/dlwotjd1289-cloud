from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from pac_common import Size3D


SCENARIO_FAMILIES = (
    "normal",
    "size_mixed",
    "weight_mixed",
    "late_large",
    "late_heavy",
    "repeated_sku",
)


@dataclass(frozen=True)
class SkuSpec:
    sku_id: str
    size: Size3D
    weight_min_kg: float
    weight_max_kg: float
    allowed_yaws_rad: tuple[float, ...]
    sampling_weight: float
    source_group: str

    @property
    def volume_m3(self) -> float:
        return self.size.x * self.size.y * self.size.z

    @property
    def dimension_sum_m(self) -> float:
        return self.size.x + self.size.y + self.size.z


def load_yaml(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("YAML root must be a mapping")
    return payload


def load_sku_catalog(cfg: dict[str, Any]) -> list[SkuSpec]:
    specs = []
    for item in cfg["sku_catalog"]:
        specs.append(
            SkuSpec(
                sku_id=str(item["sku_id"]),
                size=Size3D(
                    x=float(item["size_m"]["x"]),
                    y=float(item["size_m"]["y"]),
                    z=float(item["size_m"]["z"]),
                ),
                weight_min_kg=float(item["weight_min_kg"]),
                weight_max_kg=float(item["weight_max_kg"]),
                allowed_yaws_rad=tuple(
                    float(value) for value in item["allowed_yaws_rad"]
                ),
                sampling_weight=float(item.get("sampling_weight", 1.0)),
                source_group=str(item.get("source_group", "representative")),
            )
        )
    return specs


def validate_config(cfg: dict[str, Any]) -> None:
    required = (
        "schema_version",
        "dataset",
        "parcel_domain",
        "pallet",
        "sku_catalog",
        "generation",
        "split",
    )
    for key in required:
        if key not in cfg:
            raise ValueError(f"config missing required key: {key}")

    if int(cfg["schema_version"]) != 1:
        raise ValueError("supported schema_version is 1")

    domain = cfg["parcel_domain"]
    max_weight = float(domain["max_weight_kg"])
    max_sum = float(domain["max_dimension_sum_m"])
    max_side = float(domain["max_single_side_m"])

    if min(max_weight, max_sum, max_side) <= 0.0:
        raise ValueError("parcel-domain limits must be positive")

    pallet = cfg["pallet"]
    for key in ("x_m", "y_m", "physical_height_m", "max_height_m"):
        if float(pallet[key]) <= 0.0:
            raise ValueError(f"pallet.{key} must be positive")

    catalog = load_sku_catalog(cfg)
    if not catalog:
        raise ValueError("sku_catalog must not be empty")

    seen = set()
    for sku in catalog:
        if sku.sku_id in seen:
            raise ValueError(f"duplicate SKU: {sku.sku_id}")
        seen.add(sku.sku_id)
        if min(sku.size.x, sku.size.y, sku.size.z) <= 0.0:
            raise ValueError(f"invalid size for {sku.sku_id}")
        if sku.weight_min_kg <= 0.0:
            raise ValueError(f"weight_min_kg must be positive for {sku.sku_id}")
        if sku.weight_max_kg < sku.weight_min_kg:
            raise ValueError(f"invalid weight range for {sku.sku_id}")
        if sku.weight_max_kg > max_weight + 1e-12:
            raise ValueError(f"{sku.sku_id} exceeds parcel max weight")
        if sku.dimension_sum_m > max_sum + 1e-12:
            raise ValueError(f"{sku.sku_id} exceeds parcel dimension sum")
        if max(sku.size.x, sku.size.y, sku.size.z) > max_side + 1e-12:
            raise ValueError(f"{sku.sku_id} exceeds parcel max single side")
        if sku.sampling_weight <= 0.0:
            raise ValueError(f"sampling_weight must be positive for {sku.sku_id}")
        if not sku.allowed_yaws_rad:
            raise ValueError(f"allowed_yaws_rad must not be empty for {sku.sku_id}")

    observation_mode = cfg["generation"]["observation"]["mode"]
    if observation_mode != "identity":
        raise ValueError(
            "v1.2 supports identity observation only; camera error injection is "
            "reserved for the observation generator"
        )

    family_counts = cfg["generation"]["benchmark"]["family_counts"]
    for family, count in family_counts.items():
        if family not in SCENARIO_FAMILIES:
            raise ValueError(f"unknown scenario family: {family}")
        if int(count) < 0:
            raise ValueError("benchmark family count must be non-negative")

    split = cfg["split"]
    ratios = [
        float(split["train_ratio"]),
        float(split["val_ratio"]),
        float(split["test_ratio"]),
    ]
    if min(ratios) < 0.0 or abs(sum(ratios) - 1.0) > 1e-12:
        raise ValueError("train/val/test ratios must be non-negative and sum to 1")
