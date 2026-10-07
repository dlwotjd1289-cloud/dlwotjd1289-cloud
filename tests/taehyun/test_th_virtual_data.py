import json
from pathlib import Path

import pytest

from th_helpers import REPO
from pac_common.adapters import context_from_json, state_from_json
from pac_candidates import CandidateConfig, load_candidate_config
from virtual_data import load_virtual_config, virtual_config_from_dict
from virtual_data.pipeline import generate
from virtual_data.scenario_source import build_catalog, load_dataset, stack_height_limit
from virtual_data.validation import validate_output

FIXTURE = Path(__file__).parent / "fixtures" / "ahead_sample"


@pytest.fixture(scope="module")
def run_dir(tmp_path_factory):
    out = tmp_path_factory.mktemp("vd")
    vcfg = load_virtual_config(REPO / "config/taehyun/virtual_data.yaml")
    cfg = load_candidate_config(REPO / "config/taehyun/candidates.yaml")
    generate(FIXTURE, out, cfg, vcfg, repo_root=REPO)
    return out


def test_dataset_loading_and_catalog():
    ds = load_dataset(FIXTURE)
    assert [s.scenario_id for s in ds.scenarios] == ["S0001", "S0003", "S0005"]
    spec = ds.scenarios[0]
    assert len(spec.arrivals) == 24
    assert [b.box_id for b in spec.arrivals] == sorted(b.box_id for b in spec.arrivals)
    vcfg = virtual_config_from_dict({})
    assert stack_height_limit(spec, vcfg) == pytest.approx(1.35)
    catalog = build_catalog(ds.sku_ranges, vcfg, CandidateConfig().constraints.load_model)
    assert set(catalog) == set(ds.sku_ranges)
    for spec_ in catalog.values():
        assert spec_.top_load_capacity_n > 0


def test_outputs_are_complete_and_valid(run_dir):
    manifest = json.loads((run_dir / "manifest.json").read_text())
    assert manifest["scenario_count"] == 3
    assert manifest["scene_count"] == 72
    assert manifest["contract"]["future_order_in_scene"] is False
    assert validate_output(run_dir, recheck_rows=10) == []


def test_scene_is_donghan_compatible(run_dir):
    path = sorted((run_dir / "scenes").rglob("S0001-T010.json"))[0]
    data = json.loads(path.read_text())
    assert data["schema_version"] == "pac-common-v0.2+planning-v1"
    state = state_from_json(data["state"])
    context_from_json(data["context"])
    assert data["current_box_id"] in state.inventory.tracked_boxes
    assert state.pallet.size.z == pytest.approx(1.35)
    # step 10: 11 boxes arrived (incl. the current one) -> 13 unseen remain;
    # the current box and placed boxes are never counted as unseen.
    assert sum(state.inventory.remaining_by_sku.values()) == 24 - 11


def test_labels_have_reasons_and_evidence(run_dir):
    rows = [json.loads(l) for l in (run_dir / "candidate_sets" / "S0001.jsonl").open()]
    assert len(rows) == 24
    for row in rows:
        assert row["generated_count"] == len(row["candidates"])
        for c in row["candidates"]:
            if c["valid"]:
                assert 0 <= c["evidence"]["support_ratio"] <= 1
            else:
                assert c["reasons"]


def test_split_folders_follow_generator_split(run_dir):
    assert sorted(p.name for p in (run_dir / "scenes").iterdir() if p.is_dir()) == [
        "test",
        "train",
        "val",
    ]


def test_generation_is_reproducible(run_dir, tmp_path):
    vcfg = load_virtual_config(REPO / "config/taehyun/virtual_data.yaml")
    cfg = load_candidate_config(REPO / "config/taehyun/candidates.yaml")
    generate(FIXTURE, tmp_path, cfg, vcfg, scenario_ids={"S0003"}, repo_root=REPO)
    a = (run_dir / "candidate_sets" / "S0003.jsonl").read_text().splitlines()
    b = (tmp_path / "candidate_sets" / "S0003.jsonl").read_text().splitlines()
    strip = lambda rows: [
        {k: v for k, v in json.loads(r).items() if not k.endswith("_sec")} for r in rows
    ]
    assert strip(a) == strip(b)


def test_true_size_errors_absorbed_by_delta(run_dir):
    for path in (run_dir / "episodes").glob("*.json"):
        metrics = json.loads(path.read_text())["metrics"]
        assert metrics["true_overlaps"] == []
        assert metrics["true_protrusions"] == []
        assert metrics["snapshot_issues"] == {}
