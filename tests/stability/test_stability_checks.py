"""Analytic stack checks (pac_candidates.stability) and the hard-mask hook."""

import importlib.util
from pathlib import Path
import random
import sys

from pac_candidates.stability import StackBox, boxes_from_layout, lateral_check, pbs_check, sme_check

ROOT = Path(__file__).resolve().parents[2]


def box(i, x, y, z, dx, dy, dz, m=10.0, com=(0.0, 0.0, 0.0)):
    return StackBox(f"b{i}", x, y, z, dx, dy, dz, m, com)


def test_single_box_on_pallet_passes_everything():
    b = [box(0, 0.1, 0.1, 0.0, 0.3, 0.3, 0.3)]
    assert lateral_check(b).ok and sme_check(b).ok and pbs_check(b).ok


def test_tall_narrow_column_fails_only_the_lateral_check():
    # h = 0.4 m above the base, 0.3 g shifts the CoG by 0.12 m > half width 0.1 m
    b = [box(0, 0.0, 0.0, 0.0, 0.2, 0.2, 0.8)]
    assert sme_check(b).ok and pbs_check(b).ok
    res = lateral_check(b, 0.3)
    assert not res.ok and res.worst_box == "b0"
    assert lateral_check(b, 0.2).ok          # 0.08 m < 0.1 m


def test_lateral_threshold_matches_half_width_over_height():
    b = [box(0, 0.0, 0.0, 0.0, 0.3, 0.3, 1.0)]       # limit = 0.15 / 0.5 = 0.3 g
    assert lateral_check(b, 0.29).ok and not lateral_check(b, 0.31).ok


def test_overhang_fails_sme_and_pbs():
    b = [box(0, 0.0, 0.0, 0.0, 0.4, 0.4, 0.2), box(1, 0.25, 0.0, 0.2, 0.4, 0.4, 0.2)]
    assert not sme_check(b).ok and not pbs_check(b, 0.70).ok


def test_displaced_centre_of_mass_is_an_overestimation_case_for_pbs():
    """Mazur et al. 2025 section 5.2: PBS ignores a displaced CoM (OE), SME does not."""
    base = box(0, 0.0, 0.0, 0.0, 0.4, 0.4, 0.2)
    top = box(1, 0.1, 0.0, 0.2, 0.4, 0.4, 0.2)                     # 75 % supported
    assert sme_check([base, top]).ok and pbs_check([base, top]).ok
    heavy_end = box(1, 0.1, 0.0, 0.2, 0.4, 0.4, 0.2, com=(0.15, 0.0, 0.0))
    assert pbs_check([base, heavy_end]).ok and not sme_check([base, heavy_end]).ok


def test_load_from_above_counts_on_every_level():
    # the top box alone is fine on the middle one, but it shifts the middle box's load over its edge
    b = [box(0, 0.0, 0.0, 0.0, 0.4, 0.4, 0.2), box(1, 0.15, 0.0, 0.2, 0.4, 0.4, 0.2, m=1.0),
         box(2, 0.35, 0.0, 0.4, 0.4, 0.4, 0.2, m=30.0)]
    assert not sme_check(b).ok


def test_boxes_from_layout_swaps_extents_and_rotates_com_on_quarter_turn():
    layout = [{"box_id": "a", "size": [0.4, 0.2, 0.3], "pose": [0.0, 0.0, 0.0, 1.5707963], "mass_kg": 5.0,
               "com_offset": [0.1, 0.0, 0.0]}]
    (b,) = boxes_from_layout(layout)
    assert (round(b.dx, 6), round(b.dy, 6)) == (0.2, 0.4)
    assert abs(b.com_offset[0]) < 1e-6 and abs(b.com_offset[1] - 0.1) < 1e-6


def _prototype():
    path = ROOT / "tools/prototypes/lookahead/lookahead.py"
    spec = importlib.util.spec_from_file_location("proto_lookahead", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod             # dataclasses resolve annotations through sys.modules
    spec.loader.exec_module(mod)
    return mod


def test_matches_the_prototype_stack_lateral_ok_on_random_stacks():
    proto = _prototype()
    rng = random.Random(7)
    checked = 0
    for _ in range(300):
        boxes = []
        for i in range(rng.randint(1, 6)):
            dx, dy, dz = rng.choice([0.2, 0.3, 0.4]), rng.choice([0.2, 0.3, 0.4]), rng.choice([0.2, 0.3, 0.5])
            x, y = rng.choice([0.0, 0.1, 0.2, 0.3]), rng.choice([0.0, 0.1, 0.2])
            z = max([b.top for b in boxes
                     if min(x + dx, b.x + b.dx) - max(x, b.x) > 1e-3 and min(y + dy, b.y + b.dy) - max(y, b.y) > 1e-3]
                    + [0.0])
            boxes.append(box(i, x, y, z, dx, dy, dz, m=rng.uniform(1, 30)))
        tup = [(b.x, b.y, b.z, b.dx, b.dy, b.dz, b.mass_kg) for b in boxes]
        for a_g in (0.2, 0.3):
            expected = proto.stack_lateral_ok(tup[:-1], tup[-1], a_g)
            assert lateral_check(boxes, a_g, only=len(boxes) - 1).ok == expected
            checked += 1
    assert checked == 600


def test_hard_mask_lateral_stability_is_opt_in():
    """A 0.2 x 0.2 x 0.8 m box passes the hard mask, and is rejected with
    LATERAL_STABILITY once the 0.3 g check is enabled (candidates.yaml: off)."""
    from dataclasses import replace

    sys.path.insert(0, str(ROOT / "tests/taehyeon"))
    from th_helpers import candidate, make_box, make_context, make_state
    from pac_candidates import CandidateBackend, CandidateConfig
    from pac_candidates.config import load_candidate_config
    from pac_common import RejectCode as R

    assert not load_candidate_config(ROOT / "config/taehyeon/candidates.yaml").constraints.lateral_stability.enabled
    tall = make_box("TALL", size=(0.2, 0.2, 0.8), yaws=(0.0,))
    state = make_state()

    def verdict(cfg):
        return CandidateBackend(make_context(), cfg).validate_constraints(
            tall, candidate(tall, 0.45, 0.45, 0.0, version=state.state_version), state)

    base = CandidateConfig()
    assert verdict(base).success
    on = replace(base, constraints=replace(base.constraints, lateral_stability=replace(
        base.constraints.lateral_stability, enabled=True)))
    res = verdict(on)
    assert not res.success and R.COG_VIOLATION in res.codes
    assert any(r.startswith("LATERAL_STABILITY") for r in res.details.get("reasons", ()))
