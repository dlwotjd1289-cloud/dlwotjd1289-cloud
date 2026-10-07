"""Stage 5-1/5-2 (taehyeon) wired into stage 5-3~5-6 (donghan) planner.

Skipped automatically when the planner package is not available
(run scripts/taehyeon/fetch_team_deps.sh before merging).
"""

import pytest

from th_helpers import PLANNING
from pac_common import ConstraintEvidence, StateMode, plain
from pac_candidates import CandidateBackend

pytestmark = pytest.mark.skipif(PLANNING is None, reason="pac_planning not available")

if PLANNING is not None:
    from pac_planning import PlacementPlanner, PlannerConfig
    from pac_planning.demo import scene_from_file

    FIXTURES = PLANNING.parents[2] / "test_data"


@pytest.fixture(params=["scenario_001_basic.json", "scenario_002_partial_pallet.json"])
def scene(request):
    return scene_from_file(FIXTURES / request.param)


def make_planner(context, backend, **cfg):
    return PlacementPlanner(
        context=context,
        config=PlannerConfig(**cfg),
        generate_candidates=backend.generate_candidates,
        validate_constraints=backend.validate_constraints,
    )


def test_end_to_end_plan_with_taehyeon_backend(scene):
    _, box, state, context = scene
    backend = CandidateBackend(context)
    planner = make_planner(context, backend, horizon=2)
    before = plain(state)
    result = planner.plan(box, state, seed=7, use_time_budget=False)
    assert plain(state) == before
    assert result.ranked, result.diagnostics
    assert result.state_mode == StateMode.PLANNED
    assert result.requires_robot_validation
    assert result.diagnostics["completed_scenarios"] == 7
    for cand in result.ranked:
        verdict = backend.validate_constraints(box, cand, state)
        assert verdict.success
        assert isinstance(verdict.details["evidence"], ConstraintEvidence)
    again = make_planner(context, CandidateBackend(context), horizon=2).plan(
        box, state, seed=7, use_time_budget=False
    )
    assert again.ranked == result.ranked


def test_ems_supplied_features(scene):
    _, box, state, context = scene
    backend = CandidateBackend(context)
    report = backend.generate_with_report(box, state)
    ctx = backend.context_with_ems(context, report)
    planner = make_planner(ctx, backend, horizon=1, scenario_count=2)
    result = planner.plan(box, state, list(report.candidates), seed=1, use_time_budget=False)
    sources = {e.features.geometry_source for e in result.evaluations}
    assert "EMS_SUPPLIED" in sources


def test_rejections_use_team_codes(scene):
    _, box, state, context = scene
    backend = CandidateBackend(context)
    planner = make_planner(context, backend, horizon=1, scenario_count=1)
    result = planner.plan(box, state, seed=3, mode="current", use_time_budget=False)
    for verdict in result.rejected.values():
        assert not verdict.success and verdict.codes


def test_soft_budget_run_completes(scene):
    _, box, state, context = scene
    backend = CandidateBackend(context)
    planner = make_planner(context, backend)
    result = planner.plan(box, state, seed=11)
    assert result.ranked
    assert result.diagnostics["planning_time_sec"] < 5.0
