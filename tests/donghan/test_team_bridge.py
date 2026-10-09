"""Run with the teammate's pac_candidates on PYTHONPATH (see integration guide)."""
from dataclasses import replace
import json
import math
from pathlib import Path

import pytest

pytest.importorskip("pac_candidates")
from pac_candidates import CandidateBackend
from pac_common import BoxStatus, InventoryState, Pose3D, Size3D, plain
from pac_planning import PlannerConfig
from pac_planning.team_bridge import TeamPlacer, TeamRuntimeRanker, plan_with_backend
from pac_planning.planning_service import plan_request
from pac_planning.scene_bridge import PalletFrame, bullet_payload
from pac_planning.team_bridge import check_policy_contract

FAST = PlannerConfig(horizon=1, scenario_count=1)


@pytest.mark.parametrize("yaw", [0.0, math.pi, -math.pi/2, 3*math.pi/2])
def test_real_ems_and_no_actual_state_commit(scene, yaw):
    _, box, state, context = scene
    box = replace(box, allowed_yaws_rad=(yaw,))
    state = replace(state, inventory=replace(state.inventory,
                    tracked_boxes={**state.inventory.tracked_boxes, box.box_id: box}))
    before = plain(state)
    result = plan_with_backend(box, state, CandidateBackend(context), config=FAST,
                               use_time_budget=False)
    assert result.ranked
    assert all(c.target_pose.yaw == yaw for c in result.ranked)
    assert all(e.features.geometry_source == "EMS_SUPPLIED" for e in result.evaluations)
    assert result.requires_robot_validation
    assert result.diagnostics["model_status"] == "NO_MODEL_HEURISTIC"
    assert result.diagnostics["completed_scenarios"] == 1
    assert plain(state) == before


def test_buffered_snapshot_and_four_slot_context(scene):
    _, arrival, state, context = scene
    boxes = [replace(arrival, box_id="buffer_" + str(i), status=BoxStatus.BUFFERED)
             for i in range(4)]
    state = replace(state, inventory=InventoryState({b.box_id: b for b in boxes}, {}))
    context = replace(context, observed_preview=(), buffer_capacity=4)
    backend = CandidateBackend(context)
    placer = TeamPlacer(FAST, use_time_budget=False)
    # The high-level arrival object still says READY_FOR_PICK.
    old = replace(boxes[0], status=BoxStatus.READY_FOR_PICK)
    assert placer(backend.generate_candidates(boxes[0], state), old, state, backend)
    assert placer.last_result.requires_robot_validation
    assert state.inventory.tracked_boxes[old.box_id].status == BoxStatus.BUFFERED


def test_runtime_ranker_preserves_real_ems_and_does_not_commit(scene):
    _, box, state, context = scene
    backend = CandidateBackend(context)
    valid = [c for c in backend.generate_candidates(box, state)
             if backend.validate_constraints(box, c, state).success]
    before = plain(state)
    ranker = TeamRuntimeRanker(FAST, use_time_budget=False)
    ordered = ranker(valid, box, state, backend)
    assert ordered == list(ranker.last_result.ranked)
    assert ordered
    assert all(e.features.geometry_source == "EMS_SUPPLIED"
               for e in ranker.last_result.evaluations)
    assert ranker.last_result.requires_robot_validation
    provenance = ranker.provenance()
    assert provenance["calls"] == 1
    assert provenance["candidate_evaluations"] == len(ranker.last_result.evaluations)
    assert provenance["geometry_sources"] == {
        "EMS_SUPPLIED": len(ranker.last_result.evaluations)
    }
    assert provenance["ems_verified"]
    assert provenance["model_statuses"] == {"NO_MODEL_HEURISTIC": 1}
    assert provenance["robot_validation_required_calls"] == 1
    assert len(provenance["backend_contract_sha256"]) == 64
    assert plain(state) == before


@pytest.mark.parametrize("change", ["version", "pose", "id"])
def test_reject_mismatched_candidate_from_another_snapshot(scene, change):
    _, box, state, context = scene
    backend = CandidateBackend(context)
    c = backend.generate_candidates(box, state)[0]
    if change == "version": c = replace(c, base_state_version=state.state_version+1)
    if change == "pose": c = replace(c, target_pose=replace(c.target_pose, x=c.target_pose.x+0.01))
    if change == "id": c = replace(c, candidate_id="wrong")
    with pytest.raises(ValueError, match="snapshot"):
        plan_with_backend(box, state, backend, candidates=[c], config=FAST)


def test_service_request_matches_direct_planner(scene):
    _, box, state, context = scene
    backend = CandidateBackend(context)
    result = json.loads(plan_request(json.dumps(plain(state)), json.dumps(plain(context)),
                                    box.box_id, state.state_version, backend.config, FAST,
                                    use_time_budget=False))
    direct = plan_with_backend(box, state, backend, config=FAST, use_time_budget=False)
    assert result["ranked"] == plain(direct.ranked)
    assert result["requires_robot_validation"]
    with pytest.raises(ValueError, match="STALE_PLAN"):
        plan_request(json.dumps(plain(state)), json.dumps(plain(context)), box.box_id,
                     state.state_version+1, backend.config, FAST)


@pytest.mark.parametrize("yaw", [0.0, math.pi/2])
def test_corner_to_bullet_and_world_coordinates(scene, yaw):
    _, box, state, context = scene
    c = CandidateBackend(context).generate_candidates(box, state)[0]
    c = replace(c, target_pose=Pose3D("pallet", 0.05, 0.06, 0.0, yaw=yaw))
    p = state.pallet.size
    dx, dy = (box.size.y, box.size.x) if yaw else (box.size.x, box.size.y)
    expected = (0.05 + dx/2 - p.x/2, 0.06 + dy/2 - p.y/2, box.size.z/2)
    payload = bullet_payload(box, c, state, context, simulator_size_xy=(p.x, p.y))
    assert payload["target_position_m"] == pytest.approx(expected)
    frame = PalletFrame(Size3D(p.x, p.y, .15), Pose3D("world", 1.35, -1., .15))
    world = frame.centre_in_world(box, c, state)
    assert (world.x, world.y, world.z) == pytest.approx((1.35+expected[0], -1+expected[1], .15+expected[2]))


def test_footprint_mismatch_and_zero_capacity_preserved(scene):
    _, box, state, context = scene
    c = CandidateBackend(context).generate_candidates(box, state)[0]
    context = replace(context, capacity_overrides_n={box.box_id: 0.0})
    p = state.pallet.size
    assert bullet_payload(box, c, state, context, simulator_size_xy=(p.x,p.y))["max_top_load_n"] == 0
    with pytest.raises(ValueError, match="footprints differ"):
        bullet_payload(box, c, state, context, simulator_size_xy=(p.x+.1,p.y))


def test_workcell_wood_center_is_not_deck_top(tmp_path):
    path = tmp_path / "workcell.yaml"
    path.write_text("layout:\n  pallet:\n    size_m: [1.2, 1.0, 0.15]\n    center_world_m: [1.35, -1.0, 0.075]\n")
    frame = PalletFrame.from_workcell(path)
    assert frame.top_center.z == .15


def test_ppo_dblf_contract_cannot_be_relabelled_as_new_placer():
    with pytest.raises(ValueError, match="PPO placer"):
        check_policy_contract({"placer":"dblf", "value_provider":"proxy"},
                              placer_name=TeamPlacer.name, value_provider="proxy")
    with pytest.raises(ValueError):
        check_policy_contract({"placer":TeamPlacer.name, "value_provider":"proxy"},
                              placer_name=TeamPlacer.name, value_provider="donghan")
    check_policy_contract({"placer":TeamPlacer.name, "value_provider":"proxy"},
                          placer_name=TeamPlacer.name, value_provider="proxy")


@pytest.mark.parametrize('uncertain', [False, True])
@pytest.mark.parametrize('dedup_mode', ['off', 'support_aware', 'mask_aware'])
def test_pose_only_generator_matches_real_ems_report_for_post_placement(scene, uncertain, dedup_mode):
    from pac_planning.features import sku_box
    from pac_planning.geometry import simulate_placement
    _, box, state, context = scene
    if uncertain:
        context = replace(context, uncertain_box_ids=(box.box_id,))
    base = CandidateBackend(context)
    cfg = replace(base.config, generation=replace(base.config.generation, dedup_mode=dedup_mode))
    backend = CandidateBackend(context, cfg)
    assert backend.generate_candidates(box, state) == list(backend.generate_with_report(box, state).candidates)
    for root in backend.candidate_set(box, state).valid[:3]:
        post = simulate_placement(state, box, root).state
        for sku, spec in context.catalog.items():
            probe = sku_box(spec, '__probe__' + sku, state.stamp_sec)
            fast = backend.generate_candidates(probe, post)
            full = list(backend.generate_with_report(probe, post).candidates)
            assert fast == full
            assert [backend.validate_constraints(probe, c, post) for c in fast] == [
                backend.validate_constraints(probe, c, post) for c in full]


MODEL = Path(__file__).parents[2] / "models/dual_head_ranker.json"


def test_trained_model_rejects_a_planner_config_it_was_not_trained_for(scene):
    """A horizon/scenario mismatch fails at load time instead of silently
    falling back to the heuristic on every call (INFERENCE_FAILED)."""
    from pac_highlevel.value import DonghanPlacer
    from pac_planning.team_bridge import TeamPlacer, load_checked_model

    with pytest.raises(ValueError, match="rollout contract"):
        TeamRuntimeRanker(FAST, model_path=MODEL)
    with pytest.raises(ValueError, match="rollout contract"):
        DonghanPlacer(model_path=MODEL, planner_config=FAST)
    assert load_checked_model(MODEL, PlannerConfig()) is not None
    _, box, state, context = scene
    with pytest.raises(ValueError, match="rollout contract"):
        plan_with_backend(box, state, CandidateBackend(context), config=FAST, model_path=MODEL)
    # the high-level evaluation placer is the same adapter as the runtime one
    assert isinstance(DonghanPlacer(planner_config=FAST), TeamPlacer)


def test_v44_bridge_uses_this_repository_planner():
    import subprocess
    import sys

    root = Path(__file__).parents[2]
    probe = ("import runpy, sys; sys.argv=['x','--help']\n"
             "try: runpy.run_path(r'%s', run_name='__main__')\n"
             "except SystemExit: pass\n"
             "import pac_planning; print(pac_planning.__file__)" % (root / "scripts/ahead_planner_bridge_v44.py"))
    out = subprocess.run([sys.executable, "-I", "-c", probe], capture_output=True, text=True, check=True).stdout
    assert Path(out.strip().splitlines()[-1]).resolve().is_relative_to((root / "ros2_ws/src/pac_planning").resolve())


class _RejectingRobot:
    """Stage-6 stand-in that rejects a fixed set of candidate ids."""

    def __init__(self, rejected):
        self.rejected = set(rejected)
        self.checked = []

    def validate_robot_motion(self, box, candidate, state, robot_state=None):
        from pac_common import RejectCode, ValidationResult

        self.checked.append(candidate.candidate_id)
        ok = candidate.candidate_id not in self.rejected
        return ValidationResult(ok, () if ok else (RejectCode.IK_FAIL,), {"cycle_time_s": 8.0})

    def first_executable(self, box, ranked, state, robot_state=None):
        rejected = {}
        for c in ranked:
            v = self.validate_robot_motion(box, c, state)
            if v.success:
                return c, v, rejected
            rejected[c.candidate_id] = v
        return None, None, rejected


def test_runtime_ranker_ranks_only_robot_executable_candidates(scene):
    """The planner does not know robot reach; with a bound stage-6 checker it
    only ranks executable candidates, so its Top-K never strands stage 6."""
    from pac_runtime.placer import RobotAwarePlacer

    _, box, state, context = scene
    backend = CandidateBackend(context)
    valid = [c for c in backend.generate_candidates(box, state)
             if backend.validate_constraints(box, c, state).success]
    unbound = TeamRuntimeRanker(FAST, use_time_budget=False)
    top = [c.candidate_id for c in unbound(valid, box, state, backend)]
    robot = _RejectingRobot(top)                      # everything the planner liked is unreachable
    ranker = TeamRuntimeRanker(FAST, use_time_budget=False)
    placer = RobotAwarePlacer(robot, ranker=ranker)
    assert ranker.robot is robot                      # bound by the placer
    chosen = placer(valid, box, state, backend)
    assert chosen is not None and chosen.candidate_id not in top
    assert placer.stats["no_executable"] == 0 and placer.stats["robot_rejected"] == 0
    assert ranker.provenance()["robot_prefiltered"] == len(top)
    # nothing executable: empty order, the option is infeasible for stage 4
    none_ok = TeamRuntimeRanker(FAST, use_time_budget=False)
    none_ok.bind_robot(_RejectingRobot(c.candidate_id for c in valid))
    assert none_ok(valid, box, state, backend) == [] and none_ok.last_result is None
