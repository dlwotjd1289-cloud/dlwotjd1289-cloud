import math

import pytest
from pac_common.models import *
from pac_common.state_manager import CommitTolerance, StateManager

SIZE = Size3D(.4, .3, .2)
PLAN = Pose3D('pallet', .2, .2, 0.0, yaw=0.0)


def initial():
    return SystemState(0, 0.0, PalletState('P001', Size3D(1.1, 1.1, 1.5), ()), InventoryState({}, {'SKU_A': 2}))


def observed(box_id='B001'):
    return BoxState(box_id, 'SKU_A', SIZE, 12.0, Pose3D('conveyor', 0, 0, .9), (0.0, math.pi / 2),
                    BoxStatus.READY_FOR_PICK, 1.0, 1.0, 'sim')


def planned(m, box_id='B001', pose=PLAN):
    c = PlacementCandidate(f'S{m.snapshot.state_version:04d}-{box_id}-C000', box_id, pose, m.snapshot.state_version)
    m.register_plan(c)
    return c


def test_observation_moves_box_out_of_anonymous_stock():
    m = StateManager(initial())
    s1 = m.commit_observation(observed(), 1.0)
    assert s1.state_version == 1
    assert s1.inventory.remaining_by_sku['SKU_A'] == 1


def test_measured_pose_within_tolerance_commits_planned_pose():
    m = StateManager(initial())
    m.commit_observation(observed(), 1.0)
    c = planned(m)
    measured = Pose3D('pallet', .2 + 1e-5, .2, -1.2e-5, yaw=math.pi + 0.001)  # contact sink, 180 deg symmetric
    out = m.commit_execution(ExecutionResult(True, 'B001', c.candidate_id, measured, (), 2.0))
    assert out.placed and out.state.state_version == 2
    assert out.state.pallet.boxes[0].pose == PLAN
    assert out.state.inventory.tracked_boxes['B001'].status is BoxStatus.PLACED


def test_measured_pose_outside_tolerance_is_not_placed():
    m = StateManager(initial(), CommitTolerance())
    m.commit_observation(observed(), 1.0)
    c = planned(m)
    out = m.commit_execution(ExecutionResult(True, 'B001', c.candidate_id, Pose3D('pallet', .21, .2, 0.0), (), 2.0))
    assert not out.placed and out.codes == (RejectCode.SENSOR_UNCERTAIN,)
    assert out.state.pallet.boxes == ()
    assert out.state.inventory.tracked_boxes['B001'].status is BoxStatus.FAILED


def test_stale_plan_duplicate_and_unknown_results_are_rejected():
    m = StateManager(initial())
    m.commit_observation(observed(), 1.0)
    stale = PlacementCandidate('S0000-B001-C000', 'B001', PLAN, 0)
    with pytest.raises(ValueError, match='STALE_PLAN'):
        m.register_plan(stale)
    c = planned(m)
    with pytest.raises(ValueError):
        planned(m)  # second pending plan for the same box
    ok = ExecutionResult(True, 'B001', c.candidate_id, PLAN, (), 2.0)
    m.commit_execution(ok)
    with pytest.raises(ValueError):
        m.commit_execution(ok)  # duplicate execution result


def test_pallet_change_between_plan_and_execution_is_stale():
    m = StateManager(initial())
    m.commit_observation(observed('B001'), 1.0)
    m.commit_observation(observed('B002'), 1.5)
    c1 = planned(m, 'B001')
    c2 = planned(m, 'B002', Pose3D('pallet', .7, .2, 0.0))
    m.commit_execution(ExecutionResult(True, 'B001', c1.candidate_id, PLAN, (), 2.0))
    out = m.commit_execution(ExecutionResult(True, 'B002', c2.candidate_id, c2.target_pose, (), 3.0))
    assert not out.placed and out.codes == (RejectCode.STALE_PLAN,)
