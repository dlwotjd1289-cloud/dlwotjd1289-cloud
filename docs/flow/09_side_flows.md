# 곁가지 흐름과 입력·제약

## Inspection / NG
- **코드**: `ActionType.REJECT_NG`, 검증기의 `"INSPECTION"` 경로, `RuntimeCore` `on_result`, `pac_highlevel/world.py` `_reject` (:455)
- **흐름도**: 파손은 Reject + 관리자 알림, 인지 실패는 Base-view로 Recover, 규격 불일치는 재측정 → 실측값으로 갱신 후 진행
- **미구현**: Gazebo에 물리 NG 구역이 없습니다.

## Pallet 교체 (T_change)
- **코드**: `Supervisor.pallet_change`; 시간 T_change는 `config/taehyeon/highlevel.yaml` `timing.pallet_change_time_s` 하나(60 s)를 4단계 모델과 Supervisor가 함께 씀
- **Gazebo**: 재성 `scripts/pallet_swap_v44.py`가 팔레트 반출(가상 AGV, 시험 중)

## Buffer 선반 (팔레트 양쪽)
- **코드**: `BufferConfig` (4칸, 칸별 이동시간), `pac_common.StateManager.to_buffer`, `world.free_slot`
- **미구현**: 논리 모델뿐이고 Gazebo에 선반·동작이 없습니다.

## Partial Repack
- **코드**: `pac_highlevel/repack.py` `plan_repack`, 이동마다 6단계 검사(`core.py` `next_command`), `Supervisor.repacking`
- **흐름도와 차이**: 흐름도는 MCTS + A*인데, 구현은 BFS 기반 소규모 탐색입니다(최대 2회 이동·24노드).

## 잔여재고 (EXPECTED_UNSEEN)
- **코드**: `InventoryState.remaining_by_sku` (`pac_common/models.py` :210), `Supervisor.outstanding`, `world.remaining_by_sku` (:218), 미래 시나리오 `pac_planning/scenarios.py` `sample_scenarios` (:39)
- **이름**: `EXPECTED_UNSEEN`이라는 이름의 코드는 없습니다.

## 측정 불확실성 δ (2에서 전달)
- `pac_candidates/config.py` `UncertaintyConfig` (:91), `pallet_model.py` `box_tolerance` (:84), `PlanningContext.uncertain_box_ids`; 가상 모델 `tools/virtual_data/virtual_data/observation.py`

## Decision Budget
- **코드**: `PlannerConfig.timeout_sec` (`pac_planning/config.py` :16), `planner.py` 마감 (:189), `rollout.py` `check_budget` (:22, 소프트 마감), `RuntimeConfig.robot_checks_per_option`, `RepackConfig.max_nodes`
- **흐름도 규칙**: 시간 부족 시 Top-K 수 → 시나리오 수 → Horizon 순으로 축소. Hard Mask·안전검증은 생략 안 함
- **미구현**: 사이클 전체를 관리하는 통합 예산은 없습니다.

## 점수 가중치 결정 (Weight Sweep → Sensitivity → Pareto → Hold-out)
- **동한**: `pac_planning/experiment.py` `weight_sweep` (:286, Pareto :342), `team_training.py` `training_splits`·`benchmark_holdout`, `evaluation.py` `paired_comparison`, `model_pipeline.py` `evaluate_models`
- **태현**: `tools/tuning/` (LLM + 무작위 탐색, 검증 → 테스트), `docs/taehyeon/tuning.md`
- **테스트**: `test_th_tuning.py`, `tests/donghan/test_team_training.py`
