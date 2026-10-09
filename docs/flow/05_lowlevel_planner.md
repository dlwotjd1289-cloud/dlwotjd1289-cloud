# 5. Low-level Placement Planner — 어디에, 어떤 방향으로 놓을까

## ① 후보 생성 (태현, `ros2_ws/src/pac_candidates/pac_candidates/`)
- `candidate_generation.py`: `raw_candidates` (:308), EMS `_ems_raw` (:165), Extreme Point `extreme_points` (:207)·`_ep_raw` (:228), 균형점 `_balance_raw` (:96), 80 mm 중복 제거 `deduplicate` (:370)
- `pallet_model.py` `PalletModel` (:105, EMS `maximal_rectangles` :630)
- `backend.py` `CandidateBackend` (:88): `generate_with_report` (:258), `candidate_set` (:314)

## ② Hard Mask — 생략 불가 (태현)
- `hard_mask.py` `evaluate` (:75), `pallet_cog_limits` (:63); 하중 `loads.py` (McKee :106)
- `backend.validate_constraints` (:198); 측정 불확실성 δ `config.py` `UncertaintyConfig` (:91)
- 설정 `config/taehyeon/candidates.yaml`
- 13단계 옆 가속 안정성 (동한, 기본 꺼짐): `lateral.py` `check` (기하 → 힘 평형 LP → 짧은 경사 시험 `lateral_sim.py`). 켜는 설정 `config/donghan/candidates_lateral.yaml`, 설명 `docs/donghan/lateral_stability.md`

## ③~⑥ Feature → AI Top-K → Rollout → 최종 점수 (동한, `ros2_ws/src/pac_planning/pac_planning/`)
| 단계 | 위치 |
|---|---|
| 진입 | `planner.py` `PlacementPlanner.plan` (:174; 모드 ahead / teacher / ranking / current / greedy) |
| ③ Feature | `features.py` `compute_features` (:186) |
| ④ AI 평가 → Top-K | `model.py` `DualHeadRanker` (:18), LambdaRank `lambda_gradient` (:91); 값 헤드 = Future Value |
| ⑤ Top-K Rollout | `rollout.py` `evaluate_shared` (:116), `run_scenario` (:38), `lower_tail_cvar` (:87); 시나리오 `scenarios.py` `sample_scenarios` (:39) |
| ⑥ 최종 점수 | `scoring.py` `score_terms` (:4: safety / space / future / risk / time), `priority` (:21) |
| 태현 후보기와 연결 | `team_bridge.py` `plan_with_backend` (:34), `TeamPlacer` (:95), `TeamRuntimeRanker` (:134) |
| ROS 서비스 | `planning_service.py` (`/pac/plan_placement`, srv `ros2_ws/src/pac_planning_interfaces/srv/PlanPlacement.srv`), launch `ros2_ws/src/pac_planning/launch/placement_planner.launch.py` |
| 학습·평가 | `model_pipeline.py`, `training.py`, `training_data.py`, `evaluation.py`, `team_training.py`; 스크립트 `scripts/donghan/model_pipeline.py`; 설정 `config/donghan/model_*.yaml`, `candidate_runtime_v2.yaml` |

- **런타임 연결**: `ros2_ws/src/pac_runtime/pac_runtime/placer.py` `RobotAwarePlacer` (:22, DBLF 또는 동한 순위 → 6단계 검사), `donghan_ranker` (:48, `TeamRuntimeRanker`로 위임). ROS: `ros_node.make_runtime_ranker` (`ranker:=dblf|donghan`)
- **테스트**: 태현 `test_th_candidates.py`, `test_th_hard_mask.py`, `test_th_geometry.py`, `test_th_planner_integration.py`, `test_th_strength.py` / 동한 `test_planning.py`, `test_learning.py`, `test_rollout.py`, `test_scalable_training.py`, `test_team_bridge.py`, `test_team_load_regression.py`, `test_regressions.py`, `test_contracts.py`, `test_team_physics.py`, `test_runtime_teacher.py`
- **문서**: `docs/donghan/low_level_planner.md`, `integration.md`, `model_14400_v3_ko.md` / `docs/taehyeon/algorithms.md`, `interface.md`, `virtual_data.md`
- **주의**:
  - ROS `runtime_node`의 기본값은 여전히 `ranker:=dblf`입니다. ③~⑥을 쓰려면 `ranker:=donghan`을 지정하세요.
  - 학습 모델은 학습 때의 플래너 설정(`ranker_config`)과 함께 써야 합니다. 다르면 시작할 때 오류가 납니다.
  - 순위 어댑터는 `pac_planning.team_bridge` 하나로 정리했습니다(`TeamRuntimeRanker` / `TeamPlacer`).
