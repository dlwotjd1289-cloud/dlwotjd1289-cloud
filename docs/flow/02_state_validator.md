# 2. State Validator

**흐름도**: 라벨·무게·크기 교차 검증 + 인식 확신도 → 이상 원인 분류: 파손 / 규격·무게 불일치 / 인지 실패

| 구현 | 위치 |
|---|---|
| 검증기 (태현) | `ros2_ws/src/pac_runtime/pac_runtime/state_validator.py`: `Anomaly` (:24: OK / RECOGNITION_FAIL / DAMAGED / SPEC_MISMATCH / UNKNOWN_SKU), `Verdict` (:33), `StateValidator.validate` (:45). 설정 `config.py` `ValidatorConfig` |
| 후속 연결 | 판정의 `uncertain` → 측정 불확실성 δ(`PlanningContext.uncertain_box_ids`), `no_load_on_top` → 상부 허용하중 0(`capacity_overrides_n`) |
| 거절 코드 | `ros2_ws/src/pac_common/pac_common/models.py` `RejectCode` (:84) 재사용 |
| 재인식 RI 코드 (동한, 제안) | `ros2_ws/src/pac_reinspection/` `ReinspectionValidator`: `StateValidator`를 수정 없이 감싸 재판독·재계량·재촬영(상한·시간 예산)과 RI 코드(`Verdict.notes`)를 더함. 같은 `validate` 시그니처. **`core.py`에는 아직 연결 안 됨.** 규정 `docs/donghan/reinspection_policy.md`, 설정 `config/donghan/reinspection_policy.yaml`(가정치) |

- **테스트**: `tests/taehyeon/test_th_runtime.py` (`test_validator_ok_*`, `test_damage_policy_reject_or_no_load`, `test_spec_mismatch_is_remeasured_*`), `tests/donghan/test_reinspection.py`; 교체 확인 `tools/donghan/reinspection_drop_in.py`
- **미구현**: 실제 파손 검출기. 확신도는 시뮬레이터 값입니다. 재성 V4.x Gazebo 파이프라인에는 검증기가 연결되어 있지 않습니다.
- **연결**: 이상 박스는 [곁가지: Inspection/NG](09_side_flows.md#inspection--ng)
