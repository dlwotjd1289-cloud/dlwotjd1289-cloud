# 8. 상태 갱신

**흐름도**: 실측 기준으로 Pallet · 팔레트 CoG·총하중 · Inventory · Buffer 갱신 → 다음 박스로

| 구현 | 위치 | 사용처 |
|---|---|---|
| **런타임 상태 (태현)** | `ros2_ws/src/pac_runtime/pac_runtime/state_manager.py` `StateManager` (:16, 가변): `arrive`, `to_buffer`, `place`, `reconcile` (:109), `cog_and_weight` (:66), `close_pallet`, `confirm_missing`, `snapshot`, `context` | `RuntimeCore` |
| 공통 상태 (공통 기준 v0.3) | `ros2_ws/src/pac_common/pac_common/state_manager.py` `StateManager` (:63, 불변·단일 작성자): `commit_observation`, `register_plan`, `commit_execution`, `CommitTolerance` (xy 5 mm / z 3 mm / yaw 1°) | `scripts/verify_stack_bullet_v45.py`, 테스트 |
| V4.4 브리지 상태 (재성) | `scripts/ahead_planner_bridge_v44.py` `cmd_commit` (:244, JSON 파일) | V4.4 Gazebo 사이클 |

- **테스트**: `tests/taehyeon/test_th_runtime.py::test_state_manager_*`, `tests/workcell/test_state_manager.py`
- **주의**: 상태 관리가 세 갈래입니다. 공통 기준서는 단일 작성자 원칙을 정했지만, 실제 런타임은 `pac_runtime` 것을 씁니다([KNOWN_ISSUES](KNOWN_ISSUES.md)).
