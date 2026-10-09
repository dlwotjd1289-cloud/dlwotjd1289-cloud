# 8. 상태 갱신

**흐름도**: 실측 기준으로 Pallet · 팔레트 CoG·총하중 · Inventory · Buffer 갱신 → 다음 박스로

| 구현 | 위치 | 사용처 |
|---|---|---|
| **State Manager (단일)** | `ros2_ws/src/pac_common/pac_common/state_manager.py` `StateManager`: 런타임 `arrive`, `to_buffer`, `place`(확정 규칙 적용), `reconcile`, `cog_and_weight`, `close_pallet`, `confirm_missing`, `context`; 스냅샷 `commit_observation`, `register_plan`, `commit_execution`; `CommitTolerance` (xy 5 mm / z 3 mm / yaw 1°) | `RuntimeCore`(`StateManager.for_order`), Gazebo V4.4 commit, PyBullet 폐루프 |

- **확정 규칙**: 측정 pose가 계획과 허용오차 안이면 계획 pose, 밖이면 다시 측정한 pose(`reconcile` 후). `commit_execution`(단발 실행기)은 허용오차 밖이면 FAILED + `SENSOR_UNCERTAIN`.
- **Inventory**: PLACED 박스도 팔레트를 닫을 때까지 `tracked_boxes`에 status로 남습니다(기준서 11장).
- **테스트**: `tests/workcell/test_state_manager.py`, `tests/taehyeon/test_th_runtime.py::test_state_manager_*`
- **삭제한 중복**: `pac_runtime/state_manager.py`, V4.4 브리지의 JSON 직접 기록.
