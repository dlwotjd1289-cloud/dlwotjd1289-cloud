# 1. 인식

**흐름도**: 무게 측정 → ID·Label 연결 → Top-view RGB-D(주 카메라) → Box State 생성 · Base-view(보조)는 이상 감지 시에만

| 구현 | 위치 | 환경 |
|---|---|---|
| 가상 인식 (태현) | `ros2_ws/src/pac_runtime/pac_runtime/perception.py`: `PerceptionSim` (:33), `observe` (:42), `base_view` (:52, 이상 시만), `to_box_state` (:62). 잡음 설정 `config.py` `PerceptionConfig` | 가상 셀, ROS `runtime_node` |
| 자동 계량 (재성) | `scripts/scale_cycle_core_v43.py` `AutoScaleCycle` (:78, ROS 없는 상태기계), `scripts/run_auto_scale_v43.py` | Gazebo |
| CCTV 박스 위치 (재성) | `scripts/box_perception_v44.py` `BoxPerception` (:283), `localize` (:99), `fit_known_rect` (:181); 기둥 CCTV + 손목 카메라, RGB만(높이는 SKU 값). `scripts/perceive_once_v44.py` | Gazebo |
| 관측 생성기 (재성, 대체용) | `ros2_ws/src/pac_perception/pac_perception/observation_generator.py` (정답 그대로 전달) | 테스트용 |

- **테스트**: `tests/taehyeon/test_th_runtime.py` (`test_label_failure_uses_the_base_view_then_inspection`), `tests/workcell/test_auto_scale_v43.py`, `test_box_perception_v44.py`, `test_observation_boundary.py`
- **문서**: `docs/taehyeon/runtime.md`, `docs/jaesung/README_V43.md`, `docs/jaesung/README_V44.md`
- **미구현**: 실제 ID·라벨 판독기, 깊이(RGB-D) 처리, 실제 Base-view 카메라. Gazebo에서는 라벨을 도착 정보나 크기 매칭으로 대신합니다(`scripts/ahead_planner_bridge_v44.py` `match_sku`).
