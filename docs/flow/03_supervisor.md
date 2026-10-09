# 3. Supervisor

**흐름도**: 공정 상태 NORMAL · PALLET_CHANGE · REPACKING · HOLD · Conveyor 정지/재개 · 미입고 박스 MISSING 확정

| 구현 | 위치 |
|---|---|
| 상태 관리 (태현) | `ros2_ws/src/pac_runtime/pac_runtime/supervisor.py`: `Mode` (:15), `Supervisor` (:22): `pallet_change`, `repacking`, `hold`, `can_pick`, `outstanding`, `confirm_missing` |
| 컨베이어 | `conveyor_running` 플래그, `RuntimeCore.on_conveyor_idle` (`core.py` :99, ROS 토픽 `/pac/conveyor_idle`) |
| 설정 | `config.py` `SupervisorConfig` (`pallet_change_time_s`, `missing_timeout_s`, `operator_time_s`) |
| Gazebo 팔레트 반출 (재성) | `scripts/pallet_swap_v44.py` (가상 AGV, 시험 중) |

- **테스트**: `test_th_runtime.py::test_supervisor_modes_and_missing`, `test_missing_confirmed_when_last_box_goes_to_inspection`
- **미구현**: 컨베이어 정지/재개는 논리 플래그뿐이고 Gazebo 구동은 없습니다. `pac_execution`은 PLACE_CURRENT 외 명령을 받으면 `CELL_ACTUATOR_REQUIRED`로 거절합니다(`contract.py` :136).
- **주의**: 팔레트 교체 시간 60 s가 `SupervisorConfig`와 `pac_highlevel` `TimingConfig` 두 곳에 있습니다. 함께 바꿔야 합니다.
