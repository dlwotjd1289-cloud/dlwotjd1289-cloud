# 3. Supervisor

**흐름도**: 공정 상태 NORMAL · PALLET_CHANGE · REPACKING · HOLD · Conveyor 정지/재개 · 미입고 박스 MISSING 확정

| 구현 | 위치 |
|---|---|
| 상태 관리 (태현) | `ros2_ws/src/pac_runtime/pac_runtime/supervisor.py`: `Mode` (:15), `Supervisor` (:22): `pallet_change`, `repacking`, `hold`, `can_pick`, `outstanding`, `confirm_missing` |
| 컨베이어 | `conveyor_running` 플래그, `RuntimeCore.on_conveyor_idle` (`core.py` :99, ROS 토픽 `/pac/conveyor_idle`) |
| 설정 | `config.py` `SupervisorConfig` (`missing_timeout_s`, `operator_time_s`, `pallet_change_manual`). 교체 시간 T_change는 `config/taehyeon/highlevel.yaml` `timing.pallet_change_time_s` 하나를 4단계 모델과 함께 씀 |
| Gazebo 팔레트 반출 (재성) | `scripts/pallet_swap_v44.py` (가상 AGV, 시험 중) |

- **테스트**: `test_th_runtime.py::test_supervisor_modes_and_missing`, `test_missing_confirmed_when_last_box_goes_to_inspection`
- **미구현**: 컨베이어 정지/재개는 논리 플래그뿐이고 Gazebo 구동은 없습니다.
