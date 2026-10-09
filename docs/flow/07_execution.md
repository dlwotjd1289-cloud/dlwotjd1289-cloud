# 7. 실행 → 사후 검증

**흐름도**: 로봇이 박스를 놓은 뒤, 놓기 전후 Heightmap을 비교해 계획과 실제의 편차 확인

| 구현 | 위치 | 환경 |
|---|---|---|
| 가상 실행 (태현) | `ros2_ws/src/pac_runtime/pac_runtime/executor.py` `ExecutorSim` (`grip`, `check`, `place`, 단계적 대응 L0~L4; L0 = 공통 확정 허용오차); 결과 분류 `core.py` `RuntimeCore.verify`; 설정 `ExecutionConfig`, `VerifyConfig` | 가상 셀 |
| **Gazebo 실행 (재성, 단일)** | V4.5 미션 `scripts/run_mission_v45.sh` → `plan_placement_v45.py`(5·6단계 팀 구현) → `run_mission_v45.py`(관절 경로 + 흡착) → `ExecutionResult` JSON. 다중 박스 사이클 `run_generator_cycle_v44.sh` + `ahead_planner_bridge_v44.py`(plan: 5·6단계, commit: 8단계 확정 규칙) + `moveit_pick_place_v44.py`. 흡착 `scripts/suction_gripper_node.py` (DetachableJoint) | Gazebo Fortress + MoveIt (V4.4 월드) |
| 물리 재생·검증 | `ros2_ws/src/pac_simulation/pac_simulation/ahead_sim/`, `scripts/verify_stack_bullet_v45.py`(5·6단계 → PyBullet → StateManager 폐루프), `tools/runtime/scripts/physics_replay.py`, `tools/virtual_data/scripts/physics_crosscheck.py` | PyBullet |

- **테스트**: `tests/workcell/test_planner_bridge_v44.py`, `test_mission_bridge_v45.py`, `test_pick_place_v44.py`, `test_state_manager.py`
- **문서**: `docs/jaesung/README_V44.md`, `docs/taehyeon/runtime.md`
- **삭제한 중복**: `pac_execution`·`pac_gazebo_grasp`(1.2 × 1.0 m 팔레트를 강제하던 원박스 데모), `pac_runtime/gazebo_driver.py`(순간이동형), `tools/realtime/gazebo_replay.py`, 이전 월드·launch V2~V4.1.
- **미구현·주의**:
  - 흐름도의 Heightmap 전후 비교는 아직 없습니다. 위치·외곽 기하로 비교합니다(`VerifyConfig.heightmap_cell_m` 미사용).
  - Gazebo 경로는 ROS `runtime_node`(`/pac/command`)로 구동하지 않고, 스크립트가 5·6·8단계 팀 함수를 직접 부릅니다. 4단계(버퍼·팔레트 교체·보류)는 `run_generator_cycle_v44.sh`가 자체 규칙으로 결정합니다([IMPLEMENTATION_CONFLICTS](IMPLEMENTATION_CONFLICTS.md) C1).
- **편차·실패 시**: 단계적 대응 L0~L4, 영향 부분만 재계획, 집기 실패는 재시도 → 다른 집기 → 확인 영역([곁가지](09_side_flows.md))
