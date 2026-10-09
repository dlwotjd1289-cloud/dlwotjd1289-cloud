# 7. 실행 → 사후 검증

**흐름도**: 로봇이 박스를 놓은 뒤, 놓기 전후 Heightmap을 비교해 계획과 실제의 편차 확인

| 구현 | 위치 | 환경 |
|---|---|---|
| 가상 실행 (태현) | `ros2_ws/src/pac_runtime/pac_runtime/executor.py` `ExecutorSim` (:56: `grip`, `check` :78, `place` :107, 단계적 대응 L0~L4); 결과 분류 `core.py` `RuntimeCore.verify` (:148); 설정 `ExecutionConfig`, `VerifyConfig` | 가상 셀 |
| Gazebo 순간이동형 (태현) | `pac_runtime/gazebo_driver.py` `GazeboDriverCore.on_command` (:154); launch `ros2_ws/src/pac_runtime/launch/gazebo_cell.launch.py` + `tools/runtime/launch/hdr50_pedestal_workcell.launch.py` | 궤적 재생 후 측정 위치에 박스 생성, 흡착 없음 |
| **Gazebo + MoveIt 물리형 (동한)** | `ros2_ws/src/pac_execution/pac_execution/`: `executor_node.py` (MoveGroup / ExecuteTrajectory, `/pac/command` → `/pac/execution_result`), `grasp_node.py` + C++ 플러그인 `ros2_ws/src/pac_gazebo_grasp/src/confirmed_grasp.cpp`, `source_node.py` (박스 생성), `audit.py` `check_placement` (측정 위치로 Hard Mask 재검사), `contract.py`, `node_io.py`, `trace_check.py`, `runtime_node.py`; launch `original_boxes_demo.launch.py` | PLACE_CURRENT만 지원 |
| V4.x 흡착 실행 (재성) | `scripts/moveit_pick_place_v44.py` `MoveItPickPlace` (:215), `run_pick_place_v44.py`, `run_mission_v45.py`, `scripts/suction_gripper_node.py`; EOAT `ros2_ws/src/pac_eoat/urdf/vacuum_gripper_v1.urdf.xacro` | Gazebo + MoveIt, 실제 흡착(DetachableJoint) |
| 물리 재생·검증 | `ros2_ws/src/pac_simulation/pac_simulation/ahead_sim/` (`simulator.py`, `robot_cell.py`, `server.py`), `scripts/verify_stack_bullet_v45.py`, `tools/runtime/scripts/physics_replay.py`, `tools/virtual_data/scripts/physics_crosscheck.py` | PyBullet |

- **테스트**: `tests/donghan/test_execution.py`; 점검 스크립트 `scripts/donghan/check_execution_trace.py`, `check_closed_loop_physics.py`, `check_robot_demo.py`, `robot_preflight.py`
- **문서**: `docs/donghan/original_boxes_demo_v3_ko.md`, `docs/jaesung/README_V44.md`, `docs/taehyeon/runtime.md`
- **미구현·주의**:
  - 흐름도의 Heightmap 전후 비교는 아직 없습니다. 위치·외곽 기하로 비교합니다(`VerifyConfig.heightmap_cell_m` 미사용).
  - 실행 스택이 3개입니다([KNOWN_ISSUES](KNOWN_ISSUES.md)).
- **편차·실패 시**: 단계적 대응 L0~L4, 영향 부분만 재계획, 집기 실패는 재시도 → 다른 집기 → 확인 영역([곁가지](09_side_flows.md))
