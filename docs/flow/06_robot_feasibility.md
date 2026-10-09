# 6. Robot 실행 가능성

**흐름도**: 하강 경로 간섭(박스+그리퍼) → Reach / IK → Collision → Payload (그리퍼 질량·Load CoM 포함)

| 구현 | 위치 | 비고 |
|---|---|---|
| **주 경로 (태현)** | `ros2_ws/src/pac_robot_check/pac_robot_check/validator.py` `RobotFeasibility` (:57): `box_descent_clear` (:78), `gripper_descent_clear` (:93), `arm_clearance` (:121), 가반하중 (:73), `validate_robot_motion` (:209), `first_executable` (:271), `cycle_time` (:197); IK/FK `kinematics.py` | `RuntimeCore`·`RobotAwarePlacer`가 사용 |
| 설정 | `config/taehyeon/robot_check.yaml`, `robot_check_gazebo.yaml` (받침대 셀) | |
| 로봇 어댑터 (재성) | `ros2_ws/src/pac_robot/pac_robot/capability.py` `RobotFeasibilityAdapter`, `hdr50_22_sim_adapter.py` `Hdr50_22Adapter`, `hdp160_31_adapter.py` | 이전 대용 로봇 설계, 중복 |
| V4.x 운동학 (재성) | `scripts/hdr50_kinematics.py` (`fk`, `ik`, `ik_near`, `cartesian_path`), `scripts/pick_place_plan_v44.py` `plan_pick_place`, `pick_place_plan_v45.py` `plan_pick_place_yaw` | `run_mission_v45.py`가 사용, 중복 |
| MoveIt 계획 | `ros2_ws/src/pac_execution/pac_execution/executor_node.py`, `scripts/moveit_pick_place_v44.py` | 실행 단계에서 다시 확인 |

- **테스트**: `tests/taehyeon/test_th_robot_check.py`, `tests/workcell/test_proxy_robot_strategy.py`, `test_robot_adapter.py`, `test_pick_place_v44.py`, `test_mission_bridge_v45.py`
- **문서**: `docs/taehyeon/robot_check.md`, `docs/jaesung/PROXY_ROBOT_STRATEGY.md`, `ROBOT_INTEGRATION_PLAN.md`
- **흐름도 연결**: 실패하면 다음 후보로 갑니다(`first_executable`, 흐름도의 "실패 시 다음 후보" 점선).
