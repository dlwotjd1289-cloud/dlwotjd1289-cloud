# 6. Robot 실행 가능성

**흐름도**: 하강 경로 간섭(박스+그리퍼) → Reach / IK → Collision → Payload (그리퍼 질량·Load CoM 포함)

| 구현 | 위치 |
|---|---|
| **단일 구현 (태현)** | `ros2_ws/src/pac_robot_check/pac_robot_check/validator.py` `RobotFeasibility`: `box_descent_clear`, `gripper_descent_clear`, `arm_clearance`, 가반하중 `payload`, `validate_robot_motion`, `first_executable`, `cycle_time`; IK/FK `kinematics.py` (공식 hdr_description URDF, 폐형식 IK) |
| 설정 | `config/taehyeon/robot_check.yaml` = 팀 작업셀(`config/workcell.yaml`, V4.2/V4.4): 로봇 base_link 월드 (0, 0, 0.40), 팔레트 중심 (0, 1.20), V4.4 흡착컵(TCP 0.06 m). `tests/test_config_consistency.py`가 `workcell.yaml`·받침대 URDF와 대조 |
| 사용처 | `RuntimeCore`·`RobotAwarePlacer`(가상 셀·ROS), Gazebo 브리지 `scripts/mission_bridge_v45.py` `plan_ranked` (V4.4 사이클, V4.5 미션, PyBullet 폐루프) |
| Gazebo 경로 생성 (7단계) | `scripts/hdr50_kinematics.py`는 `pac_robot_check.kinematics`를 감싼 좌표 변환뿐(flange 기준, 받침대 높이). `pick_place_plan_v44/v45.py`는 6단계를 통과한 후보의 관절 경로를 만듭니다 |

- **테스트**: `tests/taehyeon/test_th_robot_check.py`, `tests/test_config_consistency.py` (6단계 관절값 → Gazebo 박스 위치 2 mm 이내 일치), `tests/workcell/test_pick_place_v44.py`, `test_mission_bridge_v45.py`
- **문서**: `docs/taehyeon/robot_check.md`
- **흐름도 연결**: 실패하면 다음 후보로 갑니다(`first_executable`; Gazebo 브리지는 `plan_ranked`의 `executable` 순서).
- **삭제한 중복**: `pac_robot`(fail-closed 어댑터, HDP160-31 보관본), `robot_check_gazebo.yaml`(이전 1.2 × 1.0 m 월드의 받침대 셀), V4.x 스크립트의 자체 IK 판정.
- **남은 범위 밖**: 컨베이어 → 팔레트 운반 경로(transfer)는 6단계가 보지 않습니다. V4.4 브리지가 운반 높이(TCP z ≤ 1.95 m)만 따로 확인합니다.
