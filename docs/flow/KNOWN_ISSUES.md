# 알려진 문제 (2026-10-09 통합 시점)

흐름도 기준으로 코드를 대조하면서 찾은 것입니다. 수정은 담당자 확인 후 진행합니다.

## 1. 바로 고쳐야 할 것
| 문제 | 위치 | 영향 |
|---|---|---|
| **깨진 import** `make_runtime_ranker` | `ros2_ws/src/pac_execution/pac_execution/runtime_node.py:38`, `scripts/donghan/check_closed_loop_physics.py:44`, `check_robot_demo.py:39` | 이 함수가 `pac_runtime/ros_node.py`에 없습니다. 동한 님 저장소 `archive/team_runtime_files_v3/`의 패치된 `ros_node.py`에만 있어서, `verified_runtime` 노드와 점검 스크립트 2개는 import 단계에서 실패합니다. → 동한 님 패치를 태현 님 `ros_node.py`에 반영할지 두 사람 확인 필요 |
| **저장소 밖 경로 의존** | `scripts/ahead_planner_bridge_v44.py:26` | 옛 폴더 `~/AHEAD/planner_donghan_7860043`의 `pac_planning`을 불러옵니다. 통합 저장소의 `pac_planning`으로 바꿔야 합니다(재성). `pac_runtime` launch도 `repo:=pac-mission1-shared` 인자를 기대합니다. |

## 2. 같은 단계의 중복 구현 (통합 과제)
| 단계 | 구현들 | 제안 |
|---|---|---|
| 5 ①② | `pac_candidates` (팀 기준) vs `pac_planning/reference_backend.py` (오프라인 참조용, EMS 아님). V4.4 브리지는 참조용을 씀 | V4.4 브리지를 `pac_candidates`로 전환 |
| 5 순위 연결 | `pac_runtime.placer.donghan_ranker`, `pac_planning.team_bridge.TeamRuntimeRanker`, `TeamPlacer`, `pac_highlevel.value.DonghanPlacer` | 하나로 정리 |
| 6 | `pac_robot_check` (주 경로) vs `pac_robot` 어댑터 vs `scripts/hdr50_kinematics.py`·`pick_place_plan_v4x.py` | `pac_robot_check`로 통일, 나머지는 시각화·Gazebo 전용으로 한정 |
| 7 | `pac_execution` (동한) vs `scripts/moveit_pick_place_v44.py`·`run_mission_v45.py` (재성) vs `pac_runtime` executor·`gazebo_driver` (태현) | 실제 흡착이 되는 재성 V4.4 실행부를 `pac_execution` 계약(`/pac/command` → `/pac/execution_result`)에 맞춰 하나로 |
| 8 | `pac_runtime.StateManager` vs `pac_common.StateManager` vs V4.4 JSON 상태 | 공통 기준서는 단일 작성자 원칙. 어느 쪽을 기준으로 할지 팀 결정 |
| 기하 | `pac_planning/geometry.py`, `pac_candidates/geometry.py`, `pac_common/frames.py` | 공통으로 |
| 설정 | 팔레트 교체 시간이 `SupervisorConfig`와 `TimingConfig` 두 곳 | `config/default.yaml` 단일 원본으로 |

## 3. 흐름도에는 있지만 아직 없는 것
- **1 인식**: 실제 ID·라벨 판독, 깊이(RGB-D) 처리, 실제 Base-view 카메라
- **2 검증**: 실제 파손 검출기
- **3·곁가지**: 컨베이어 정지/재개 구동, 물리 NG 구역, 버퍼 선반, Gazebo 팔레트 교체 마무리(시험 중)
- **5**: ROS `runtime_node`에 ③~⑥ 순위모델 미연결(지금은 DBLF로 실행)
- **7**: Heightmap 전후 비교, `pac_execution`의 PLACE_CURRENT 외 행동
- **곁가지**: 재적재가 흐름도의 MCTS + A*가 아니라 소규모 BFS, 사이클 전체 시간 예산 관리 없음

## 4. 참조 없는 설정 파일
`config/robot.yaml`, `config/eoat.yaml`, `config/camera.yaml`은 코드에서 읽지 않습니다. `config/donghan/*.yaml`은 명령줄 인자로만 씁니다.
