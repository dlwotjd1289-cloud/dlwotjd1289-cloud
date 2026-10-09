# 실행 진입점

## ROS 2 노드와 launch
| 패키지 | 노드 | launch | 역할 |
|---|---|---|---|
| `pac_runtime` (태현) | `runtime_node` (`ros_node.py` :120; `/pac/observation`, `/pac/execution_result`, `/pac/conveyor_idle` → `/pac/command`, `/pac/status`), `gazebo_cell` | `runtime.launch.py`, `gazebo_cell.launch.py` | 1~8단계 코어, Gazebo 순간이동 셀 |
| `pac_execution` (동한) | `verified_runtime`, `moveit_executor`, `gazebo_grasp`, `gazebo_box_source` | `original_boxes_demo.launch.py` | Gazebo + MoveIt 물리 실행 |
| `pac_planning` (동한) | `placement_planner_node` | `placement_planner.launch.py` | 5단계 ③~⑥ 서비스 |
| `pac_bringup` (재성) | — | `hdr50_workcell*.launch.py` (v3~v4_2), `hdr50_workcell_v4_4_pick.launch.py`, `hdr50_moveit_v44.launch.py` | 작업셀 월드, MoveIt |

## 실행 스크립트
- **재성 V4.x (Gazebo)**:
  - `scripts/run_full_cycle_v44.sh`
  - `run_ahead_cycle_v44.sh`: 계량 → CCTV → 플래너 브리지 → MoveIt
  - `run_generator_cycle_v44.sh`, `run_stack_v44.sh`, `run_review_v44.sh`, `run_pick_place_v44.sh`, `run_moveit_pick_place_v44.sh`, `run_auto_scale_v43.sh`, `run_mission_v45.sh`
  - 준비: `preflight.sh`, `build_ros_ws.sh`
- **태현 (오프라인)**: `tools/runtime/scripts/run_runtime.py` (가상 셀 1~8 루프), `scripts/taehyeon/run_validation.sh`, `tools/highlevel/scripts/*`, `tools/tuning/scripts/llm_tune.py`
- **동한 (학습·시연)**: `scripts/donghan/model_pipeline.py`, `prepare_robot_demo.py`, `robot_preflight.py`, `check_*.py`
- **데이터 생성**: `tools/ahead_dataset_generator/run_sample.sh`, `run_benchmark.sh`

## 테스트
- **전체**: `python3 -m pytest -q` (저장소 루트, 2026-10-09 기준 388 passed, 4 skipped)
