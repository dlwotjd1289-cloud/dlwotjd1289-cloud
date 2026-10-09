# 실행 진입점

## 환경
- **Docker (권장)**: `docker/run.sh` (최초 실행 시 `docker/Dockerfile` 빌드: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress, MoveIt 2, ros2_control, gz_ros2_control)
- **Ubuntu 22.04 직접 설치**: `bash scripts/setup_ubuntu22.sh`
- **매 셸**: `source scripts/env.sh` (ROS + 워크스페이스, `PAC_REPO`, `PAC_COMMON_CONFIG`)
- **빌드**: `bash scripts/build_ros_ws.sh` (현대로보틱스 서브모듈 포함 전체 패키지)

## ROS 2 노드와 launch
| 패키지 | 노드 | launch | 역할 |
|---|---|---|---|
| `pac_runtime` (태현) | `runtime_node` (`ros_node.py`; `/pac/observation`, `/pac/execution_result`, `/pac/conveyor_idle` → `/pac/command`, `/pac/status`) | `runtime.launch.py` (`repo:=` 기본값 `$PAC_REPO`) | 1~8단계 코어 |
| `pac_planning` (동한) | `placement_planner_node` | `placement_planner.launch.py` | 5단계 ③~⑥ 서비스 `/pac/plan_placement` |
| `pac_bringup` (재성) | — | `hdr50_workcell_v4_2.launch.py`, `hdr50_workcell_v4_4_pick.launch.py` (`GZ_GUI=0`이면 화면 없이 서버만), `hdr50_moveit_v44.launch.py` | 작업셀 월드, MoveIt |

## 실행 스크립트
- **Gazebo (재성, V4.4 월드)**:
  - `scripts/run_mission_v45.sh`: 계량 → 팀 5·6단계 → 로봇 적재 → `ExecutionResult`
  - `scripts/run_generator_cycle_v44.sh`: 생성기 시나리오 다중 박스 사이클 (`ahead_planner_bridge_v44.py` plan/commit)
  - `run_ahead_cycle_v44.sh`, `run_full_cycle_v44.sh`, `run_review_v44.sh`, `run_pick_place_v44.sh`, `run_moveit_pick_place_v44.sh`, `run_auto_scale_v43.sh`
  - 준비: `preflight.sh`, `verify_hdr50_proxy.sh`
- **PyBullet 폐루프**: `scripts/verify_stack_bullet_v45.py`, `tools/runtime/scripts/physics_replay.py`
- **0.3 g 안정성 검사 검증**: `tools/stability/run_stability_validation.py` (설정 `config/stability_validation.yaml`, 설명 [STABILITY_VALIDATION](STABILITY_VALIDATION.md))
- **태현 (오프라인)**: `tools/runtime/scripts/run_runtime.py` (가상 셀 1~8 루프), `scripts/taehyeon/run_validation.sh`, `tools/highlevel/scripts/*`, `tools/tuning/scripts/llm_tune.py`
- **동한 (학습·평가)**: `scripts/donghan/model_pipeline.py`, `train_team_model.py`, `run_team_replay.py`, `benchmark_teacher.py`
- **데이터 생성**: `tools/ahead_dataset_generator/run_sample.sh`, `run_benchmark.sh`

## 테스트
- **전체**: `python3 -m pytest -q` (저장소 루트). ROS를 source한 Docker 환경에서 483 passed (2026-10-09); ROS 없이 실행하면 ROS·PyBullet 의존 테스트는 skip
