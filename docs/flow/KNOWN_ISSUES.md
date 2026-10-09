# 알려진 문제 (2026-10-09 정리 후)

흐름도 기준으로 코드를 대조하면서 찾은 것입니다. 최종 결정사항과의 충돌은 [DECISION_REVIEW](DECISION_REVIEW.md), 구현끼리의 충돌(재성 Gazebo·v3 ↔ 팀 런타임)은 [IMPLEMENTATION_CONFLICTS](IMPLEMENTATION_CONFLICTS.md)에 있습니다.

## 1. 해결됨 (2026-10-09 정리)
| 문제 | 처리 |
|---|---|
| 같은 단계 중복 구현 | 단계마다 하나만 남김 (아래 2장 표) |
| PPO 정책 (1.35 m 기준 학습) | 사용하지 않기로 결정, 코드·모델·학습 스크립트·`rl` 의존성 삭제 |
| `pac_runtime` launch 예시의 `repo:=pac-mission1-shared` | `repo:=` 기본값을 `$PAC_REPO`(`scripts/env.sh`)로 |
| 참조 없는 설정 `config/robot.yaml`·`eoat.yaml`·`camera.yaml`·`local.example.yaml`, 생성기 `robot.yaml`·`local.example.yaml` | 삭제 |
| 팔레트 교체 시간이 `SupervisorConfig`·`TimingConfig` 두 곳 | `highlevel.yaml` `timing` 하나 |
| 기하 함수 3벌 (`pac_planning/geometry.py`, `pac_candidates/geometry.py`, `pac_common/frames.py`) | 기본 함수(`quarter_turns`, `rotated_dims`, 모서리 ↔ 중심)는 `pac_common.frames` 하나, 나머지는 거기서 import |
| 병합 후 깨진 경로: `run_auto_scale_v43.sh`의 박스 SDF, 형제 checkout을 가리키던 기본 경로, `.deps/` 추출 방식 | 저장소 안 경로로 수정, `.deps`·`fetch_team_deps.sh` 삭제 |
| `test_box_perception_v44.py`가 `cv2`/`rclpy` 없으면 pytest 전체를 중단 | 없으면 skip |
| ROS/Gazebo 환경 미구성 | `docker/`, `scripts/setup_ubuntu22.sh`, `scripts/env.sh` |
| 깨진 import `make_runtime_ranker` 등 (이전 정리) | `fix/planner5-runtime-integration`에서 해결 |

## 2. 단계별 단일 구현
| 단계 | 남긴 구현 | 삭제한 중복 |
|---|---|---|
| 4 | `pac_highlevel` Rule / Look-ahead | MaskablePPO(NumPy·sb3), gym 환경, 학습기. **남은 중복**: Gazebo 사이클 스크립트의 자체 규칙 (IMPLEMENTATION_CONFLICTS C1). 런타임은 Rule만 연결 (C30) |
| 5 ①② | `pac_candidates` | `pac_planning/reference_backend.py`, V4.4 브리지의 오프라인 후보기 |
| 5 ③~⑥ | `pac_planning` (`team_bridge.plan_with_backend`, `planning_service.plan_request`) | — |
| 6 | `pac_robot_check` | `pac_robot` 어댑터, `robot_check_gazebo.yaml`, V4.x 자체 IK 판정·도달 높이 판정 |
| 7 (Gazebo) | V4.4/V4.5 흡착 실행 (`run_mission_v45.py`, `moveit_pick_place_v44.py`) | `pac_execution`, `pac_gazebo_grasp`, `pac_runtime/gazebo_driver.py`, `tools/realtime/gazebo_replay.py` |
| 8 | `pac_common.StateManager` | `pac_runtime/state_manager.py`, V4.4 브리지 JSON 직접 기록 |
| 작업셀 | V4.2 배치 + V4.4 흡착 월드 (`config/workcell.yaml`) | V2(HDP160)~V4.1 월드·launch, `pac_eoat`. V4.6(`jaesung/workcell-v46`, 미병합)은 배치를 환경변수로 바꿈 (C29) |

동한 님 `overlap_review_20261009.md`는 `gazebo_driver`(재생용), `pac_execution` 계약 검사, `ReferenceBackend`(오프라인 참조)를 남기자고 제안합니다. 이번 정리에서 삭제했으므로 유지 여부는 팀이 정해야 합니다(git 기록에서 복원 가능) → [IMPLEMENTATION_CONFLICTS 3차 대조표](IMPLEMENTATION_CONFLICTS.md#동한-님-overlap_review_20261009md-대조).

## 3. 흐름도에는 있지만 아직 없는 것
- **1 인식**: 실제 ID·라벨 판독, 깊이(RGB-D) 처리의 Gazebo 연결, 실제 Base-view 카메라
- **2 검증**: 실제 파손 검출기. Gazebo 브리지는 검증기를 거치지 않음
- **3·곁가지**: 컨베이어 정지/재개 구동, 물리 NG 구역, 버퍼 선반, Gazebo 팔레트 교체 마무리(시험 중)
- **4 → 7 Gazebo**: ROS `runtime_node`의 `/pac/command`를 Gazebo 실행기가 받지 않음. Gazebo 사이클은 5·6·8단계 팀 함수를 직접 호출하고, 4단계는 스크립트 자체 규칙(버퍼 2칸·팔레트 교체·보류)으로 결정 → [IMPLEMENTATION_CONFLICTS](IMPLEMENTATION_CONFLICTS.md)
- **6**: 컨베이어 → 팔레트 운반 경로 검사 (V4.4 브리지가 운반 높이만 확인)
- **7**: Heightmap 전후 비교
- **곁가지**: 재적재가 흐름도의 MCTS + A*가 아니라 소규모 BFS, 사이클 전체 시간 예산 관리 없음
- **ALGORITHM_V3 결정 중 미구현**: [DECISION_REVIEW 4장](DECISION_REVIEW.md#4-결정됐지만-아직-구현되지-않은-것-algorithm_v3_draft-0장)

## 4. 확인된 버그 (3차 검토, 미수정)
자세한 내용은 [IMPLEMENTATION_CONFLICTS](IMPLEMENTATION_CONFLICTS.md) 해당 항목.

| # | 위치 | 문제 |
|---|---|---|
| C30 | `pac_highlevel.runtime` | `load_policy`에 Look-ahead 없음, `HighLevelDecider`의 미리보기 박스 0개 |
| C31 | `pac_runtime/executor.py` | 측정 yaw가 90° 배수에서 벗어나면 `ValueError`(ROS 노드 정지), 데크 위 z +2 mm면 L4 |
| C32 | `pac_runtime/supervisor.py`, `core.py` | 모드가 즉시 NORMAL로 복귀, `PALLET_CLOSE`가 교체 시간을 거치지 않음 |
| C33 | `scripts/scale_cycle_core_v43.py` | 0.5 kg 미만 박스에서 Gazebo 사이클 중단 |
| C34 | `models/pybullet/hdr50_22_suction.urdf` | 메시 경로가 `/home/jaesung/...` 절대 경로 |
| — | `run_ahead_cycle_v44.sh`, `run_moveit_pick_place_v44.sh` | 요구하는 `/pac/cctv_pick/image`가 V4.4·V4.6 월드에 없음 |

## 5. 명령줄로만 쓰는 설정
`config/donghan/model_*.yaml`, `candidate_runtime_v2.yaml`, `team_fd683e56_smoke.yaml`은 `scripts/donghan/model_pipeline.py --config`로 넘기는 실행 설정입니다(코드가 자동으로 읽지 않음).
