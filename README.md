# PAC 2026 · HD현대로보틱스 미션 1 — AHEAD Mixed Palletizing (팀 통합 저장소)

동한·태현·재성 세 사람의 작업을 **하나의 모노레포**로 합친 저장소입니다.
기존 공유 저장소 `yang8988/pac-mission1-shared`의 모든 브랜치와 `dlwotjd1289-cloud/pac2026-ahead`를
**커밋 이력째로** 병합했고, 공통 기준 [v0.3.1](docs/common_development_standard.md)에 맞게 정리했습니다.
2026-10-09 정리에서 **흐름도 단계마다 구현을 하나만** 남겼습니다([단계별 단일 구현](docs/flow/KNOWN_ISSUES.md#2-단계별-단일-구현), [결정사항 충돌 검토](docs/flow/DECISION_REVIEW.md)).

- 목표 로봇: **HDR50-22** (2026-10-08 결정)
- 팔레트: **1.10 × 1.10 m**, 데크 0.15 m, **데크 위 적재 1.5 m**, 총중량 1000 kg → [`config/default.yaml`](config/default.yaml) 한 곳에서만 변경
- 4단계 행동 선택: Rule / Look-ahead (**PPO 미사용**, 2026-10-09)
- 환경: Ubuntu 22.04 · ROS 2 Humble · Gazebo Fortress · MoveIt 2 · Python 3.10 → [`docker/`](docker/Dockerfile)

## 전체 흐름과 담당

```mermaid
flowchart TD
    G[재성: 데이터 생성기 / Gazebo 계량 V4.3] -->|BoxState 무게·크기| S[State Manager<br/>pac_common.StateManager]
    S -->|SystemState snapshot| H[태현 4: High-level 행동 선택 Rule/Look-ahead]
    H -->|PLACE / RETRIEVE| C[태현 5-①② 후보 생성 · Hard Mask]
    C --> D[동한 5-③~⑥ 특징 · AI Top-K · 미래 rollout · 최종 순위]
    D -->|PLANNED 후보 순위| R[태현 6: pac_robot_check HDR50-22]
    R -->|통과| E[재성 7: Gazebo V4.4 집기·놓기 / PyBullet]
    R -->|IK_FAIL: 다음 후보| D
    E -->|ExecutionResult| S
```

| 파트 | 담당 | 코드 | 문서 |
|---|---|---|---|
| 공통 자료형·State Manager(8)·frames·공통 설정 | 공통 | `ros2_ws/src/pac_common`, `config/default.yaml`, `config/workcell.yaml` | [기준서 v0.3.1](docs/common_development_standard.md) |
| 1~3·7 런타임 코어와 1→8 루프 | 태현 | `ros2_ws/src/pac_runtime`, `tools/runtime` | [docs/taehyeon/runtime.md](docs/taehyeon/runtime.md) |
| 4 High-level (Rule / Look-ahead) | 태현 | `ros2_ws/src/pac_highlevel`, `tools/highlevel` | [docs/taehyeon/highlevel.md](docs/taehyeon/highlevel.md) |
| 5-①② 후보 생성·Hard Mask, 가상데이터 | 태현 | `ros2_ws/src/pac_candidates`, `tools/virtual_data` | [docs/taehyeon/README.md](docs/taehyeon/README.md) |
| 5-③~⑥ 배치 평가·순위, ROS 서비스 | 동한 | `ros2_ws/src/pac_planning`, `pac_planning_interfaces` | [docs/donghan/README.md](docs/donghan/README.md), [integration.md](docs/donghan/integration.md) |
| 데이터 생성기 | 재성 | `tools/ahead_dataset_generator` | [README](tools/ahead_dataset_generator/README.md) |
| 6 로봇 실행 가능성 (HDR50-22) | 태현 | `ros2_ws/src/pac_robot_check`, `config/taehyeon/robot_check.yaml` | [docs/taehyeon/robot_check.md](docs/taehyeon/robot_check.md) |
| PyBullet 팔레트 시뮬레이터·뷰어 | 재성 | `ros2_ws/src/pac_simulation/pac_simulation/ahead_sim`, `viewer/` | [docs/jaesung/README_AHEAD_LIVE_SIMULATOR.md](docs/jaesung/README_AHEAD_LIVE_SIMULATOR.md) |
| Gazebo 작업셀 V4.2/V4.4, 계량·흡착 실행(7) | 재성 | `pac_simulation/worlds`, `pac_bringup`, `scripts/*v43*`/`*v44*` | [README_WORKCELL](docs/jaesung/README_WORKCELL.md), [V4.3](docs/jaesung/README_V43.md), [V4.4](docs/jaesung/README_V44.md) |
| V4.5 통합 (계량 → 팀 5·6단계 → 로봇) | 재성 | `scripts/*v45*` | 아래 "V4.5" |

## 설치와 테스트

### Docker (권장: ROS 2 Humble + Gazebo Fortress + MoveIt 2)

```bash
git clone --recurse-submodules https://github.com/dlwotjd1289-cloud/dlwotjd1289-cloud.git pac2026
cd pac2026
docker/run.sh                      # 처음 한 번 이미지 빌드 후 컨테이너 셸 (저장소는 /ws/pac2026)
# 컨테이너 안에서
bash scripts/build_ros_ws.sh       # 팀 패키지 + 현대로보틱스 서브모듈 전체 colcon build
source scripts/env.sh              # ROS + 워크스페이스 + PAC_REPO / PAC_COMMON_CONFIG
python3 -m pytest -q               # 2026-10-09: 460 passed (ROS를 source한 컨테이너)
```

Linux 호스트에서 `DISPLAY`가 있으면 Gazebo·RViz 창이 호스트에 뜹니다. 창 없이 돌리려면 `GZ_GUI=0`.

### Ubuntu 22.04에 직접 설치

```bash
bash scripts/setup_ubuntu22.sh     # ROS apt 저장소, 필요한 패키지, 서브모듈, rosdep, colcon build, pytest
source scripts/env.sh              # 새 셸마다
```

### ROS 없이 Python만

```bash
python3 -m pytest -q               # 루트 conftest.py가 패키지 경로를 잡음 (ROS·PyBullet 의존 테스트는 skip)
python3 -m venv .venv && . .venv/bin/activate && pip install -e '.[dev,sim]'   # 패키지 설치가 필요하면
```

venv 안에서 pytest를 돌릴 때 ROS가 source된 셸이면 ROS pytest 플러그인 때문에 실패할 수 있습니다 → `env -u PYTHONPATH python -m pytest -q`.

## 자주 쓰는 실행

```bash
# 동한: 플래너 데모 / 팀 연결 재생 (생성기 데이터 → 태현 Rule·후보 → 동한 평가)
python3 -m pac_planning.demo --model models/dual_head_ranker.json --fixed-work
python3 scripts/donghan/run_team_replay.py --dataset runs/v45_dataset --max-boxes 5

# 태현: 가상데이터 생성·검증, 전체 검증 리포트
python3 tools/virtual_data/scripts/generate_virtual_data.py --run-generator sample --sample-per-family 2 --output runs/vd
scripts/taehyeon/run_validation.sh

# Gazebo V4.4 작업셀 (창 없이: GZ_GUI=0)
ros2 launch pac_bringup hdr50_workcell_v4_4_pick.launch.py

# 재성: 데이터 생성기 / PyBullet 실시간 시뮬레이터 (브라우저 http://127.0.0.1:4173)
tools/ahead_dataset_generator/run_sample.sh
python3 scripts/run_ahead_simulator.py

# ROS 배치 계획 서비스 (/pac/plan_placement)
ros2 launch pac_planning placement_planner.launch.py \
  candidate_config:=$PWD/config/taehyeon/candidates.yaml planner_config:=$PWD/config/default.yaml use_sim_time:=false

# ROS 런타임 노드 (1~8단계 판단, repo:= 기본값 $PAC_REPO)
ros2 launch pac_runtime runtime.launch.py order_file:=$PWD/config/taehyeon/example_order.json
```

## V4.5: 계량 → 팀 5·6단계 → 로봇 (통합 검증)

| 스크립트 | 역할 |
|---|---|
| `scripts/run_mission_v45.sh` | Gazebo 1박스: V4.3 계량 → 플래너 → HDR50-22 IK 검증·적재 → `ExecutionResult` JSON |
| `scripts/plan_placement_v45.py` | 계량값으로 `SystemState`를 만들고 `mission_bridge_v45.plan_ranked` = 5단계 `plan_request`(서비스와 같은 함수) + 6단계 `pac_robot_check` |
| `scripts/verify_stack_bullet_v45.py` | 여러 박스: 생성기 도착 순서 → 5·6단계 → PyBullet → `StateManager` 확정, 반복 |
| `scripts/mission_bridge_v45.py`, `pick_place_plan_v45.py`, `run_mission_v45.py` | 좌표 변환(팔레트 frame은 `config/workcell.yaml`), yaw 회전 경로, ROS 실행 노드 |
| `scripts/ahead_planner_bridge_v44.py` | V4.4 다중 박스 사이클의 plan(5·6단계)·commit(8단계 확정 규칙) |

```bash
# Gazebo (터미널 1): ros2 launch pac_bringup hdr50_workcell_v4_4_pick.launch.py  → ▶ (GZ_GUI=0이면 바로 실행)
bash scripts/run_mission_v45.sh
# PyBullet 여러 박스
python3 scripts/verify_stack_bullet_v45.py --scenario S0001 --skip-unplaceable
```

## 2026-10-09 정리 (공통 기준 v0.3.1)

| 항목 | 이전 | 지금 |
|---|---|---|
| 4단계 | Rule + MaskablePPO(NumPy·sb3) | Rule / Look-ahead, PPO 코드·모델·`rl` 의존성 삭제 |
| 5-①② | `pac_candidates` + 오프라인 `reference_backend` | `pac_candidates` 하나 |
| 6단계 | `pac_robot_check` + `pac_robot` 어댑터 + V4.x 자체 IK + 이전 월드용 `robot_check_gazebo.yaml` | `pac_robot_check` 하나, 설정은 팀 작업셀(V4.2/V4.4)과 일치 검사 |
| 7단계 (Gazebo) | `pac_execution`(1.2 × 1.0 m 강제) + 순간이동 `gazebo_driver` + V4.x | V4.4/V4.5 흡착 실행 하나 |
| 8단계 | `pac_runtime.StateManager`(측정값 저장) + `pac_common.StateManager` + V4.4 JSON | `pac_common.StateManager` 하나, 팀 확정 규칙 적용 |
| 기하 함수 | 3벌 | `pac_common.frames` |
| 무거운-위-가벼운 | 철회됐지만 코드 기본값은 켜짐 | 기본값 꺼짐 (허용 하중 누적 검사) |
| 팔레트 frame | 브리지마다 다름 | `config/workcell.yaml` `frame_yaw_rad` (로봇에서 먼 모서리 원점) |
| Gazebo 작업셀 | V2(HDP160)~V4.4 | V4.2 배치 + V4.4 흡착 |
| 설정 | `robot.yaml`·`eoat.yaml`·`camera.yaml` 등 미사용 | 삭제 |
| 환경 | 수동 | `docker/`, `scripts/setup_ubuntu22.sh`, `scripts/env.sh` |

## 이번 통합에서 바꾼 것 (공통 기준 v0.3)

| 항목 | 이전 | 지금 |
|---|---|---|
| 저장소 | 브랜치 4개 + 별도 저장소, 서로 `.deps`/별도 checkout으로 참조 | 모노레포 하나, 같은 이름 패키지 1벌 |
| `pac_common` | 동한본·작업셀본 2벌 (내용 다름) | 동한본을 기준으로 StateManager·config·frames 추가 |
| 의존성 | requirements 3개 + pyproject | 루트 `pyproject.toml` 하나 |
| 팔레트 | 1.2×1.0 / 1.1×1.1, 높이 1.35·1.5·1.6 m 혼재 | `config/default.yaml` 단일 원본 (1.1×1.1, 데크 위 1.5 m) + 일치 검사 테스트 |
| `PalletState.size.z` | 생성기는 목재 두께 0.15 | 데크 위 적재 높이 (생성기 1.3.0) |
| 상태 확정 | 측정 pose 그대로 → 플래너가 거부 | 오차 xy 5 mm·z 3 mm·yaw 1° 이내면 계획 pose 확정 |
| 목표 로봇 | HDP160-31 (HDR50-22는 대용) | HDR50-22 |
| 로봇 명령 | 후보 모서리를 그대로 전달 | 박스 중심으로 변환 (`pac_common.frames`) |
| 0 N 상부 하중 | 시뮬레이터가 거부 | 동한 패치 적용 (적재 불가 박스 표현) |
| ROS 서비스 | 빌드·호출 미검증 | Humble에서 빌드·호출 검증, 종료 오류 수정 |

변경 이유와 검증은 각 커밋 메시지에 있습니다 (`git log`).

## 흐름별 코드 안내

박스 1개가 도착해서 팔레트에 놓이기까지의 Runtime 흐름(1 인식 → 2 State Validator → 3 Supervisor → 4 High-level 행동 → 5 Low-level Placement Planner → 6 Robot 실행 가능성 → 7 실행·사후 검증 → 8 상태 갱신) 기준으로 각 단계의 코드·담당·테스트·문서 위치를 [docs/flow/](docs/flow/README.md)에 정리했습니다. 단계 사이 중복 구현과 미구현 항목은 [docs/flow/KNOWN_ISSUES.md](docs/flow/KNOWN_ISSUES.md)에 있습니다.

## 남은 일 / 팀 확인 필요

1. **공통 기준 v0.3.1 팀 확인** (기준서 26장), 특히 [결정사항 충돌 검토](docs/flow/DECISION_REVIEW.md) 3장: 적재 1.5 m vs V4.4 주석의 1.6 m, 팔레트 frame 원점(먼 모서리), 내려놓기 간격 20 mm vs 4 mm, EOAT 미선정
2. **State Manager 운영 담당** 확정 (구현은 `pac_common.StateManager` 하나)
3. Gazebo 사이클의 4단계 결정이 `pac_highlevel`과 다름 (버퍼 2칸, 자체 교체 규칙) 등 구현 간 충돌 15건 → [IMPLEMENTATION_CONFLICTS](docs/flow/IMPLEMENTATION_CONFLICTS.md)
4. 큰 박스가 늦게 오면 지지면 부족(`LOW_SUPPORT`)으로 놓을 곳이 없음 → High-level 버퍼/NG와 평탄 적재 점수 조정
5. ALGORITHM_V3 결정 중 미구현: 0.3 g 충격 내성, 닫기 vs 재적재 비용 비교, 버퍼 비상칸 예약
6. `main` 보호 규칙 설정 (기준서 23장: 직접 push 금지, PR + 1명 확인)

## 이력

| 원본 | 커밋 | 위치 |
|---|---|---|
| shared `feature/donghan-placement-planner` | 7860043 | `ros2_ws/src/pac_planning*`, `docs/donghan`, `tests/donghan` |
| shared `claude/pensive-pasteur-dwbu3g` (태현) | e75e274 | `pac_candidates`, `pac_highlevel`, `tools/{virtual_data,highlevel}` |
| shared `feature/jaesung-dataset-generator` | cc4c075 | `tools/ahead_dataset_generator` |
| shared `feature/jaesung-physics-simulator` | 7f4ae37 | `pac_simulation/ahead_sim`, `viewer/` |
| `pac2026-ahead` `fix/hyundai-submodule-init` + 미커밋 V3~V4.5 | 0c8fb80 | 작업셀·로봇·`docs/jaesung`·`tests/workcell` |

**2026-10-09 갱신 (각자 최신본으로 교체)**

| 원본 | 커밋 | 위치 / 비고 |
|---|---|---|
| `yang8988/pac-mission1-taehyeon` main (태현) | 74cbc38 | e75e274 이후 19커밋 이력째 병합: `pac_runtime`, `pac_robot_check`, `tools/{runtime,tuning}` |
| `donghan7298-code/pac2026-ahead-donghan` main (동한 v3) | c0dd992 | 위와 같은 경로 규칙으로 병합: `pac_execution`, `pac_gazebo_grasp`, 학습·평가 파이프라인, 시연 설정. `archive/`(24 MB 기록)는 원본 저장소에만 있음 |
| `pac2026_hdr50_proxy_scaffold` 미커밋 V4.4 (재성) | fa3115a + 작업본 | 카메라 인식, AHEAD 연결, 다중 박스·생성기 사이클, 팔레트 교체(시험 중). `*_v45`는 이미 통합된 최신본 유지 |
| `ahead-dataset-generator` main (재성) | 401f097 | 생성기 v1.4.0 (중복 제거, 무게 분포) |
| 감사·알고리즘 검토 (재성) | — | `docs/jaesung/audit_20261009`, `tools/prototypes` (실험용, 운영 경로 아님) |

`git log --follow <파일>`로 원래 위치의 이력까지 볼 수 있습니다.
