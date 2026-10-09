# 런타임 1~3 · 7~8단계와 1→8 루프 (`pac_runtime`)

흐름도의 미완성 부분(1 인식, 2 State Validator, 3 Supervisor, 7 실행 → 사후 검증, 8 상태 갱신)과 이 단계들을 4~6단계와 잇는 루프를 구현했습니다.
2026-10-08 태현 님이 흐름도의 미완성 부분을 맡기로 해서 태현 파트로 추가했습니다(팀원 파일은 수정하지 않음).
6단계는 [robot_check.md](robot_check.md)에 있습니다.

실제 센서와 로봇이 없으므로 **결정 로직(코어)과 가상 설비(시뮬레이션)를 나눴습니다.** 실제 셀이나 ROS 2 노드는 코어만 쓰면 됩니다.

```text
           ┌──────────── RuntimeCore (결정, 실제 셀에서도 그대로) ─────────────┐
 관측 ──▶  │ 2 StateValidator ─▶ 8 StateManager ◀─ 7 사후 검증(L0~L4)          │ ◀── 실행 결과
           │        │                  │ snapshot                ▲            │
           │        ▼                  ▼                         │            │
           │  Inspection/NG     3 Supervisor ─▶ 4 HighLevelDecider ─▶ Command ─┼──▶ 로봇
           │                                     (5 후보·마스크·순위 + 6 로봇 검사)  │
           └──────────────────────────────────────────────────────────────────┘
 가상 설비(RuntimeLoop): 1 PerceptionSim, ExecutorSim(실제 크기·놓기 오차), 시계
```

## 1. 단계별 구현

| 단계 | 모듈 | 하는 일 |
|---|---|---|
| 1 인식 | `perception.PerceptionSim` | 무게(저울) → 라벨 → 상단 RGB-D 크기 → 원시 관측. 보조(Base) 뷰는 이상이 있을 때만 |
| 2 State Validator | `state_validator.StateValidator` | 라벨·무게·크기를 SKU 카탈로그와 교차 검증, 원인 분류 |
| 3 Supervisor | `supervisor.Supervisor` | NORMAL / PALLET_CHANGE / REPACKING / HOLD, 컨베이어 정지·재개, MISSING 확정 |
| 4·5·6 | `core.RuntimeCore.next_command` | `HighLevelDecider`(4) + 후보·Hard Mask·순위(5) + 로봇 검사(6, `RobotAwarePlacer`) |
| 7 실행 → 사후 검증 | `core.RuntimeCore.verify`, `executor.ExecutorSim` | 계획 대비 실측 편차와 실제 기하(관통·돌출·지지·높이)로 L0~L4 판정 |
| 8 상태 갱신 | `state_manager.StateManager` | **실측 자세** 기준 팔레트, CoG·총중량, 재고, 버퍼, version |

### 2단계 판정 (흐름도 Inspection / NG 영역)

| 분류 | 조건 | 처리 |
|---|---|---|
| `RECOGNITION_FAIL` | 라벨을 못 읽음 또는 신뢰도 < 0.5 | Base 뷰로 다시 봄(Recover). 그래도 모르면 Inspection |
| `DAMAGED` | 찌그러짐 감지 | 기본: Reject + 관리자 알림(흐름도). 옵션 `place_no_load`: 적재하되 위에 아무것도 안 올림(0 N) |
| `SPEC_MISMATCH` | 크기가 SKU 공칭과 12 mm·4 % 넘게 다름, 또는 무게가 SKU 범위를 8 % 넘게 벗어남 | 재측정 후 **실측값으로 갱신해 진행**, 불확실 박스(δ 확대)로 표시 |
| `OK` | 그 외 | 공칭 크기 + 실측 무게 |

크기는 SKU마다 하나(생성기와 같음)이고 무게는 SKU 범위 안에서 박스마다 다르므로, 무게는 범위 밖일 때만 불일치로 봅니다. x/y가 바뀐 같은 상자는 불일치가 아닙니다.

### 3단계 MISSING

주문 목록(SKU별 수량)을 미리 아는 조건이라, 컨베이어가 `missing_timeout_s`(30 s) 동안 비어 있으면 아직 안 온 박스를 MISSING으로 확정하고 남은 재고에서 뺍니다. 그래야 4·5단계가 오지 않을 박스를 위해 공간을 남기지 않습니다.
라벨을 끝내 못 읽어 Inspection으로 간 박스는 주문 목록과 짝지을 수 없어 MISSING 수에 함께 잡힙니다(관리자가 식별하면 바로잡을 수 있음).

### 7단계 에스컬레이션 (흐름도 "편차·실패 발생 시")

| 단계 | 조건 | 처리 |
|---|---|---|
| L0 | xy 5 mm, z 5 mm 이내 | 완료 |
| L1 | xy 20 mm, z 10 mm 이내 | 실측 자세로 상태 갱신 |
| L2 | 그보다 크지만 안전 | 실측 자세로 갱신, 이전 상태로 만든 계획은 version 불일치로 다시 계산(영향 부분만 재계획) |
| L3 | 파지 실패 | 재시도 → 다른 Grasp(그리퍼 yaw +180°) → 확인 영역(Inspection) |
| L4 | 관통 > 2 mm, 팔레트 밖 돌출 > 2 mm, 지지율 < 0.6, 높이 초과 | HOLD, 작업자가 계획 위치로 바로잡음 (120 s) |

## 2. 사용 방법

### 실제 셀 / 다른 모듈에서 (코어)

```python
from pac_runtime import RuntimeCore, ExecutionReport, load_runtime_config
from pac_runtime.order import load_order

core = RuntimeCore(load_order("order.json", cand_cfg), cand_cfg, hl_cfg, load_runtime_config(...),
                   RobotFeasibility(...), load_policy("lookahead", "config/taehyeon/lookahead.yaml", config=hl_cfg))
verdict = core.on_observation(raw_obs, base_view=camera.base_view)  # 1 → 2 → 8
core.on_preview(next_obs)        # 컨베이어 카메라에 보이는 다음 박스들 (N개 탐색 창)
cmd = core.next_command()        # 3 → 4 → 5 → 6: PLACE_CURRENT / RETRIEVE_BUFFER / BUFFER_CURRENT / PALLET_CLOSE / PARTIAL_REPACK / REJECT_NG / WAIT
level = core.on_result(cmd, ExecutionReport(measured_pose=top_view_pose))   # 7 → 8
core.on_conveyor_idle(idle_s)    # 3: MISSING 확정
```

로봇이 명령을 실행하는 동안 `CoreBridge`가 다음 명령을 별도 스레드에서 미리 계산합니다(`RuntimeCore.forecast`: 계획한 위치에 놓였고 컨베이어에 보이던 다음 박스가 왔다고 가정). 결과가 오면 실제 상태가 예상과 같은지(놓인 위치 L0 범위, 무게 검증 범위, 보이는 박스) 보고, 같으면 고른 자리를 실제 상태로 Hard Mask·6단계 검사만 다시 해서 바로 보냅니다. 다르면 처음부터 다시 계산합니다. `status`의 `planned_ahead` / `replanned`가 사용 횟수입니다.

주문 목록 형식은 `pac_runtime/order.py`, 예시는 `config/taehyeon/example_order.json`(생성기 시나리오 1개)입니다.

### ROS 2 노드

`std_msgs/String`에 JSON을 실어 주고받습니다(메시지 정의를 새로 만들지 않아 팀 합의 전에도 붙일 수 있음).

| 토픽 | 방향 | 내용 |
|---|---|---|
| `/pac/observation` | 입력 | box_id, label_sku, weight_kg, size_m, confidence, visual_damage |
| `/pac/execution_result` | 입력 | state_version, ok, attempts, measured_pose(팔레트 좌표, 최소 모서리), issues |
| `/pac/conveyor_idle` | 입력 | idle_s |
| `/pac/command` | 출력 | action, box_id, slot, 목표 자세(최소 모서리 + 박스 중심), 6단계 관절값·그리퍼 yaw·사이클 시간 |
| `/pac/status` | 출력 | state version, 팔레트, 모드, 집계 |

```bash
colcon build --packages-select pac_common pac_candidates pac_highlevel pac_robot_check pac_runtime
ros2 launch pac_runtime runtime.launch.py repo:=$PWD order_file:=$PWD/config/taehyeon/example_order.json
```

메시지 처리 로직(`ros_node.CoreBridge`)은 ROS 없이 테스트했습니다. **노드 자체(rclpy 부분)는 이 환경에 ROS가 없어 실행해 보지 못했습니다.**

### Gazebo 로봇과 연결 (pac2026-ahead 작업셀)

`gazebo_cell` 노드가 컨베이어와 로봇 쪽을 맡고, `runtime_node`와 토픽으로 주고받습니다.

```text
gazebo_cell ──/pac/observation──▶ runtime_node (1~8단계 결정)
gazebo_cell ◀──/pac/command────── runtime_node
   └─ HDR50-22 관절 궤적 → /joint_trajectory_controller/joint_trajectory
      (home → 집는 곳 위 → 집기 → 위 → 놓을 곳 위 → 놓기 → 위)
   └─ 로봇이 도착하면 Gazebo에 박스 생성(ros_gz_sim create), Gazebo 물리로 안착
gazebo_cell ──/pac/execution_result──▶ runtime_node → 다음 박스
```

- 관절값은 6단계 결과(`q_approach`, `q_place`)와 `pick_path`(컨베이어 (0.0, 1.2) 지점)를 씁니다. 두 노드 모두 `config/taehyeon/robot_check_gazebo.yaml`(로봇이 받침대 0.5 m 위, 월드 (1.35, 0.15); 팔레트 중심 (1.35, −1.0), 상판 0.15 m; 컨베이어 윗면 약 0.9 m)을 써서 Gazebo의 로봇과 좌표가 맞습니다.
- **받침대 배치**: pac2026-ahead의 `hdr50_workcell.launch.py`는 로봇을 월드 원점(바닥)에 두는데, 이 배치는 높이 0.8 m 이상에 거의 닿지 않습니다. 그래서 같은 월드·로봇 모델·컨트롤러를 쓰되 로봇을 받침대 위 (1.35, 0.15, 0.5)에 띄우는 launch 파일 `tools/runtime/launch/hdr50_pedestal_workcell.launch.py`를 따로 두었습니다(pac2026-ahead 파일은 수정하지 않음). 이 배치에서는 1.2 × 1.0 팔레트의 맨 위층(1.35 m)까지 시험한 100곳이 모두 닿습니다.
- 작업셀에 흡착 그리퍼 플러그인이 없어서, 박스는 로봇이 놓는 자세에 도착했을 때 그 자리에 생성됩니다(들고 가는 모습은 없음). 생성된 박스는 Gazebo 물리로 내려앉습니다.
- 버퍼에서 꺼낼 때(RETRIEVE_BUFFER)는 버퍼 선반 위치가 정해지지 않아 컨베이어 쪽 집기 동작으로 대신합니다. 팔레트 마감(PALLET_CLOSE) 때는 놓인 박스를 지웁니다.
- 연결 로직(`GazeboDriverCore`)은 ROS 없이 `runtime_node` 로직과 메시지로 이어 테스트했습니다. **ROS 2/Gazebo에서 실제로 띄운 확인은 아직 못 했습니다.**

```bash
# 터미널 1 (pac2026-ahead 작업공간): 받침대 배치의 작업셀
ros2 launch ~/pac-mission1-shared/tools/runtime/launch/hdr50_pedestal_workcell.launch.py
# 터미널 2 (pac-mission1-shared 작업공간, pac2026-ahead는 source하지 않음)
ros2 launch pac_runtime gazebo_cell.launch.py repo:=$HOME/pac-mission1-shared
```

### 가상 셀에서 여러 시나리오 돌리기

```bash
python tools/runtime/scripts/run_runtime.py --dataset tools/highlevel/output/dataset80 --split test \
    --report docs/taehyeon/reports/runtime_test.json
```

생성기 시나리오에 숨겨진 박스 강도(파손 포함), 규격 불일치(2 %, 한 변 ±15 %), 미입고(2 %)를 넣어 돌립니다(`virtual_data/runtime_cell.py`).

## 3. 결과

test 9 시나리오(학습에 안 쓴 것), 박스 80개씩, 4단계 Rule, 5단계 DBLF 순위, 에피소드당 평균입니다(`reports/runtime_test.json`).
네 조건 모두 **같은 박스·같은 관측**(단계별 난수 분리)으로 짝지어 비교했습니다.

| 조건 | 팔레트 | 실제 채움률 | 시간 | 적재 | 검사 영역 | MISSING | L0 / L2 / L4 |
|---|---|---|---|---|---|---|---|
| **기본 (6단계 켬, 그리퍼 0.34 × 0.26 m)** | **5.44** | 22.5 % | 1168 s | 74.1 | 4.4 | 2.0 | 644 / 23 / 0 |
| 6단계 끔 (모든 유효 후보를 실행 가능으로 간주) | 5.11 | 23.8 % | 1146 s | 74.1 | 4.4 | 2.0 | 646 / 20 / 1 |
| 작은 그리퍼 0.20 × 0.15 m | 5.00 | 24.7 % | 1129 s | 74.1 | 4.4 | 2.0 | 641 / 26 / 0 |
| 놓기 오차 3 mm (5-② 예산 밖) | 5.33 | 22.9 % | 1809 s | 74.1 | 4.4 | 2.0 | 442 / 22 / 48 |
| 5단계 순위를 동한 님 planner로 (`runtime_test_donghan.json`) | 5.56 | 22.0 % | 1104 s | 74.1 | 4.4 | 2.0 | 641 / 25 / 1 |

이상 처리 집계(기본, 9 에피소드 합): 정상 636, 규격 불일치 31(재측정 후 진행), 파손 35(Reject), 라벨 인식 실패 5(Base 뷰로도 실패 → 검사 영역). 파지 실패(L3)는 없었습니다.

**알게 된 것**
- **기본 조건의 모든 적재는 로봇 검사를 통과한 자리에서만 이뤄졌고 L4는 0건**입니다. 6단계를 끄면 L4가 1건 생겼습니다.
- 6단계는 후보를 많이 거부합니다(기본 그리퍼 2065건, 대부분 그리퍼 몸체가 더 높은 이웃 박스에 걸리는 `ROBOT_COLLISION`). 다음 순위 후보로 넘어가 대부분 해결되지만, 팔레트가 에피소드당 0.33개 늘었습니다(5.11 → 5.44).
- 그 비용은 거의 그리퍼 크기 때문입니다. 0.20 × 0.15 m 그리퍼에서는 거부가 339건으로 줄고 팔레트는 5.00개로, 6단계를 끈 경우와 비슷했습니다. **작은 박스가 많은 혼합 적재에는 박스보다 크지 않은 흡착면(또는 구역별 흡착)이 유리합니다.** 9 시나리오라 차이의 유의성은 확인하지 못했습니다.
- **놓기 오차는 5-②의 여유(박스 사이 4 mm, 가장자리 2 mm) 안에 있어야 합니다.** 3 mm로 키우면 적재의 7 %가 팔레트 밖 돌출(L4)이 되고, HOLD 때문에 시간이 55 % 늘었습니다. 실제 로봇의 놓기 정확도를 측정해 5-② `lateral_clearance_m` / `size_tolerance_m`을 맞춰야 합니다.
- 동한 님 planner로 순위를 매겨도 루프가 끝까지 돌고(9 시나리오), 이 조건에서는 DBLF보다 팔레트가 약간 많고 시간은 약간 짧았습니다.
- 동한 님 planner는 겹침·돌출이 조금이라도 있는 상태를 거부합니다. 측정 오차 때문에 생기는 1~4 mm 겹침은 8단계가 저장 전에 정합합니다(`StateManager.reconcile`).

### 물리 재현 (`reports/runtime_physics.json`)

기본 조건으로 만든 팔레트를 재성 님 PyBullet 시뮬레이터에서 **실제 크기·실제 자세로 놓인 순서대로 한 박스씩** 다시 쌓았습니다.
박스마다 0.5 s, 마지막에 2 s를 시뮬레이션하고, 어느 박스든 1 cm 넘게 움직이거나 2° 넘게 기울면 실패로 봅니다(5-② 물리 교차 검증과 같은 기준).

| 대상 | 결과 |
|---|---|
| test 9 시나리오의 팔레트 49개, 박스 667개 | **49/49 안정**, 가장 크게 움직인 박스 3.8 mm |

```bash
python tools/runtime/scripts/physics_replay.py --dataset tools/highlevel/output/dataset80 --split test \
    --report docs/taehyeon/reports/runtime_physics.json
```

### 3D로 보기

**파일 하나로 보기**: `reports/runtime_viewer.html`을 브라우저로 열면 위 49개 팔레트를 고르고, 적재 순서를 한 박스씩 재생할 수 있습니다(색: 무게, 빨간 선: 무게중심, 회색 틀: 높이 한계, PyBullet 안정 여부 표시).
three.js를 CDN에서 불러오므로 인터넷이 필요합니다. 다시 만들려면 `python tools/runtime/scripts/export_viewer.py docs/taehyeon/reports/runtime_physics.json docs/taehyeon/reports/runtime_viewer.html`.

**물리 시뮬레이터로 보기**:

재성 님(또는 pac2026-ahead) 라이브 시뮬레이터를 띄운 뒤 `--live`로 팔레트 하나를 보내면 브라우저에서 한 박스씩 쌓이는 모습을 볼 수 있습니다.
시뮬레이터의 팔레트 크기를 보낼 팔레트와 맞춰야 합니다(시나리오마다 1.2 × 1.0 / 1.1 × 1.1 / 1.2 × 0.8 m).

```bash
python3 scripts/run_ahead_simulator.py --no-browser          # 시뮬레이터 쪽, 브라우저로 http://127.0.0.1:4173
python tools/runtime/scripts/physics_replay.py --dataset tools/highlevel/output/dataset80 --split test \
    --scenarios 1 --live http://127.0.0.1:4173 --live-pallet 0
```

## 4. 한계

- 1단계와 7단계의 설비는 가상입니다. 확률·허용오차는 가정값이며 `config/taehyeon/runtime.yaml`에 모았습니다.
- 버퍼 넣기·꺼내기의 로봇 경로는 6단계 검사 없이 시간만 더합니다(버퍼 선반 위치가 정해지지 않음).
- PARTIAL_REPACK은 옮길 박스마다 6단계를 통과해야 실행합니다. 하나라도 안 되면 재적재를 취소하고 4단계가 다른 행동(대개 마감)을 고릅니다.
- PARTIAL_REPACK 명령은 옮길 박스들만 담습니다. 현재 박스는 이동이 끝난 뒤 다음 명령으로 다시 계획합니다.
- 같은 box_id 관측이 다시 들어오면(메시지 재전송, 재스캔) 무시합니다.
- L3의 "다른 Grasp"는 가상 로봇 안에서만 처리합니다(별도 명령 없음). 실제 셀에서는 실행 모듈이 그리퍼 yaw +180°로 다시 시도한 뒤 결과를 보고하면 됩니다.
- ROS 2 노드는 JSON 문자열 토픽입니다. 팀이 메시지 정의를 정하면 `ros_node.py`의 변환 함수만 바꾸면 됩니다.
