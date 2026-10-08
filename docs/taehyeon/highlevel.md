# 4. High-level 행동 선택 (태현, 2026-10-08 추가)

흐름도 4번 박스를 구현했습니다. 결정 사항(2026-10-08): **앞의 3가지 행동은 PPO로 학습**합니다.

| 행동 | 결정 방법 | 언제 가능한가 (Mask) |
|---|---|---|
| `PLACE_CURRENT` | **MaskablePPO** (1차는 Rule) | 현재 박스에 5-①/5-② 유효 후보가 있을 때 |
| `BUFFER_CURRENT` | **MaskablePPO** (1차는 Rule) | 현재 박스가 있고 빈 버퍼 칸이 있을 때 |
| `RETRIEVE_BUFFER(i)` | **MaskablePPO** (1차는 Rule) | i번 칸에 박스가 있고 그 박스에 유효 후보가 있을 때 |
| `PALLET_CLOSE` | Rule 유지 | 학습 행동이 모두 불가능하고 재적재로도 해결되지 않을 때 |
| `PARTIAL_REPACK` | Rule 유지 | 학습 행동이 모두 불가능할 때, 위가 비어 있는 박스를 옮겨 현재 박스 자리가 생기면 |

- 불가능한 행동은 확률 0으로 막습니다(invalid action masking). 마스크는 5-①/5-②로 계산하므로 **Hard Mask를 통과하지 못한 위치에는 절대 놓지 않습니다.**
- 빈 팔레트에도 놓을 수 없는 박스(규격 초과 등)는 NG(2단계 Inspection 흐름)로 보내고 개수만 기록합니다. 행동으로 고르지 않습니다.
- 5-③~⑥(동한 님)은 바꾸지 않았습니다. 4번이 박스를 정하면 5번이 위치를 정하는 구조 그대로입니다.

코드: `ros2_ws/src/pac_highlevel/` (ROS 2 ament_python 패키지, ROS 없이 동작), 설정: `config/taehyeon/highlevel.yaml`, 스크립트: `tools/highlevel/scripts/`.

## 1. 시뮬레이션 세계 (`world.py`)

학습, Rule 정책, 평가가 모두 같은 `PalletizingWorld`를 씁니다(흐름도의 "학습·실전 동일").

- 입력: 재성 님 제너레이터 시나리오의 도착 순서 그대로. 측정 오차·불확실 박스·팔레트 규격 순환·숨겨진 박스 강도는 가상데이터와 같습니다(`tools/virtual_data/virtual_data/highlevel.py`).
  검사(2단계)에서 파손이 검출된 박스는 **도착하는 순간부터** 허용하중 0 N(위에 쌓지 않음)으로 처리합니다. 미래 박스의 정보는 미리 쓰지 않습니다.
- 버퍼: 팔레트 양쪽 선반, 기본 4칸(설정). 칸마다 이동 시간이 다르고(가까운 칸 4초, 먼 칸 5초), 박스 하나씩만 둡니다.
  버퍼에 넣을 수 있는 것은 컨베이어의 현재 박스뿐이고, 버퍼에서 꺼낸 박스는 바로 팔레트에 놓으므로 박스당 버퍼 방문은 구조적으로 1회입니다.
- 팔레트: 닫히면 새 팔레트로 교체합니다(T_change 60초). 한 주문 흐름에서 팔레트 여러 개를 씁니다.
- 위치 결정(5번 대용): 행동별로 유효 후보 중 DBLF(가장 낮고 안쪽) 위치를 씁니다. 학습 속도를 위한 선택입니다. 실제 운영에서는 같은 자리에 동한 님 planner가 들어갑니다.
- 모든 상태는 SIMULATED입니다. 로봇이나 실제 상태를 건드리지 않습니다.

### 보상 (단위: 팔레트 부피 비율, 동한 님 Future Value와 같은 단위)

| 항목 | 값 |
|---|---|
| 박스 적재 | + 박스 부피 / 팔레트 부피 |
| 팔레트 닫기 | − (1 − 채움률). 합하면 "사용한 팔레트 수 − 실은 부피"이므로 **팔레트 수를 줄이는 것**과 같습니다 |
| 로봇 시간 | − 0.0005 / 초 (적재 8초, 버퍼 보관 4~5초, 버퍼에서 꺼내 적재 8초 + 이동, 팔레트 교체 60초, 재적재 박스당 12초) |
| 버퍼 점유 | − 0.002 / 칸 / 결정 |
| NG | − 0.05 / 박스 |

시간 값은 HDR50-22 사이클을 가정한 값이며, 실측이 나오면 `highlevel.yaml`에서 바꿉니다.

## 2. 관측과 선택지별 Future Value (`features.py`, `value.py`)

고정 길이 벡터(버퍼 4칸 기준 77차원)입니다. 이름 목록이 정책 파일에 함께 저장되고, 배치할 때 다르면 로드를 거부합니다.

- 팔레트 상태 8개, 미도착 재고(EXPECTED_UNSEEN) 4개
- 현재 박스: 치수·무게·부피 + **선택지 특징 7개**
- 버퍼 칸마다: 점유·대기 시간·이동 시간·무게·부피·불확실 여부 + **선택지 특징 7개**

선택지 특징 = 가능 여부, 놓일 높이, 놓은 뒤 윗면 높이, 지지율, 유효 후보 수, 놓은 뒤 heightmap 평탄도, **Future Value**.

Future Value 공급자는 설정으로 고릅니다.
- `proxy`(기본, 학습에 사용): 놓은 뒤 heightmap 평탄도. 빠르고 팀 의존성이 없습니다.
- `donghan`: 동한 님 5-④ 값 헤드(`plan(mode="ranking")`, rollout 없음)의 미래 추가 부피 예측. 학습된 모델 파일(`features.value_model_path`)이 필수이며 한 번만 읽습니다. 모델 없이 쓰면 미래값이 모두 0이 되므로 로드 단계에서 거부합니다.

정책 파일에 공급자 이름이 기록되고, **학습 때와 다른 공급자로 배치하면 로드가 실패**합니다. 흐름도의 "학습·실전 동일" 조건을 코드로 보장한 것입니다.

## 3. MaskablePPO (`ppo.py`, `trainer.py`)

sb3-contrib `MaskablePPO`와 같은 알고리즘을 NumPy로 구현했습니다.
- PyTorch를 이 환경에서 설치할 수 없었습니다(download.pytorch.org 차단, PyPI 판은 CUDA 포함 수 GB). 정책이 작아(입력 77, 행동 6) NumPy로 충분합니다.
- 구성: actor·critic 분리 tanh MLP(64-64), 마스크된 categorical(불가능 행동 logit = −1e9, 엔트로피도 마스크 기준), GAE(λ=0.95), clipped surrogate(0.2), 값 MSE, 엔트로피 보너스, gradient norm clipping, Adam, 관측 정규화.
- 정책 gradient를 유한차분으로 검증하는 테스트가 있습니다(상대 오차 < 1e-4).
- 4개 프로세스가 병렬로 경험을 모읍니다. 각 프로세스는 행동할 때 쓴 정규화 관측을 그대로 돌려주므로, 업데이트 시작 시 확률비가 정확히 1입니다.
- `gym_env.HighLevelGymEnv`는 Gymnasium 인터페이스와 `action_masks()`를 제공합니다. PyTorch가 있는 PC에서는 `sb3_contrib.MaskablePPO("MlpPolicy", env)`로 바로 바꿔 학습할 수 있습니다.

## 3-1. PyTorch(sb3-contrib) 사용 방법

PyTorch가 있으면 sb3-contrib의 `MaskablePPO`로 학습합니다(`pac_highlevel/sb3.py`, `tools/highlevel/scripts/train_highlevel_sb3.py`).
세계·관측·마스크·Rule 모방 warm start는 NumPy 판과 똑같고, 학습기만 다릅니다. 정책 파일은 `*.zip` + `*.contract.json`(특징 목록, Future Value 공급자 확인)입니다.

### 설치 (내 PC)

```bash
# 1) 가상환경 (Python 3.10 권장, 팀 환경과 동일)
python3.10 -m venv .venv && source .venv/bin/activate        # Windows: .venv\Scripts\activate

# 2) PyTorch — 둘 중 하나
pip install torch --index-url https://download.pytorch.org/whl/cpu   # CPU 전용, 약 200 MB (이 정책은 작아서 CPU로 충분)
pip install torch                                                     # 기본판 (Linux는 CUDA 포함, 수 GB)

# 3) 강화학습 라이브러리
pip install sb3-contrib gymnasium "numpy>=1.23,<3" "PyYAML>=6,<7" pytest

# 4) 확인
python -c "import torch, sb3_contrib; print(torch.__version__, sb3_contrib.__version__)"
```

GPU가 있어도 이 정책(입력 77, 행동 6, 64-64 MLP)은 CPU가 더 빠릅니다. 시간은 대부분 5-①/5-② 시뮬레이션에서 씁니다.

### 설치 (Claude Code 클라우드 환경)

`pypi.org`는 허용되어 있어 `pip install torch sb3-contrib gymnasium`이 됩니다(CUDA 포함판이라 몇 분 걸림).
CPU 전용판을 쓰려면 환경 설정 → Network access → Allowed domains에 `download.pytorch.org`를 추가해야 합니다.

### 학습과 평가

```bash
scripts/taehyeon/fetch_team_deps.sh
python tools/highlevel/scripts/train_highlevel_sb3.py --run-generator 10 --steps 100000 \
    --imitation-episodes 120 --output ros2_ws/src/pac_highlevel/models/highlevel_sb3.zip
python tools/highlevel/scripts/evaluate_highlevel.py --run-generator 10 --split test \
    --policies no_buffer greedy rule ppo sb3 \
    --policy-file ros2_ws/src/pac_highlevel/models/highlevel_ppo.json \
    --sb3-file ros2_ws/src/pac_highlevel/models/highlevel_sb3.zip
```

PyTorch가 없으면 `sb3` 관련 테스트는 건너뛰고 NumPy 판(`train_highlevel_ppo.py`)이 그대로 동작합니다.

## 4. Rule 정책 (1차)

- `RulePolicy`: 오래 기다린 버퍼 박스 우선 → 현재 박스가 잘 맞으면(지지율 ≥ 0.95) 적재(단, 버퍼 박스가 2 cm 이상 낮게 들어가면 그것부터) → 아니면 맞는 버퍼 박스를 꺼냄 → 빈 칸이 있으면 현재 박스 보관 → 그래도 안 되면 현재 박스 적재
- `GreedyPolicy`: 놓을 수 있으면 바로 적재. 버퍼 0칸으로 돌리면 "버퍼 없음" 기준선입니다.
- `PARTIAL_REPACK`: 위에 아무것도 없는 박스만 대상으로, 이동 횟수가 적은 순서로 탐색합니다(A*, 휴리스틱 0, 최대 2회 이동·24노드). 이동할 때마다, 그리고 마지막 적재도 5-②로 다시 검증합니다.
  이동은 footprint가 바뀌어야 인정합니다(제자리 90° 회전은 이동, 같은 footprint는 이동 아님). 같은 박스에 대한 재적재 시도는 2회로 제한합니다.
  흐름도의 MCTS 변형과 "대형 SKU Blocking 위험" 발동 조건은 구현하지 않았습니다(현재 발동 조건은 "유효 후보 0개").

## 5. 결과 (2026-10-08, 무거운-위-가벼운 `share` 기준)

데이터: 재성 님 제너레이터 `sample` 모드, 패밀리당 10개 × 80박스 = 60 시나리오(train 42 / val 9 / test 9). 버퍼 4칸.
학습(NumPy): Rule 정책 120 에피소드 모방(정확도 90 %) → MaskablePPO 60k 단계(lr 1e-4, 엔트로피 0.003, 4 프로세스).
평가: 같은 박스 흐름·노이즈·팔레트 규격으로 정책끼리 짝지어 비교. 결정적(deterministic) 정책.

**test (학습에 쓰지 않은 9 시나리오 × 3회 = 27 에피소드, `reports/highlevel_eval_test.json`)**

| 정책 | 사용 팔레트(소수) | 팔레트 수 | 채움률 | 로봇 시간 | NG | 안전 이슈 |
|---|---|---|---|---|---|---|
| 버퍼 없음 | 5.20 | 6.04 | 20.8 % | 974 s | 0 | 0 |
| Greedy + 버퍼 | 4.55 | 5.37 | 22.9 % | 994 s | 0 | 0 |
| Rule (1차) | 3.91 | 4.74 | 26.2 % | 1166 s | 0 | 0 |
| **MaskablePPO (NumPy, 기본 정책)** | **3.86** | **4.70** | **26.5 %** | 1146 s | 0 | 0 |
| MaskablePPO (sb3-contrib, PyTorch) | 3.94 | 4.78 | 26.2 % | 1243 s | 0 | 0 |

- PPO − Rule: 평균 −0.05 팔레트, 27개 중 15개 개선 / 4개 같음 / 8개 악화. 로봇 시간도 20초 짧습니다.
- 버퍼의 효과가 가장 큽니다: 버퍼 없음 대비 Rule/PPO는 팔레트 약 1.3개(25 %) 절약.
- `per_box` 기준 결과(PPO 4.10, Rule 4.19, 버퍼 없음 5.74)는 `reports/per_box_2026-10-08/`에 보관했습니다. `share`로 바꾸면서 모든 정책이 약 0.3~0.5 팔레트 줄었습니다.
- 처음부터 PPO만 학습한 경우(모방 없이 30k 단계)는 개선이 없었습니다(`reports/highlevel_train_log_scratch.jsonl`). 그래서 Rule 모방 → PPO 미세조정 순서로 학습합니다.
- 모든 정책에서 Hard Mask 위반·안전 이슈 0건입니다(마스크가 구조적으로 보장).
- sb3-contrib 판(Rule 모방 120 에피소드 정확도 95 % → 100k 단계, 약 76분)은 Rule과 거의 같았습니다(Rule보다 평균 0.03 팔레트 많음, 12 개선 / 4 같음 / 11 악화).
  같은 데이터·모방 시작점이므로 차이는 학습기 설정에서 옵니다(sb3는 관측 정규화 없음, 배치·업데이트 방식 차이). 그래서 **기본 정책은 NumPy 판(`models/highlevel_ppo.json`)**으로 두고, sb3 판(`models/highlevel_sb3.zip`)은 PyTorch 환경에서 이어 학습할 수 있도록 함께 둡니다.
  다음 시도: `VecNormalize`로 관측 정규화, 학습 단계 확대(300k 이상), 엔트로피 계수 조정.

### 참고: 무거운-위-가벼운 규칙과 채움률

채움률이 20~26 %로 낮은 주된 원인은 무거운-위-가벼운 규칙입니다(VALIDATION 10장). 무작위 무게 순서로 도착하면 위로 갈수록 가벼워야 해서 높이 쌓기 어렵습니다.
`per_box`에서 `share`로 바꿔 팔레트를 약 20 % 줄였고, 동한 님 planner로 위치를 정해도 같은 경향이었습니다.

### 남은 일
- 5번 자리에 DBLF 대신 동한 님 planner를 넣은 평가(느림, 1 결정당 약 0.5 s)
- `donghan` Future Value 공급자로 학습한 정책 (현재 정책은 `proxy`로 학습, 섞어 쓰면 로드 거부)
- 흐름도의 Repack MCTS 변형, "대형 SKU Blocking" 발동 조건

## 6. 실제 상태 입구: `HighLevelDecider.decide` (`runtime.py`)

State Manager / ROS 2 노드가 매 결정마다 부르는 함수입니다. 실제 `SystemState`를 받아 **결정 하나만** 돌려주고, 아무것도 실행하거나 바꾸지 않습니다.
안에서는 학습 때와 같은 세계 로직·마스크·관측·규칙을 실제 상태의 복사본에 적용합니다("학습·실전 동일").

```python
from pac_highlevel import HighLevelDecider, load_policy

decider = HighLevelDecider(
    context,                                    # 이번 사이클의 PlanningContext (catalog, 하중 override, 불확실 박스)
    candidate_config,                           # 5-①/5-② 설정 (load_candidate_config)
    highlevel_config,                           # load_highlevel_config("config/taehyeon/highlevel.yaml")
    policy=load_policy("numpy", "ros2_ws/src/pac_highlevel/models/highlevel_ppo.json", config=highlevel_config),
)
decision = decider.decide(
    state,                                      # SystemState (pallet = 현재 팔레트, tracked_boxes에 현재 박스와 BUFFERED 박스)
    current_box_id="B0123",                     # 대기 중인 박스가 하나뿐이면 생략 가능
    buffer_slots={0: "B0101", 2: "B0117"},      # 버퍼 칸 → 박스 ID (생략하면 ID 순서로 0번부터 배정)
    buffer_age={"B0101": 5},                    # 선택: 칸에 들어간 뒤 지난 결정 수
    repack_attempts=0,                          # 같은 박스에 대해 이미 시도한 재적재 횟수
)
```

### 입력 조건
- `state.pallet.size.z` = 최대 적재 높이(공통 계약). 현재 박스는 `MEASURED / ON_CONVEYOR / READY_FOR_PICK`, 버퍼 박스는 `BUFFERED` 상태여야 합니다.
- `buffer_slots`는 BUFFERED 박스를 정확히 한 번씩 나열해야 합니다. 칸 번호는 `highlevel.yaml`의 `buffer.slots` 범위 안이어야 하며, 칸 번호가 이동 시간(가까운 칸이 쌈)을 결정합니다.
- 정책 파일은 특징 목록과 계약(버퍼 칸 수, Future Value 공급자, 5-① 설정 이름)이 맞아야 로드됩니다.

### 출력 `HighLevelDecision`

| 필드 | 의미 |
|---|---|
| `action` | `PLACE_CURRENT` / `BUFFER_CURRENT` / `RETRIEVE_BUFFER(i)` / `PALLET_CLOSE` / `PARTIAL_REPACK` / `REJECT_NG`, 할 일이 없으면 `None` |
| `box` | 이 행동의 대상 박스. PLACE/RETRIEVE이면 **5단계에 넘길 박스**(RETRIEVE는 BUFFERED 상태 그대로) |
| `requires_low_level` | True이면 `planner.plan(decision.box, state)`로 5단계가 최종 위치를 정합니다 |
| `slot` | BUFFER_CURRENT의 넣을 칸 / RETRIEVE_BUFFER의 꺼낼 칸 |
| `candidate` | 행동이 가능하다고 판단한 근거인 5-①/5-② 유효 자세 (참고용, 최종 위치는 5단계가 정함) |
| `repack_moves` | PARTIAL_REPACK: 순서대로 실행할 `(box_id, Pose3D)` 목록. 실행 후 새 상태로 다시 `decide` |
| `mask`, `probabilities` | 행동 가능 여부와 정책 확률 (0: PLACE, 1: BUFFER, 2+i: RETRIEVE(i)) |
| `decided_by`, `reason` | `policy:maskable_ppo_numpy` / `rule:close` / `rule:repack` / `rule:ng`, 사유 코드 |
| `state_version` | 입력 snapshot 버전. 실행 전에 상태가 바뀌었으면 결정을 버리고 다시 부릅니다 |

`runtime.as_dict(decision)`은 로그·ROS 메시지 변환용 JSON 형태를 돌려줍니다.

### 결정 순서 (학습 세계와 동일)
1. 현재 박스도 버퍼 박스도 없음 → `None` (`NO_BOX`)
2. 현재 박스가 빈 팔레트에도 안 들어감 → `REJECT_NG` (2단계 NG 흐름)
3. 학습 행동 중 하나라도 가능:
   - 현재 박스도, 버퍼 박스도 놓을 수 없고 채움률이 30 % 이상 → `PALLET_CLOSE` (`FILL_BEFORE_BUFFER`)
   - 그 외 → 정책이 마스크 안에서 선택
4. 아무것도 불가능 → 재적재 계획이 있으면 `PARTIAL_REPACK`, 없으면 `PALLET_CLOSE`. 빈 팔레트인데도 불가능하면 `REJECT_NG`

### State Manager가 할 일 (결정 이후)
- `PLACE_CURRENT` / `RETRIEVE_BUFFER`: 5단계 → 6단계(로봇 검증) → 실행. 결과를 `ExecutionResult`로 반영하고 새 버전 snapshot을 만듭니다.
- `BUFFER_CURRENT`: 박스를 `slot`으로 옮기고 상태를 `BUFFERED`로 바꿉니다.
- `PALLET_CLOSE`: 팔레트 교체(PALLET_CHANGE 흐름) 후 빈 팔레트 snapshot을 만듭니다.
- `PARTIAL_REPACK`: `repack_moves`를 순서대로 실행하고, 다시 `decide`를 부릅니다(`repack_attempts`를 1 늘림).
- `REJECT_NG`: Inspection/NG 영역으로 보냅니다.
