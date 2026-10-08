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

- 입력: 재성 님 제너레이터 시나리오의 도착 순서 그대로. 측정 오차·불확실 박스·팔레트 규격 순환은 가상데이터와 같습니다(`tools/virtual_data/virtual_data/highlevel.py`).
- 버퍼: 팔레트 양쪽 선반, 기본 4칸(설정). 칸마다 이동 시간이 다르고(가까운 칸 4초, 먼 칸 5초), 박스 하나씩만 둡니다.
  버퍼에서 꺼낸 박스는 바로 팔레트에 놓으므로 박스당 버퍼 방문은 1회입니다.
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
- `donghan`: 동한 님 5-④ 값 헤드(`plan(mode="ranking")`, rollout 없음)의 미래 추가 부피 예측.

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
  흐름도의 MCTS 변형과 "대형 SKU Blocking 위험" 발동 조건은 구현하지 않았습니다(현재 발동 조건은 "유효 후보 0개").

## 5. 결과 (2026-10-08)

데이터: 재성 님 제너레이터 `sample` 모드, 패밀리당 10개 × 80박스 = 60 시나리오(train 42 / val 9 / test 9). 버퍼 4칸.
학습: Rule 정책 120 에피소드 모방(정확도 89 %) → MaskablePPO 60k 단계(lr 1e-4, 엔트로피 0.003, 4 프로세스, 약 10분).
평가: 같은 박스 흐름·노이즈·팔레트 규격으로 정책끼리 짝지어 비교. 결정적(deterministic) 정책.

**test (학습에 쓰지 않은 9 시나리오 × 3회 = 27 에피소드, `reports/highlevel_eval_test.json`)**

| 정책 | 사용 팔레트(소수) | 팔레트 수 | 채움률 | 로봇 시간 | NG | 안전 이슈 |
|---|---|---|---|---|---|---|
| 버퍼 없음 | 5.74 | 6.56 | 19.3 % | 1000 s | 0 | 0 |
| Greedy + 버퍼 | 5.08 | 5.93 | 20.9 % | 1040 s | 0 | 0 |
| Rule (1차) | 4.19 | 5.00 | 25.0 % | 1164 s | 0 | 0 |
| **MaskablePPO** | **4.10** | **4.93** | **25.3 %** | 1166 s | 0 | 0 |

- PPO − Rule: 평균 −0.09 팔레트, 27개 중 18개 개선 / 2개 같음 / 7개 악화.
- val(18 에피소드, `reports/highlevel_eval_val.json`)에서는 PPO 4.75, Rule 5.02, Greedy 5.00 (PPO가 11 개선 / 3 같음 / 4 악화).
- 버퍼 자체의 효과가 가장 큽니다: 버퍼 없음 대비 Rule/PPO는 팔레트 약 1.6개(27 %) 절약.
- 처음부터 PPO만 학습한 경우(모방 없이 30k 단계)는 개선이 없었습니다(`reports/highlevel_train_log_scratch.jsonl`). 그래서 Rule 모방 → PPO 미세조정 순서로 학습합니다.
- 모든 정책에서 Hard Mask 위반·안전 이슈 0건입니다(마스크가 구조적으로 보장).

### 참고: 채움률이 낮은 이유

채움률 20~25 %는 확정 규칙 **무거운-위-가벼운 `per_box`** 영향이 큽니다. val 4개 시나리오, Rule 정책 비교(규칙은 바꾸지 않았고 참고용 실험입니다):

| 무거운-위-가벼운 | 사용 팔레트 | 채움률 | 닫을 때 높이 |
|---|---|---|---|
| per_box (확정) | 5.11 | 21.7 % | 0.80 m |
| share (실제 전달 하중 기준) | 4.20 | 26.1 % | 1.00 m |
| 끔 (참고) | 2.80 | 37.8 % | 1.23 m |

무작위 무게 순서로 도착하면 위로 갈수록 가벼워야 해서 높이 쌓기 어렵습니다. 팀 논의 거리입니다.

### 남은 일
- 5번 자리에 DBLF 대신 동한 님 planner를 넣은 평가(느림, 1 결정당 약 0.5 s)
- `donghan` Future Value 공급자로 학습한 정책 (현재 정책은 `proxy`로 학습, 섞어 쓰면 로드 거부)
- 흐름도의 Repack MCTS 변형, "대형 SKU Blocking" 발동 조건
