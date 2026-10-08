# 태현 담당 파트: 5-① 후보 생성 · 5-② Hard Mask · 가상데이터 생성 · 4. High-level 행동 선택

> **모노레포 통합 (2026-10-08):** 팀원 패키지가 이미 `ros2_ws/src`에 있어 `fetch_team_deps.sh`는 필요 없습니다. 의존성은 루트 `pyproject.toml`. 팔레트 적재 높이는 데크 위 **1.5 m**로 바뀌었습니다 (`config/default.yaml`). 아래 1.35 m 기준 수치는 변경 전 기록입니다.

흐름도 **5. Low-level Placement Planner** 중 앞의 두 단계와 학습·검증용 가상데이터를 담당합니다.
2026-10-08부터 **4. High-level 행동 선택**(앞 3개 행동 MaskablePPO, CLOSE·REPACK Rule)도 맡았습니다 → [highlevel.md](highlevel.md).
결과물은 동한 님 5-③~⑥(`pac_planning`)이 그대로 호출하는 두 콜백과, 그 콜백을 대량으로 돌려 만든 데이터입니다.

```text
4. High-level (PLACE_CURRENT ...)
        │  선택된 박스 + SystemState snapshot
        ▼
5-① 후보 생성   generate_candidates(box, state) -> list[PlacementCandidate]
        │  EMS(Heightmap) ∪ Extreme Point · 기준점 5개 · yaw 0°/90° · 80 mm 중복 제거
        ▼
5-② Hard Mask   validate_constraints(box, candidate, state) -> ValidationResult
        │  경계·겹침·높이·방향·지지율·LBCP(+CoG δ)·박스 하중·무거운-위-가벼운·팔레트 하중·팔레트 CoG
        │  통과 시 ConstraintEvidence(안전 여유) 첨부, 탈락 시 공통 RejectCode
        ▼
5-③~⑥ (동한) Feature → AI Top-K → Future Rollout → 최종 점수
```

사용 로봇은 **HDR50-22**입니다. 로봇 도달·IK·충돌·가반하중은 6단계 담당이라 이 파트에서는 판정하지 않습니다.

## 빠른 시작

```bash
python3.10 -m venv .venv && . .venv/bin/activate          # 또는 uv venv --python 3.10
pip install "numpy>=1.23,<3" "PyYAML>=6,<7" "pytest>=7,<9"
pip install "pybullet>=3.2.6,<4" "networkx>=2.8,<4"       # 물리 교차 검증을 돌릴 때만
scripts/taehyeon/fetch_team_deps.sh                          # 팀원 코드를 .deps/team 에 읽기 전용으로 추출
python -m pytest -q tests/taehyeon                           # 177 passed (PyTorch 없으면 1 skipped)
scripts/taehyeon/run_validation.sh                           # 모든 검증 리포트 재생성 (약 6~11분, 머신에 따라 다름)
```

`fetch_team_deps.sh`는 팀원 브랜치에서 `pac_common`, `pac_planning`, 재성 님 제너레이터와 시뮬레이터를 꺼내 git 제외 폴더(`.deps/`)에 둡니다.
브랜치가 main에 합쳐진 뒤에는 `ros2_ws/src/pac_common` 같은 원래 위치를 자동으로 우선 사용합니다(`scripts/taehyeon/team_paths.py`).

코드에서 사용하는 방법:

```python
from pac_candidates import CandidateBackend, load_candidate_config

backend = CandidateBackend(context, load_candidate_config("config/taehyeon/candidates.yaml"))
candidates = backend.generate_candidates(box, state)          # 5-①
verdict = backend.validate_constraints(box, candidates[0], state)  # 5-②
cset = backend.candidate_set(box, state)                       # 5-①+②: "54 generated → 31 masked → 23 valid"
print(cset.summary(), dict(cset.reason_counts))
```

## 폴더 구조 (모두 새로 추가한 파일, 팀원 파일 수정 없음)

| 경로 | 내용 |
|---|---|
| `ros2_ws/src/pac_candidates/` | ROS 2 ament_python 패키지 (ROS 없이도 동작) |
| `├ backend.py` | 팀 v0.2 콜백, 상태별 캐시, `candidate_set`, `context_with_ems` |
| `├ candidate_generation.py` | 5-① EMS 기준점·Extreme Point·yaw·중복 제거·출력 순서 |
| `├ hard_mask.py` | 5-② 12개 검사 + `ConstraintEvidence` |
| `├ pallet_model.py` | 스냅샷 기하 모델: 지지 그래프, LBCP, 누적 하중, 압축 heightmap, EMS |
| `├ loads.py` | 하중 분배(lever/area), McKee 상자 압축강도 |
| `├ geometry.py` · `config.py` · `reports.py` | 기하 연산, 검증되는 YAML 설정, 결과 자료형 |
| `ros2_ws/src/pac_highlevel/` | 4번 High-level: 시뮬레이션 세계(버퍼·다중 팔레트), Rule 정책, PARTIAL_REPACK 탐색, NumPy MaskablePPO, Gymnasium 환경 |
| `tools/highlevel/` | 4번 학습·평가 스크립트 (`train_highlevel_ppo.py`, `evaluate_highlevel.py`) |
| `config/taehyeon/highlevel.yaml` | 4번 설정 (버퍼 칸 수, 시간, 보상, PPO) |
| `tools/virtual_data/` | 가상데이터 생성기, 2D 시각화, 오라클·물리·planner·박스 강도 벤치마크 스크립트 |
| `config/taehyeon/candidates.yaml` | 5-①/5-② 설정 (모든 기본값의 근거는 [algorithms.md](algorithms.md)) |
| `config/taehyeon/virtual_data.yaml` | 가상데이터 설정 |
| `tests/taehyeon/` | 177개 테스트 (단위, brute-force 오라클, planner 통합, 가상데이터, 박스 강도, 시각화, 4번 High-level) |
| `scripts/taehyeon/` | 팀 코드 추출, 경로 탐색, 전체 검증 스크립트 |
| `docs/taehyeon/` | 이 문서들 + `reports/` 검증 결과 |

## 문서

1. [interface.md](interface.md): 동한 님·재성 님과의 연결 계약 (입출력, 좌표, evidence, EMS 공급)
2. [algorithms.md](algorithms.md): 5-①/5-② 알고리즘과 기본값 근거
3. [virtual_data.md](virtual_data.md): 가상데이터 생성기 사용법과 출력 스키마
4. [VALIDATION.md](VALIDATION.md): 검증 방법과 결과 (오라클, 물리, planner 연동)
5. [highlevel.md](highlevel.md): 4번 High-level 행동 선택 (MaskablePPO, Rule, 버퍼, 재적재)
6. [team_review_2026-10-08.md](team_review_2026-10-08.md): 저장소 전체 검토 (모든 브랜치, 미션 대응표, main 통합 방법)
7. [PROGRESS.md](PROGRESS.md): 진행 기록 / 재개 방법

## 핵심 결과 (2026-10-08, 자세한 내용은 [VALIDATION.md](VALIDATION.md), [highlevel.md](highlevel.md))

| 항목 | 결과 |
|---|---|
| 5-① 재현율: 20 mm 격자 전수탐색에 놓을 자리가 있으면 유효 후보를 1개 이상 제공 | **97.4 %** (76/78 장면, 놓친 2장면은 자리가 1개뿐인 좁은 경우) |
| 5-② 안전성: Hard Mask 통과 후보를 실제 크기로 PyBullet에 놓았을 때 안정 | **82/82 (100 %)** |
| 5-② 유의미성: 지지/LBCP로 탈락한 후보 중 실제로 무너짐 | 68/110 (62 %) |
| 측정오차 δ 흡수: 실제 크기 기준 상호 관통/팔레트 돌출 | 0건 (720단계) |
| 박스 강도 비공개 대응: 숨겨진 실제 강도 6개 프로필, 팔레트 끝까지 적재 | 6개 프로필 모두 실제 눌림 **0건** (안전계수 1에서도) |
| 동한 님 planner 연동 (1 초 소프트 예산) | 평균 765 ms, 초과 7/23 |
| 4번 High-level (test 27 에피소드, 사용 팔레트) | 버퍼 없음 5.43 → Greedy 4.62 → **Rule 4.09 (권장 기본값)**, MaskablePPO NumPy 4.25 / sb3 4.23 (Rule보다 낫지 않음, 유의하지 않음), 안전 이슈 0 |
| 무거운-위-가벼운 규칙 | `share` 모드 (2026-10-08 결정, `per_box` 대비 팔레트 약 20 % 절약) |
