# [비교용 기준선] 현장 관행형 규칙 적재 알고리즘 (동한)

> **이 알고리즘은 우리 알고리즘이 아닙니다.** 우리 알고리즘(팀 최종: N=5 탐색 + 평평한 층 + 막힌 팔레트 마감, `docs/taehyeon/lookahead.md`)과 대조하기 위한 기준선입니다. 그래서 모든 파일을 `baseline/` 폴더에 두고 이름을 `field_rule`로 붙였습니다.

현장에서 쓰는 단순한 규칙만으로 박스를 처리할 방법(4단계)과 놓을 자리(5-③)를 고르는 적재 알고리즘입니다.
미리보기, 탐색, 학습, 부분 재적재는 쓰지 않습니다. 안전 검사는 팀 코드(5-① 후보 생성, 5-② Hard Mask)를 그대로 쓰며, 규칙은 안전 검사를 통과한 후보 사이의 순서만 정합니다.

```text
ros2_ws/src/pac_planning/pac_planning/baseline/
├── __init__.py          비교용 기준선 모음 (여기 밖의 코드는 이 폴더를 import하지 않음)
└── field_rule.py        현장 관행형 규칙 (FieldRulePolicy, FieldRulePlacer)
config/donghan/baseline/field_rule.yaml        설정 (최상위 키 baseline_field_rule)
scripts/donghan/baseline/run_field_rule.py     실행
tests/donghan/baseline/test_field_rule.py      테스트
docs/donghan/baseline/field_rule.md            이 문서
```

실행 결과 JSON의 `algorithm`과 정책 이름도 `baseline_field_rule`로 기록됩니다.

## 1. 흐름

```text
박스 도착 (지금 박스만 봄, 컨베이어의 다음 박스는 보지 않음)
 └ 4단계: 지금 박스를 놓을 수 있나?
     ├ 예 → 놓기
     ├ 아니오 → 버퍼에 놓을 수 있는 박스가 있나? → 있으면 가장 오래된 것부터 꺼내 놓기 (FIFO)
     ├ 아니오 → 빈 버퍼 칸에 두기
     └ 버퍼도 꽉 참 → 팔레트 마감 → 새 팔레트
 └ 5-① 후보 생성 → 5-② Hard Mask (팀 코드 그대로)
 └ 5-③ 규칙으로 자리 선택: 가장 낮게 → 로봇에서 먼 줄 → 옆면 접촉 많이 → 왼쪽
```

"놓을 수 있다"는 5-② Hard Mask를 통과한 후보가 하나 이상 있다는 뜻입니다.

## 2. 4단계 규칙 (`FieldRulePolicy`)

| 순서 | 조건 | 행동 |
|---|---|---|
| 1 | 지금 박스를 놓을 수 있음 | `PLACE_CURRENT` |
| 2 | 버퍼 박스 중 놓을 수 있는 것이 있음 | `RETRIEVE_BUFFER`: 버퍼에 들어간 순서가 가장 빠른 박스 |
| 3 | 빈 버퍼 칸이 있음 | `BUFFER_CURRENT` |
| 4 | 위가 모두 불가능 | 팔레트 마감 (world·`HighLevelDecider`의 기존 규칙이 처리) |

지금 박스가 없을 때(흐름 끝)는 2번부터 적용합니다. 빈 팔레트에도 들어가지 않는 박스는 기존 world 규칙대로 NG로 보냅니다.

## 3. 5-③ 자리 규칙 (`FieldRulePlacer`)

Hard Mask를 통과한 후보를 아래 순서로 거릅니다. 앞 단계에서 남은 후보끼리만 다음 단계를 비교합니다.

| 순서 | 규칙 | 기준값 | 이유 |
|---|---|---|---|
| 1 | 가장 낮은 자리 | 바닥 높이 차이 3 mm 이내는 같은 높이 (5-② 높이 허용값) | 아래부터 채움 |
| 2 | 로봇에서 가장 먼 줄 | 후보의 먼 쪽 끝 위치 차이 10 mm 이내는 같은 줄 | 로봇 팔이 이미 놓은 박스 위로 넘어가지 않게 안쪽부터 채움 |
| 3 | 옆면 접촉이 가장 많은 자리 | 박스 둘레 중 이웃 박스 옆면이나 팔레트 가장자리와 15 mm 이내로 맞닿은 비율 | 빈틈 없이 붙여 쌓음. 5-②가 이웃 간격을 8 mm 이상 두므로 15 mm로 잡음 |
| 4 | 왼쪽 | 로봇에서 본 왼쪽 끝 | 같은 조건이면 한쪽부터 채움 |
| 5 | 후보 ID | | 결과를 항상 같게 |

- 로봇 위치는 `robot_side`(기본 `-y`, `config/taehyeon/robot_check.yaml`의 `base_from_pallet_center` y = -1.15 기준)로 정합니다. 로봇이 `-y`에 있으면 `+y` 가장자리 줄을 먼저 채우고, 왼쪽은 `-x` 쪽입니다.
- 방향(yaw)은 5-①이 SKU별 허용 방향으로 후보를 만들 때 정해지므로 별도 규칙이 없습니다.
- 바로 아래에 있는 박스는 옆면 접촉으로 세지 않습니다(높이 구간이 겹치는 박스만 셈).

## 4. 마감·재적재 설정 (`field_rule_highlevel_config`)

팀 기본 설정(`config/taehyeon/highlevel.yaml`)에서 두 가지만 바꿉니다.

| 항목 | 팀 기본값 | 규칙 기반 |
|---|---|---|
| 부분 재적재 `repack.enabled` | `true` | `false` (놓인 박스는 다시 옮기지 않음) |
| 마감 `close` | `mode: dead` (막힌 팔레트 판단) | `mode: fill`, `fill_before_buffer: 0` → 4단계 규칙 4번일 때만 마감 |

버퍼 칸 수, 동작 시간, 팔레트 크기 등 셀 조건은 `config/taehyeon/environment.yaml`을 그대로 씁니다.

## 5. 사용법

**오프라인 실행** (팀 생성기 데이터, 에피소드→시나리오 대응은 팀 평가 스크립트와 같음):

```bash
python3 scripts/donghan/baseline/run_field_rule.py --dataset tools/highlevel/output/dataset80_x40_s8 \
    --split test --episodes 3 --output reports/baseline_field_rule_test.json
# 3D 뷰어용 타임라인 (컨베이어 시간 흐름 포함)
python3 scripts/donghan/baseline/run_field_rule.py --dataset tools/highlevel/output/dataset80_x40_s8 \
    --split test --episode-ids 0 --timeline reports/baseline_field_rule_test0_timeline.json
python3 tools/realtime/export_viewer.py reports/baseline_field_rule_test0_timeline.json reports/baseline_field_rule_test0.html
```

출력 JSON에는 에피소드마다 `scenario_id`, 팔레트 크기, 적재·NG·팔레트 수, 마감 팔레트 채움률, 팔레트 환산값, 로봇 시간, 행동 횟수, 안전 문제 수가 들어갑니다. 설정, seed, git 커밋도 함께 기록합니다.

**코드에서 (가상 world)**:

```python
from pac_highlevel import PalletizingWorld, run_policy
from pac_planning.baseline.field_rule import FieldRulePlacer, FieldRulePolicy, field_rule_highlevel_config

hl = field_rule_highlevel_config(hl)
world = PalletizingWorld(arrivals, pallet_size, catalog, cand_config, hl, placer=FieldRulePlacer())
summary = run_policy(world, FieldRulePolicy())
```

**런타임 코어 (1~8단계, 6단계 로봇 검사 포함)**: `rank()`가 후보 전체를 규칙 순서로 돌려주므로, `RobotAwarePlacer`가 위에서부터 6단계 검사를 하고 실패하면 다음 후보로 넘어갑니다.

```python
from pac_planning.baseline.field_rule import (FieldRulePlacer, load_field_rule_config,
                                              field_rule_highlevel_config, field_rule_loaded_policy)

placer = FieldRulePlacer(load_field_rule_config("config/donghan/baseline/field_rule.yaml"))
core = RuntimeCore(cell, cand_config, field_rule_highlevel_config(hl), rt_config, robot,
                   field_rule_loaded_policy(), ranker=placer.rank)
```

자리마다 규칙 값(높이, 먼 쪽 끝, 접촉 비율, 왼쪽 끝)을 보려면 `placer.explain(valid, box, state, backend)`를 씁니다.

## 6. 테스트

```bash
python3 -m pytest -q tests/donghan/baseline/test_field_rule.py
```

자리 규칙(높이 → 줄 → 접촉 → 왼쪽, 높이 허용값, 로봇 위치, 아래 박스 제외), 4단계 우선순위(FIFO, 막힌 행동 미선택), 설정 검증, 작은 팔레트 전체 실행(버퍼 → 마감 → 새 팔레트에서 버퍼 비우기), `RobotAwarePlacer`·`HighLevelDecider` 연결을 확인합니다. 2026-10-10 Windows Python 3.11에서 15 passed.

동작 확인(2026-10-10, main 3c82bfa 기준, `dataset80_x40_s8` test 에피소드 0~2): 세 에피소드 모두 80/80 적재, NG 0, 안전 문제 0.

## 7. 한계와 주의

- ROS `runtime_node`의 `ranker` 파라미터는 `dblf | donghan`만 받습니다. ROS 노드에서 쓰려면 `pac_runtime/ros_node.py`의 `make_runtime_ranker`에 항목을 더해야 합니다(태현 님 파일이라 이번에는 바꾸지 않음).
- 오프라인 실행(`run_field_rule.py`)에는 6단계 로봇 검사가 없습니다(팀 오프라인 평가와 같음). 6단계는 런타임 코어에서만 적용됩니다.
- 높이가 다른 박스가 섞이면 바닥부터 채우는 규칙 때문에 윗면이 들쭉날쭉해지고, 다음 층 후보가 지지율 부족(`LOW_SUPPORT`)으로 많이 탈락할 수 있습니다. 예: test 에피소드 2의 첫 팔레트 마감 시점에 지금 박스의 후보 89개 중 88개가 `LOW_SUPPORT`였고, 버퍼 4칸이 차 있어 마감했습니다. 규칙 설계상 그대로 둔 동작입니다.
- `pac_planning/package.xml`에는 `pac_highlevel` 의존성이 없습니다. `FieldRulePolicy`, `field_rule_loaded_policy`만 실행할 때 `pac_highlevel`을 불러옵니다(자리 규칙 `FieldRulePlacer`는 `pac_candidates`만 필요).
