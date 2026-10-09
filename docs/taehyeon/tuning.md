# LLM 튜닝 보조 (`tools/tuning`)

Claude(LLM)가 규칙·후보 생성 파라미터를 제안하고, 같은 박스 흐름으로 짝지어 평가한 결과를 읽으며 다음 제안을 고릅니다. 마지막에 고른 설정을 **학습·탐색에 쓰지 않은 test 시나리오로 한 번** 확인합니다.
같은 평가 횟수의 무작위 탐색을 기준선으로 함께 돌려, LLM이 실제로 도움이 되는지 판단할 수 있게 했습니다.

```text
Claude ──describe_search_space──▶ 파라미터 설명·현재값·기준 성적·남은 예산
Claude ──evaluate_config(params, 가설)──▶ 검증 시나리오 평가(기준과 짝지은 비교)
   ... (가설 → 평가 → 해석 반복, 예산 안에서)
Claude ──finish(params, 근거, 추가 아이디어)──▶ test 시나리오로 최종 확인 → report.md
```

## 무엇을 바꿀 수 있나 (안전 제약은 제외)

| 파라미터 | 범위 | 뜻 |
|---|---|---|
| `rules.good_support` | 0.6~1.0 | 현재 박스의 최선 자리 지지율이 이 값 이상일 때만 바로 놓음 |
| `rules.retrieve_margin_m` | 0~0.15 | 버퍼 박스 자리가 이만큼 낮으면 버퍼 박스를 먼저 놓음 |
| `rules.max_buffer_age` | 2~40 | 이 결정 수보다 오래된 버퍼 박스는 먼저 꺼냄 |
| `close.fill_before_buffer` | 0~0.7 | 놓을 곳이 없을 때 채움률이 이 값 이상이면 버퍼 대신 마감 |
| `repack.enabled`, `repack.max_moves` | on/off, 1~3 | 부분 재적재 허용과 이동 박스 수 |
| `generation.dedup_distance_m` | 0.02~0.15 | 5-① 후보 중복 제거 거리 |
| `generation.balance_anchors` | on/off | 5-① 하중 분산 자리 후보 |

5-② Hard Mask와 6단계 로봇 검사는 탐색 공간에 넣지 않았습니다. LLM은 "안전한 선택지 중 무엇을 고를지"만 바꿀 수 있습니다. 제안값은 범위를 벗어나면 거부되고, 그 사실이 오류로 LLM에 돌아갑니다.

## 목표와 제약
- 목표: 검증 시나리오의 평균 사용 팔레트 수(소수 포함) 최소화
- 제약(코드가 판정): 안전 이슈 0, 로봇 시간 +10 % 이내, NG 증가 없음
- 평가마다 기준 설정과 같은 박스 흐름으로 짝지어 개선/같음/악화, 부호 검정 p, 95 % 구간을 함께 줍니다. 시스템 프롬프트에서 평균만 보지 말라고 지시합니다.

## 실행

```bash
pip install anthropic                     # Python SDK
export ANTHROPIC_API_KEY=...              # 또는 `ant auth login`
python tools/tuning/scripts/llm_tune.py --dataset tools/highlevel/output/dataset80_x40_s8 \
    --optimizer llm --budget 12 --output tools/tuning/output/llm
# 비교용 무작위 탐색 (API 키 불필요)
python tools/tuning/scripts/llm_tune.py --dataset tools/highlevel/output/dataset80_x40_s8 \
    --optimizer random --budget 12 --output tools/tuning/output/random
```

- 모델: `claude-opus-5-5`, effort `high`, 서버 측 거절 대체(`fallbacks: "default"`) 사용.
- 비용(추정): 평가 12회 기준 대화 15~20턴, 약 1~3달러. 평가 1회는 4코어 CPU에서 약 1~2분(검증 36 에피소드).
- 결과: `report.md`(선택 설정, test 결과, LLM 근거·추가 아이디어, 검증 기록), `report.json`, `log.jsonl`(LLM 메시지와 평가 전부).

## 결과

### 무작위 탐색 기준선 (2026-10-09, `reports/tuning_random/`)

검증: 새 시드 240개 시나리오 중 val 36개 × 1회, 평가 12회. test: 같은 데이터의 test 36개 × 3회 = 108 에피소드(탐색에 쓰지 않음).

| | 사용 팔레트 | 적재율 | 로봇 시간 | 안전 이슈 |
|---|---|---|---|---|
| 현재 설정 (test) | 3.83 | 23.4 % | 1117 s | 0 |
| 선택 설정 `close.fill_before_buffer` 0.30 → 0.62, `repack.enabled` true (test) | **3.66** | 24.5 % | 1132 s | 0 |

- test에서 −0.16 팔레트(95 % 구간 −0.25 ~ −0.08), 108개 중 50 개선 / 46 같음 / 12 악화, 부호 검정 p < 0.001. **유의한 개선**입니다.
- 의미: 놓을 곳이 없을 때 채움률 30 %에서 바로 마감하지 말고, 62 %가 될 때까지는 버퍼를 써서 버티는 편이 팔레트를 덜 씁니다.
- 검증 기록에서 하중 분산 자리 후보(`balance_anchors`)를 끈 조합은 모두 나빠졌습니다(4/4). 지금 켜 둔 것이 맞다는 근거입니다.
- **기본값은 아직 바꾸지 않았습니다.** 30 % 마감 규칙은 2026-10-08 태현 님 결정이라, 바꿀지는 결정이 필요합니다.

LLM 결과는 태현 님 키로 `--optimizer llm`을 돌린 뒤 이 표에 같은 형식으로 추가하면 됩니다. 판단 기준: 같은 평가 횟수(12회)에서 무작위 탐색의 test −0.16보다 나은지.

## 한계
- 검증 36 에피소드로는 0.05 팔레트 차이를 구분하기 어렵습니다. 그래서 마지막에 test 108 에피소드로 다시 확인합니다.
- 동한 님 5-⑥ 점수 가중치는 아직 탐색 공간에 넣지 않았습니다. 평가가 느려서(결정당 약 0.7 s) 1회 평가가 수십 분이 걸리기 때문입니다. `donghan_ranker(planner_config=...)`로 넘길 수 있게 되어 있어, 표본을 줄이면 같은 도구로 확장할 수 있습니다.
- 이 클라우드 환경에는 API 키가 없어 LLM 실행은 확인하지 못했습니다. 도구 루프는 가짜 응답으로 테스트했습니다(`tests/taehyeon/test_th_tuning.py`).
