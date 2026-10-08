# 가상데이터 생성기 (`tools/virtual_data`)

재성 님 AHEAD 데이터셋 제너레이터의 시나리오를 실제 도착 순서대로 재생하면서 매 도착마다 5-①/5-②를 실행합니다.
결과로 **동한 님 planner가 바로 읽는 장면 파일**과 **후보·마스크 라벨**을 만듭니다.

```text
재성 generator ─(test_data, ground_truth, catalog.csv, splits)─▶ scenario_source
   └ 도착 박스(실제값) ─▶ observation (측정 노이즈 δ, 불확실 박스 표시)
        └ SystemState + PlanningContext ─▶ CandidateBackend (5-① + 5-②)
             ├ scenes/<split>/Sxxxx-Tyyy.json     ← 동한 planner 입력 (pac-common-v0.2+planning-v1)
             ├ candidate_sets/Sxxxx.jsonl          ← 후보별 유효/사유/evidence 라벨
             └ policy(dblf | random | mixed | planner)로 한 후보를 골라 적재 → 다음 도착
```

## 실행

```bash
scripts/taehyeon/fetch_team_deps.sh     # 최초 1회

# 재성 님 제너레이터를 먼저 실행 (sample: 패밀리당 N개, benchmark: 600개)
python tools/virtual_data/scripts/generate_virtual_data.py \
    --run-generator sample --sample-per-family 5 --output tools/virtual_data/output/run1

# 이미 있는 제너레이터 출력 사용
python tools/virtual_data/scripts/generate_virtual_data.py \
    --dataset path/to/generated --output tools/virtual_data/output/run2 --policy planner

python tools/virtual_data/scripts/validate_virtual_data.py tools/virtual_data/output/run1
```

`tools/virtual_data/output/`은 git에서 제외됩니다. 커밋되는 것은 `docs/taehyeon/reports/`의 요약뿐입니다.
속도: 박스 한 개당 약 12 ms (720단계 ≈ 11초, 라벨 포함). `--policy planner`는 동한 님 planner를 매 단계 실행해서 훨씬 느립니다.

## 설정 (`config/taehyeon/virtual_data.yaml`)

| 항목 | 기본값 | 설명 |
|---|---|---|
| `pallet.height_limit_includes_pallet` | true | 1.5 m − 팔레트 0.15 m = 적재 높이 1.35 m |
| `pallet.default_max_load_kg` | 1000 | 제너레이터의 `max_load_kg`가 null일 때 사용 |
| `catalog.nominal_weight` | midpoint | 미입고 박스의 `SkuSpec.weight_kg` (무게 범위의 중앙) |
| `observation.dimension_noise_std_m` / `clip` | 1 mm / 3 mm | 측정 치수 노이즈 |
| `observation.weight_noise_std_ratio` | 1 % | 무게 측정 노이즈 |
| `observation.uncertain_probability` | 5 % | 불확실 박스 비율 (치수 σ 4 mm, confidence 0.6, `uncertain_box_ids`에 등록) |
| `episode.policy` | mixed | 70 %는 첫 유효 후보, 30 %는 상위 5개 중 무작위 → 다양한 중간 상태 |
| `episode.scene_every` | 1 | 장면 저장 간격 |
| `strength.profiles` | strong·nominal·weak·humid·mixed·extreme 각 1 | 박스 **실제** 강도 프로필 (planner에는 숨김) |
| `strength.assignment` | round_robin | 시나리오마다 프로필을 돌아가며 배정(전체 데이터셋 순서 기준) |
| `strength.damage_factor` | 0.25 | 파손 박스는 윗면 강도가 25 %만 남음 |
| `strength.detect_probability` | 0.8 | 파손 박스를 2단계 검사가 찾을 확률. 찾으면 `capacity_overrides_n = 0`(위에 아무것도 못 올림) |

## 박스 강도 시나리오 (사양 비공개 대응)

박스 사양이 공개되지 않으므로 강도를 하나로 가정하지 않고 **여러 경우를 생성**합니다.
각 시나리오는 프로필 하나를 받고, SKU마다 골판지 등급, 박스마다 ±10 % 편차와 파손 여부를 뽑습니다.

`실제 허용하중 = McKee BCT(등급의 ECT, 두께) × 환경계수(습도·보관 기간) × 편차 × (파손 시 0.25)`

| 프로필 | 골판지 등급 비율 | 환경계수 | 파손 확률 |
|---|---|---|---|
| strong | 단면 C 40 %, 이중 BC 60 % | 0.6~0.8 | 0 % |
| nominal | 단면 B 60 %, 단면 C 40 % | 0.5~0.7 | 2 % |
| weak | 단면 E 50 %, 단면 B 50 % | 0.4~0.6 | 4 % |
| humid | 단면 B 50 %, 단면 C 50 % | 0.3~0.45 | 3 % |
| mixed | E·B·C·BC 혼합 | 0.35~0.8 | 3 % |
| extreme (스트레스) | 단면 E 70 %, 단면 B 30 % | 0.2~0.3 | 10 % |

등급별 ECT·두께: 단면 E 2.5~3.5 kN/m·1.5 mm, 단면 B 3.5~5 kN/m·3 mm, 단면 C 4.5~6 kN/m·4 mm, 이중 BC 7~10 kN/m·7 mm.
모두 일반적인 골판지 범위를 가정한 **개발용 값**입니다.

- planner(5-②)는 실제 값을 모르고, `candidates.yaml`의 가정 허용하중(McKee/안전계수)만 사용합니다.
- 최종 팔레트에서 실제 강도로 하중을 다시 계산합니다. 그 결과로 실제 붕괴 박스 수, 최대 실제 하중비, 검출된 파손 박스 위 적재 여부를 `episodes/`와 `analysis/summary.md`에 기록합니다.
- 실제 강도(`true_capacity_n`, `damaged_box_ids`)는 `episodes/`(정답 데이터)에만 저장합니다. planner 장면에는 넣지 않으며, 검증기가 누출을 검사합니다.

가정 안전계수별로 위험을 비교하려면 다음을 실행합니다.

```bash
python tools/virtual_data/scripts/strength_sweep.py --dataset OUT/source_dataset \
    --safety-factors 1 2 4 8 16 --report strength_sweep.json
```

## 팔레트를 끝까지 채우는 시나리오

재성 님 제너레이터 기본값은 시나리오당 24박스입니다(평균 높이 약 0.7 m). 하중 조건을 시험하려면 박스 수를 늘립니다.
제너레이터 설정 파일은 수정하지 않고, 복사본에서 `boxes_per_scenario`만 바꿔 실행합니다.

```bash
python tools/virtual_data/scripts/generate_virtual_data.py --run-generator sample \
    --sample-per-family 2 --boxes-per-scenario 80 --output tools/virtual_data/output/full
```

## 출력 스키마

### `scenes/<split>/Sxxxx-Tyyy.json`

동한 님 `pac_planning.demo.scene_from_file()`로 그대로 읽힙니다.

```json
{
  "schema_version": "pac-common-v0.2+planning-v1",
  "scenario_id": "S0001-T010",
  "current_box_id": "S0001-B011",
  "state":   { "...": "plain(SystemState): 팔레트 위 박스 + tracked_boxes + 미입고 remaining_by_sku" },
  "context": { "...": "plain(PlanningContext): catalog(McKee 허용하중), pallet_max_weight_kg, uncertain_box_ids" },
  "source":  { "generator_scenario_id": "S0001", "scenario_family": "normal", "split": "train", "step": 10, "state_kind": "SIMULATED" }
}
```

- 미래 도착 순서는 들어가지 않습니다(누출 검사 포함).
- `remaining_by_sku`에는 현재 박스와 이미 놓인 박스가 포함되지 않습니다(동한 님 계약).
- split은 재성 님 `splits.json`의 시나리오 단위 split을 그대로 따릅니다. 같은 시나리오가 두 split에 섞이지 않습니다.

### `candidate_sets/Sxxxx.jsonl` (한 줄 = 한 결정 단계)

| 필드 | 내용 |
|---|---|
| `scenario_id`, `scenario_family`, `split`, `step`, `state_version`, `box_id`, `box`, `box_uncertain` | 식별 정보 |
| `raw_count`, `generated_count`, `valid_count`, `masked_count`, `ems_count` | "54 generated → 31 masked → 23 valid" 형태의 수치 |
| `code_counts`, `reason_counts` | 마스크 사유 집계 |
| `candidates[]` | `candidate_id`, `pose{x,y,z,yaw}`, `source`(EMS/EP), `anchor`, `ems_upper_m`, `valid`, `reject_codes`, `reasons`, `evidence`(통과 시), `metrics`(지지율, LBCP 여유, 최대 하중비, 배치 후 팔레트 CoG) |
| `chosen_candidate_id`, `decision` | 정책이 실제로 적재한 후보 |
| `generation_sec`, `mask_sec`, `seed`, `run_id` | 재현·성능 기록 |

### `episodes/Sxxxx.json`, `analysis/`

- 에피소드: 최종 팔레트(`plain(SystemState)`), 미적재 박스, **실제 크기**, 지표(적재율, 부피 활용률, 높이, 안정성 이슈, 실제 크기 기준 관통/돌출).
- `analysis/summary.json|md`(패밀리별 집계), `analysis/steps.csv`(단계별), `manifest.json`(설정, 카탈로그, 소스 데이터셋, git commit).

## 활용

1. **동한 님 5-③~⑥ 학습/평가**: `scenes/`를 `scene_from_file`로 읽어 실제 EMS/LBCP 검사기 기준 교사 데이터를 다시 만들 수 있습니다.
   integration.md의 "실제 검사기로 바꾼 뒤 교사 데이터를 재생성" 항목에 해당합니다.
2. **5-② 분석**: `candidate_sets/`의 사유·evidence로 마스크 분포와 경계 사례를 분석합니다.
3. **물리 검증**: `tools/virtual_data/scripts/physics_crosscheck.py`가 이 출력을 재성 님 시뮬레이터로 검증합니다.

## 2D 시각화 (5-①/5-② 결정 화면)

미션 필수 고려사항인 "알고리즘 결과를 확인할 수 있는 2D 또는 3D 시각화"에 대응합니다.
장면 파일 하나로 5-① 후보와 5-② 판정을 위에서 내려다본 SVG를 그립니다.

```bash
python tools/virtual_data/scripts/visualize_candidates.py \
    tools/virtual_data/output/run1/scenes/train/S0001-T010.json --output viz/
```

- 쌓인 박스: 회색(높을수록 진함), 숫자는 윗면 높이(m)
- 후보 footprint: 유효(초록 실선) / 탈락(빨강 점선). 마우스를 올리면 후보 ID, z, 탈락 사유가 표시됩니다.
- 선택된 후보(파랑)와 그 후보의 EMS(보라 점선)
- 오른쪽: 박스 정보, "N generated → M masked → K valid", 탈락 사유 집계

예시: [`reports/example_decision.svg`](reports/example_decision.svg)
