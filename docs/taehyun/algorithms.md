# 5-① 후보 생성 · 5-② Hard Mask 알고리즘

## 0. 공통 기하 모델 (`pallet_model.py`)

스냅샷(`SystemState`) 하나당 한 번 만들고, 5-①과 5-②가 함께 씁니다.

- **박스 AABB**: 회전 후 치수. 위치는 최소 모서리 기준입니다.
- **δ 확장 footprint**: 실제 footprint를 `lateral_clearance/2 + size_tolerance`만큼 사방으로 넓힌 사각형입니다.
  두 박스의 확장 footprint가 겹치지 않으면 실제 간격 ≥ `lateral_clearance + 두 박스의 size_tolerance`가 보장됩니다.
  팔레트 경계도 `lateral_clearance/2`만큼 넓혀서, 가장자리에는 `size_tolerance`만 남깁니다.
- **지지 그래프**: 박스 B의 바닥 높이와 박스 A의 윗면 높이 차가 `height_tolerance`(3 mm) 이내이고 footprint가 겹치면 A가 B를 지지합니다.
- **LBCP (Load-Bearing Convex Polygon)**: 각 박스 윗면에서 하중을 안전하게 받을 수 있는 영역입니다.
  - 바닥에 놓인 박스: footprint(허용오차만큼 축소)
  - 다른 박스 위에 놓인 박스: (자기 footprint ∩ 각 지지 박스의 LBCP)들의 convex hull
  - 새 박스의 최악 CoG(중심 ± δ)가 이 다각형 안에 있으면, 새 박스가 전달하는 힘은 모두 지지 박스의 LBCP 안에 작용합니다.
    따라서 **아래쪽 박스 전체의 안정성이 유지**됩니다. 층마다 물리 해석을 할 필요가 없어서 빠릅니다.
- **누적 하중**: 위에서 아래로 내려가며 각 박스가 받는 총 힘과 합력 위치를 계산합니다.
  하중은 `lever` 모델로 지지 박스들에 나눕니다. 힘·모멘트 평형을 만족하면서 면적 가중 최소노름이 되는 해이고, 인장(음수)은 제거합니다.
  합력 바로 아래 박스가 더 많이 받기 때문에, 면적 비례(`area`)보다 현실적이고 보수적입니다.
- **압축 heightmap**: 확장 footprint의 경계 좌표만으로 만든 격자입니다. 해상도 손실 없이 정확합니다.

## 1. 5-① 후보 생성 (`candidate_generation.py`)

1. **EMS (Heightmap 기반)**: heightmap의 서로 다른 높이 h마다 "높이 ≤ h"인 셀에서 **최대 직사각형(maximal rectangle)**을 모두 구합니다.
   높이가 정확히 h인 바닥 셀을 하나 이상 포함하는 것만 h층의 EMS로 남깁니다. 천장은 최대 적재 높이입니다.
   최대 직사각형 열거는 무작위 격자 40개에서 brute-force 결과와 일치함을 테스트로 확인했습니다.
2. **기준점 5개**: EMS마다 네 모서리(좌하·우하·좌상·우상)와 중앙. 박스가 EMS(3D)에 들어가는 경우만 만듭니다.
3. **Extreme Point**: 원점 모서리, 각 박스의 오른쪽/앞쪽 점, 그 점을 원점 방향 벽까지 밀어 붙인 투영점, 각 박스의 윗면 모서리.
4. **방향**: `yaw_set_rad` ∩ `allowed_yaws_rad`. 정사각 footprint처럼 결과가 같은 yaw는 한 번만 만듭니다.
5. **z**: 확장 footprint를 heightmap에 떨어뜨린 높이(가장 높은 겹치는 윗면). numpy로 한 번에 계산합니다.
6. **중복 제거 (80 mm)**: 같은 yaw, 같은 높이에서 x·y 차이가 모두 80 mm 미만이면 중복으로 봅니다.
   `support_aware`(기본): 군집 안에서 지지율·높이·무거운-위-가벼운 근사 검사를 통과하는 후보를 대표로 고릅니다.
   그래서 중복 제거 때문에 유효한 이웃이 사라지지 않습니다. `mask_aware`(정확한 Hard Mask), `geometric`, `off` 모드도 있습니다.
7. **출력 순서**: `likely_valid_first`. 근사 검사 통과 그룹이 먼저 오고, 각 그룹 안은 deepest-bottom-left 순서 (z, y, x, 모서리 우선)입니다.

## 2. 5-② Hard Mask (`hard_mask.py`)

시간 예산과 관계없이 **절대 생략하지 않습니다**. 비용이 싼 검사부터 실행합니다.

| # | 검사 | 기준 |
|---|---|---|
| 1 | 방향 | roll = pitch = 0, yaw가 π/2의 배수이고 허용 목록에 있음 |
| 2 | 경계 | (footprint + size_tolerance) ⊂ 팔레트 (+ `pallet_overhang_m`, 기본 0 = 돌출 금지) |
| 3 | 높이 | z + 높이 + size_tolerance ≤ 최대 적재 높이 |
| 6 | 접촉 | z가 낙하 높이와 같음 (FLOATING 금지) |
| 4 | 겹침/여유폭 | 확장 footprint가 겹치는데 높이 구간도 겹침 → BOX_COLLISION (실제 겹침이면 OVERLAP, 여유폭 δ 미달이면 CLEARANCE) |
| 5 | 하강 경로 | footprint 위쪽에 박스가 있음 → APPROACH_FAIL |
| 11 | 팔레트 하중 | 총중량 ≤ `pallet_max_weight_kg` |
| 7 | 지지율 | 접촉 면적 / footprint ≥ `min_support_ratio` (0.70) |
| 8 | LBCP 안정 | 최악 CoG(중심 ± δ)가 지지 다각형 안 (여유 ≥ `min_lbcp_margin_m`) |
| 9 | 박스 허용하중 | 새 박스로 늘어나는 하중을 아래로 전파한 뒤, 모든 박스의 상부하중 ≤ 허용하중 |
| 10 | 무거운-위-가벼운 | 지분 ≥ 10 %인 직접 지지 박스에 대해 새 무게 ≤ 지지 박스 무게 × 1.0 + 0.5 kg |
| 12 | 팔레트 CoG | 배치 후 CoG가 허용 영역 안. 허용 영역은 적재량이 늘수록 줄어들고, CoG를 중심 쪽으로 되돌리는 배치는 허용 |

## 3. 기본값과 근거 (`config/taehyun/candidates.yaml`)

모두 **개발용 가정**입니다. 실측값이 생기면 YAML만 바꾸면 됩니다.

| 설정 | 기본값 | 근거 |
|---|---|---|
| `yaw_set_rad` | 0, π/2 | 흐름도 "1차 0°/90°" |
| `dedup_distance_m` | 0.08 | 흐름도 "80 mm 단위 중복 제거" |
| `lateral_clearance_m` | 4 mm | 로봇 반복 정밀도(±1~2 mm)와 박스 배부름 여유. 이웃 박스 간 실제 최소 간격은 4 + 2 + 2 = 8 mm |
| `size_tolerance_m` | 2 mm | 측정 오차 δ. 가상데이터 측정 노이즈(σ 1 mm, 최대 3 mm)는 여유폭 안에서 흡수됨을 확인 (실제 크기 관통 0건) |
| `height_tolerance_m` | 3 mm | 같은 층으로 볼 윗면 높이 차 |
| `cog_uncertainty_ratio` / `min_m` | 5 % / 5 mm | 내용물 편심을 반영한 CoG 불확실성 δ (축마다 ± max(5 % × 변, 5 mm)) |
| `uncertain_multiplier` | 2.0 | 불확실 박스(`uncertain_box_ids`)는 허용오차와 δ를 2배로 |
| `min_support_ratio` | 0.70 | 공통 기준서 19장 예시 값 |
| `min_lbcp_margin_m` | 0 | δ로 이미 보수적으로 잡았으므로 추가 여유 없음 |
| `pallet_overhang_m` | 0 | 미션 필수 고려사항: 팔레트 외부 돌출 방지 |
| 허용하중 (McKee) | ECT 5 kN/m, 두께 3 mm, 안전계수 4 | 일반 단면 골판지. 40×30 cm 박스 ≈ 49 kg 허용. 습도·장기 적재를 고려해 안전계수 4 |
| `share_model` | lever | 위 0장 참고 |
| 무거운-위-가벼운 | per_box, 비율 1.0, 허용 0.5 kg, 최소 지분 10 % | 미션 필수 고려사항 문구 그대로. 측정 노이즈로 판정이 흔들리지 않도록 0.5 kg 허용 |
| 팔레트 CoG | 최종 중앙 50 % 영역, 150 kg까지 선형 완화, 개선 허용 | 처음 박스를 모서리부터 놓을 때 막히지 않게 하면서, 적재가 진행되면 지게차·랩핑에 안전한 중심을 유지 |
| `default_pallet_max_weight_kg` | 1000 | 제너레이터의 `max_load_kg`가 null이라 T11 팔레트 일반 정격으로 설정 |

## 4. 알려진 한계

- yaw는 π/2의 배수만 지원합니다(흐름도 1차 범위). 박스를 뒤집는 배치는 없습니다.
- 팔레트 바닥은 연속 평면으로 봅니다. 재성 님의 slatted(판재 사이 틈) 팔레트 시뮬레이션에서도 통과 후보 100 %가 안정이었습니다.
- 하중 전파는 스냅샷의 분배 비율을 고정한 선형 중첩입니다. 새 박스가 아래 박스들의 합력 위치를 조금 옮기는 효과는 무시합니다.
- 박스는 강체로 가정합니다. 골판지 변형은 허용하중의 안전계수로만 반영합니다.
