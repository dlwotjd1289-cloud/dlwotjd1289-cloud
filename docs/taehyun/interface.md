# 연결 계약: 5-①/5-② ↔ 5-③~⑥ (동한), 가상데이터 ↔ 제너레이터 (재성)

기준 문서는 동한 님 브랜치의 `docs/integration.md`와 공통 개발 기준 v0.2입니다.
**공통 계약(`pac_common`, `config/default.yaml`)은 바꾸지 않았고 추가 필드도 만들지 않았습니다.**

## 1. 호출 signature

```python
from pac_candidates import CandidateBackend, load_candidate_config

backend = CandidateBackend(context, load_candidate_config("config/taehyun/candidates.yaml"))

backend.generate_candidates(box: BoxState, state: SystemState) -> list[PlacementCandidate]
backend.validate_constraints(box: BoxState, candidate: PlacementCandidate,
                             state: SystemState) -> ValidationResult
```

동한 님 planner에는 아래처럼 연결합니다(테스트 `tests/taehyun/test_th_planner_integration.py`에서 확인).

```python
report = backend.generate_with_report(box, state)          # 5-① + 생성 정보
ctx = backend.context_with_ems(context, report)            # EMS 상한을 PlanningContext에 공급
planner = PlacementPlanner(context=ctx, config=...,
                           generate_candidates=backend.generate_candidates,
                           validate_constraints=backend.validate_constraints)
result = planner.plan(box, state, list(report.candidates), seed=42)
```

`context_with_ems`를 쓰면 5-③ 특징이 `FOOTPRINT_COLUMN_PROXY` 대신 **`EMS_SUPPLIED`**(실제 EMS 범위)로 계산됩니다.
벤치마크에서 특징의 100%가 EMS_SUPPLIED였습니다.

## 2. 좌표·단위 (동한 님 계약을 그대로 따름)

- SI 단위: m, kg, N, rad. `target_pose.frame_id == "pallet"`
- `target_pose.xyz` = **yaw 적용 후 박스 AABB의 최소 x/y/z 모서리**. `z = 0`은 팔레트 적재면입니다.
- `PalletState.size.z` = 적재면 위 **최대 적재 높이** (팔레트 목재 두께 아님)
- yaw는 `box.allowed_yaws_rad`와 설정 `yaw_set_rad`(1차: 0, π/2)의 교집합입니다. 똑같은 footprint가 되는 yaw는 한 번만 생성합니다.
- roll/pitch는 항상 0입니다(뒤집기 없음).

## 3. 5-① 출력

- 후보 ID: `S{state_version:04d}-{box_id}-C{index:03d}` (공통 기준 5장 형식)
- `z`는 항상 낙하 높이(현재 heightmap 위에 닿는 높이)입니다. 공중에 뜬 후보는 생성하지 않습니다.
- **출력 순서 = likely-valid-first**: 지지율·높이·무거운-위-가벼운 근사 검사를 통과한 후보가 먼저 오고, 각 그룹 안에서는 (z, y, x) 순서입니다.
  5-③의 잔여재고 프로브는 앞 16개만 검사하므로, 이 순서 덕분에 "놓을 곳이 있는데 없다고 판단"하는 경우가 13% → 0%로 줄었습니다.
  (`generation.order: priority`로 순수 (z, y, x) 순서도 선택할 수 있습니다.)
- 부가 정보: `generate_with_report()` → `GenerationReport.infos[candidate_id]` = `CandidateInfo(source="EMS"|"EP", anchor, yaw, footprint, ems_lower_m, ems_upper_m)`

## 4. 5-② 출력

| 결과 | 내용 |
|---|---|
| 통과 | `ValidationResult(True, details={"evidence": ConstraintEvidence, "metrics": {...}})` |
| 탈락 | `ValidationResult(False, codes=(RejectCode, ...), details={"reasons": (...), "metrics": {...}})` |

`ConstraintEvidence` 필드를 이렇게 채웁니다 (`source = "LBCP_EMS_DELTA_V1"`):

| 필드 | 계산 |
|---|---|
| `support_ratio` | 접촉 높이(±3 mm)가 같은 하부 박스와 실제로 겹치는 면적 / footprint 면적. 바닥은 1 |
| `cog_margin_ratio` | min(LBCP 여유, 팔레트 CoG 여유). LBCP 여유 = CoG 불확실성 δ를 뺀 최악 CoG에서 지지 다각형 경계까지 거리 / (footprint 짧은 변 / 2) |
| `load_margin_ratio` | 배치 후 모든 박스의 min(1 − 상부하중/허용하중) |
| `pallet_load_margin_ratio` | 1 − 배치 후 총중량 / `pallet_max_weight_kg` |
| `max_load_ratio` | 배치 후 모든 박스의 max(상부하중/허용하중) |
| `support_centering` | 박스 중심에서 지지 다각형 경계까지 거리 / footprint 짧은 변 (0~0.5), 바닥은 0.5 |
| `dependency_count` | 이 후보 아래로 이어지는 하부 박스 수(전이적) |

거부 코드와 상세 사유(`details["reasons"]`):

| RejectCode | reasons 접두어 |
|---|---|
| `INVALID_STATE` | ORIENTATION_NOT_UPRIGHT / _AXIS_ALIGNED / _ALLOWED, BELOW_PALLET_SURFACE, BOX_ID_MISMATCH, ALREADY_PLACED, BOX_STATUS_*, FRAME_NOT_PALLET |
| `STALE_PLAN` | STALE_PLAN (`base_state_version` 불일치) |
| `OUT_OF_BOUND` | PALLET_BOUNDARY |
| `HEIGHT_LIMIT` | MAX_STACK_HEIGHT |
| `BOX_COLLISION` | OVERLAP:<id>, CLEARANCE:<id> (여유폭 δ 미달) |
| `APPROACH_FAIL` | OVERHEAD_OCCUPIED:<id> (위에서 내려놓는 경로가 막힘) |
| `LOW_SUPPORT` | FLOATING, SUPPORT_RATIO |
| `COG_VIOLATION` | LBCP_UNSTABLE, PALLET_COG |
| `LOAD_VIOLATION` | BOX_CAPACITY:<id>, HEAVY_ON_LIGHT:<id>, PALLET_MAX_WEIGHT |
| `SENSOR_UNCERTAIN` | UNCERTAIN_POLICY_REJECT (`uncertain_policy: reject`일 때만) |

런타임 기본값(`collect_all_reasons: false`)에서는 비용이 싼 검사부터 하고 첫 실패 그룹에서 멈춥니다.
**통과/탈락 판정은 전체 검사 모드와 항상 같습니다**(테스트로 보장). 차이는 부가 사유 목록의 길이뿐입니다.
학습 라벨을 만드는 가상데이터는 `collect_all_reasons: true`로 모든 사유를 기록합니다.

## 5. PlanningContext 사용

| 필드 | 사용 |
|---|---|
| `catalog[sku].top_load_capacity_n` | 박스 상부 허용하중. 없으면 McKee 식으로 추정 |
| `capacity_overrides_n[box_id]` | 박스별 허용하중 우선 적용 (예: 찌그러진 박스 0 N) |
| `pallet_max_weight_kg` | 팔레트 최대 하중 |
| `uncertain_box_ids` | 기본 `robust`: 해당 박스의 치수 허용오차와 CoG δ를 2배로 늘려 검사. 참조 검사기처럼 일괄 거부하지 않음 |
| `ems_upper_by_candidate` | `context_with_ems()`가 채움 |

context가 없으면(`CandidateBackend(None)`) 설정의 `default_pallet_max_weight_kg`와 McKee 허용하중을 씁니다.

## 6. 성능 특성 (롤아웃 대응)

- 스냅샷별 기하 모델, 판정, 후보 목록을 캐시합니다. 캐시 키는 박스 ID가 아니라 **박스 기하(크기·무게·yaw·불확실성)**입니다.
  그래서 5-⑤ 롤아웃의 `__future__…` 가상 박스나 5-③의 `__probe__…` 박스도 캐시를 공유합니다.
- 입력 상태는 절대 수정하지 않습니다(frozen dataclass, 테스트로 확인).

## 7. 재성 님 제너레이터·시뮬레이터와의 연결

- 제너레이터: subprocess로 실행하고 출력 파일(`test_data`, `ground_truth`, `analysis/catalog.csv`, `splits.json`)만 읽습니다.
  제너레이터 팔레트의 `size.z`(목재 0.15 m)와 `max_height_m`(1.5 m)는 **적재 높이 1.35 m**로 변환합니다(`height_limit_includes_pallet: true`).
- 시뮬레이터: `pac_simulation.ahead_sim.world.BulletPalletWorld`로 물리 교차 검증을 합니다(좌표: 팔레트 중심 원점, 박스 중심 좌표).

## 8. 팀 확인이 필요한 사항

1. ~~최대 높이 해석~~ → **확정 (2026-10-07, 태현)**: 1.5 m는 팔레트 목재(0.15 m)를 포함한 값입니다. 적재 높이는 1.35 m입니다(`height_limit_includes_pallet: true`).
2. **허용하중 값**: SKU별 실측 압축강도가 없어서 McKee 식(ECT 5 kN/m, 두께 3 mm, 안전계수 4)으로 추정했습니다.
3. ~~무거운-위-가벼운 규칙~~ → **확정 (2026-10-07, 태현)**: 미션 문구대로 `per_box`(직접 지지 박스 각각과 무게 비교)를 사용합니다.
   late_heavy처럼 바닥이 가벼운 박스로 차면 무거운 박스는 놓을 곳이 없으며, 이 경우는 4단계 버퍼/재적재가 처리합니다.
4. **5-③ 프로브 비용 (동한 님 제안)**: 유효 후보마다 SKU 수만큼 생성기를 호출하는 구조라, 유효 후보가 30개 이상이면 1초 예산의 대부분을 씁니다.
   Top-K 이후에만 프로브를 돌리거나 SKU 프로브를 상한으로 제한하면 롤아웃 축소(degradation)가 줄 것입니다. 이 파트 코드는 바꾸지 않았습니다.
