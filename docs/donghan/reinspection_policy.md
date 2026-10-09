# 재인식(Reinspection) 사유 규정 — RI 코드

- 기준: origin/main **2bb4bac** (2026-10-09)의 기존 동작
- 구현 위치: `ros2_ws/src/pac_reinspection/` (2단계 확장, 동한), `ros2_ws/src/pac_perception/pac_perception/signals.py` (1단계 원신호)
- 설정: `config/donghan/reinspection_policy.yaml` — **모든 수치는 시험용 가정치**(`assumed: true`)
- 상태: 코드는 들어왔지만 **런타임에는 아직 연결되지 않았습니다**(`pac_runtime/core.py`는 그대로 `StateValidator`를 씀). 연결 전에 담당자 확인이 필요합니다(9절).

## 1. 원칙

1. 1단계(인식)는 원신호만 만듭니다. 이상 분류와 경로는 2단계에서만 정합니다.
2. 기존 `Anomaly` 5종(OK / RECOGNITION_FAIL / DAMAGED / SPEC_MISMATCH / UNKNOWN_SKU)과 route(PLAN / INSPECTION)를 그대로 씁니다.
3. `pac_common.RejectCode`는 늘리지 않습니다. 세부 사유는 `Verdict.notes`에 `"RI-xxx: …"`로, 코드 목록은 `ReinspectionResult.ri_codes`로 남깁니다. 기존 RejectCode 중 `SENSOR_UNCERTAIN`을 `codes`에 덧붙이는 것만 합니다.
4. 태현 님 `StateValidator`는 수정하지 않고 감쌉니다(`ReinspectionValidator`). 시그니처 `validate(obs, perception, field_box)`가 같아서 `RuntimeCore`가 그대로 쓸 수 있습니다(8절).
5. `RawObservation` 계약은 바꾸지 않습니다. 거기에 없는 원신호는 `PerceptionSignals`로 따로 넘깁니다(6절).

## 2. 처리 경로

```
RawObservation (+ PerceptionSignals)
  → 사전 검사 (원신호): B7 B6 A3 A7 C3 C7 C2 C8 C5 D2 — 필요하면 1단계에 재판독/재계량/재촬영 요청 (상한·예산 안)
  → StateValidator.validate (기존 그대로; Base-view는 박스당 최대 1회)
  → 사후 처리: A4 B3 B2 B4 A6 → 재감지 후 다시 validate (상한·예산 안)
  → 경로
     OK                                  → PLAN
     OK + 확신도 < 1 (C1)                 → PLAN, δ
     센서 불확실 미해소 (B6 B7 C2 C3 C7)   → PLAN, δ, codes += SENSOR_UNCERTAIN
     C5 / C8 / A6 / A7 미해소, F6 상한     → INSPECTION
     DAMAGED, damage_policy=reject        → INSPECTION (알림은 notes 문구뿐)
     DAMAGED, damage_policy=place_no_load → PLAN, 상부 허용하중 0 N
```

INSPECTION 처리는 기존과 같습니다. 2단계 INSPECTION은 `RuntimeCore.on_observation`이 바로 처리합니다(`discard_expected` 후 inspection 목록). `ActionType.REJECT_NG` / `on_result`는 4단계 행동일 때의 경로입니다.

## 3. RI 코드

상태: **기존** = origin/main에 있는 동작, **신규** = 제안. 범위: 시뮬 = 시뮬레이션으로 검증 가능, Gazebo = 재성 님 Gazebo 스크립트에 있음(런타임 미연결), HW = 실제 검출기·센서 필요, SV = Supervisor(3단계) 영역. 구현 = 이 저장소에서 처리.

### A. 신원 (라벨 / ID)
| 코드 | 사유 | 결과 Anomaly / 코드 | 처리 | 상태 | 범위 | 구현 |
|---|---|---|---|---|---|---|
| RI-A1 | 라벨 미판독 | RECOGNITION_FAIL / TRACKING_LOST | Base-view 1회, 실패 시 INSPECTION | 기존 | 시뮬 | 표시 |
| RI-A2 | 확신도 < 0.5 (**라벨·크기 공용 값**) | 동일 | 동일 | 기존 | 시뮬 | 표시 |
| RI-A3 | 라벨 부분 판독 / 체크섬 불일치 | RECOGNITION_FAIL | 재판독(상한), 실패 시 라벨 없음 → A1 경로 | 신규 | 시뮬 | ○ |
| RI-A4 | SKU가 catalog에 없음 | UNKNOWN_SKU / INVALID_STATE | 기존: 즉시 INSPECTION. 신규: 재판독 1회 | 기존 | 시뮬 | ○ |
| RI-A5 | box_id 중복 / 이미 PLACED | – | 기존: `on_observation`이 무시(`duplicate_observation`). 신규: HOLD + 알림 | 기존(일부) | SV | – |
| RI-A6 | 잔여 수량 0인 SKU 도착 | UNKNOWN_SKU / INVALID_STATE | 재판독, 실패 시 INSPECTION | 신규 | 시뮬 | ○ |
| RI-A7 | 상단 / 보조 뷰 라벨 불일치 | RECOGNITION_FAIL / TRACKING_LOST | 3번째 판독 다수결, 실패 시 INSPECTION | 신규 | 시뮬 | ○ |

### B. 센서 교차 불일치
| 코드 | 사유 | 결과 | 처리 | 상태 | 범위 | 구현 |
|---|---|---|---|---|---|---|
| RI-B1 | 크기 불일치 (축별 max(12 mm, 4 %)) | SPEC_MISMATCH / SIZE_MISMATCH | Base-view 재측정, 일치하면 OK, 아니면 실측값 + δ | 기존 | 시뮬 | 표시 |
| RI-B2 | 무게가 범위 ±8 % 밖 | SPEC_MISMATCH / WEIGHT_MISMATCH | 기존: 실측값 + δ (무게는 다시 재지 않음). 신규: 재계량 | 기존 | 시뮬 | ○ |
| RI-B3 | 크기·무게 동시 불일치 | SPEC_MISMATCH / 두 코드 모두 (기존에도 함께 기록) | 신규: 라벨 재판독 우선 | 신규(처리) | 시뮬 | ○ |
| RI-B4 | 높이만 불일치 | SPEC_MISMATCH / SIZE_MISMATCH | 재촬영 | 신규 | 시뮬 | ○ |
| RI-B5 | x/y 뒤바뀜 | OK | 통과, 기록만 | 기존 | 시뮬 | 표시 |
| RI-B6 | 저울 미안정 | (기존 kind) + SENSOR_UNCERTAIN | 재계량, 미해소 시 PLAN + δ | 기존(Gazebo) | Gazebo | ○ |
| RI-B7 | 저울 영점 이상 | (기존 kind) + SENSOR_UNCERTAIN | 영점 재설정 후 재계량, 미해소 시 PLAN + δ | 기존(Gazebo) | Gazebo | ○ |

B6·B7: 재성 님 `scripts/scale_cycle_core_v43.py` `AutoScaleCycle`에 이미 있습니다(표준편차 0.12 kg, 안정화 15 s 제한, TARE 기대값 ± 허용치). 런타임·검증기에는 연결돼 있지 않아 `PerceptionSignals.scale_settled / scale_stdev_kg / tare_ok`로 넘기도록 했습니다.

### C. 인식 품질
| 코드 | 사유 | 결과 | 처리 | 상태 | 범위 | 구현 |
|---|---|---|---|---|---|---|
| RI-C1 | 확신도 < 1.0 | OK, uncertain | 재인식 없이 δ | 기존 | 시뮬 | 표시 |
| RI-C2 | 프레임 간 크기 분산 초과 | (기존 kind) + SENSOR_UNCERTAIN | 프레임 추가, 중앙값 사용, 미해소 시 PLAN + δ | 신규 | 시뮬 | ○ |
| RI-C3 | 윤곽 일부 가림 / 시야 경계 | 동일 | 재촬영, 미해소 시 PLAN + δ | 신규 | 시뮬 (`measure_box.in_full_view`) | ○ |
| RI-C4 | 반사·그림자·저조도로 윤곽 실패 | RECOGNITION_FAIL | 노출 조정 / Base-view | 신규 | HW | – |
| RI-C5 | 두 박스가 하나로 인식 | SPEC_MISMATCH / SIZE_MISMATCH | 분리 대기 후 재촬영, 미해소 시 **INSPECTION** | 신규 | 시뮬 | ○ |
| RI-C6 | 한 박스가 두 윤곽으로 쪼개짐 | SPEC_MISMATCH | 재촬영, 병합 규칙 | 신규 | 시뮬 | – (다음 단계) |
| RI-C7 | 깊이 결측률 초과 | (기존 kind) + SENSOR_UNCERTAIN | 재촬영, 미해소 시 PLAN + δ | 신규 | 시뮬 (`valid_ratio`) / 실측은 HW | ○ |
| RI-C8 | 도착 자세가 예상 범위 밖 | RECOGNITION_FAIL / SENSOR_UNCERTAIN | 재추정, 미해소 시 INSPECTION | 신규 | 시뮬 | ○ |

C8 기준: `allowed_yaws_rad`는 **팔레트 위 배치 방향**이므로 비교 기준이 아닙니다. 컨베이어 위 긴 변 yaw가 0°/90°에서 얼마나 벗어났는지로 봅니다(가정치 15°, V4.4 도착 흔들림은 ±10°).

### D. 외관 / 상태
| 코드 | 사유 | 결과 | 처리 | 상태 | 범위 | 구현 |
|---|---|---|---|---|---|---|
| RI-D1 | 눈에 띄는 파손 (`visual_damage`) | DAMAGED | reject: INSPECTION / place_no_load: PLAN + 상부 0 N | 기존 | 시뮬 | 표시 |
| RI-D2 | 파손 단서가 경계값 부근 | DAMAGED (미해소 시) | 다른 뷰로 재확인, 미해소면 파손으로 간주 | 신규 | 시뮬 (`damage_score`) | ○ |
| RI-D3 | 무게가 범위 아래 (내용물 이탈 의심) | SPEC_MISMATCH / WEIGHT_MISMATCH | 검출은 기존(B2 경로). DAMAGED 재분류는 팀 결정이라 기록만 | 기존(검출) | 시뮬 | 표시 |
| RI-D4 | 테이프 풀림·뚜껑 열림·윗면 돌출 | DAMAGED | 재측정 | 신규 | HW | – |
| RI-D5 | 젖음·오염 | DAMAGED | INSPECTION | 신규 | HW | – |

### E. 흐름 / 재고 (3단계)
| 코드 | 사유 | 처리 | 상태 | 구현 |
|---|---|---|---|---|
| RI-E1 | 컨베이어 유휴 30 s (`missing_timeout_s`) | **자동 확정**: 남은 기대 수량 전체를 MISSING으로 `remaining`에서 제거 (`Supervisor.confirm_missing`) | 기존 | – |
| RI-E2 | 도착 순서·간격 이상 | 라벨 재판독 | 신규 | – |
| RI-E3 | 대기열에서 본 수 > 미도착 기대 수 | 컨베이어 정지 후 재집계 (`outstanding()`은 기존, 비교는 신규) | 신규 | ○ `inventory_consistency` |
| RI-E4 | 같은 SKU 연속 도착 이상 | 경고 로그 | 신규 | – |
| RI-E5 | 인식 후 픽업 지연으로 자세 변화 | 인식 폐기 후 재인식 (STALE_PLAN은 지금 상태 버전 불일치에만 쓰임) | 신규 | – |

### F. 작업 중 피드백 (7단계)
| 코드 | 사유 | 실제 동작 / 처리 | 상태 | 범위 |
|---|---|---|---|---|
| RI-F1 | 파지 실패 | 재시도 1회 → 다른 파지(yaw+π) → L3, Inspection 사유는 문자열 `"GRIP_FAIL"` (RejectCode 아님) | 기존 | 시뮬 |
| RI-F2 | 파지 후 무게가 인식 때와 다름 | 내려놓고 재계량 | 신규 | HW (Gazebo F/T 센서로 시험 가능) |
| RI-F3 | 이송 중 낙하 | V4.4 `moveit_pick_place_v44.py`가 박스 위치 신호로 감지 → Fatal (무게 기준 아님, 런타임 미연결) | 기존(Gazebo) | Gazebo |
| RI-F4 | 배치 후 위치 오차 | L0 ≤ 5 mm, L1 ≤ 20 mm/10 mm, 그 밖은 L2 기록. 형상 문제는 L4 → HOLD(120 s) + 보정 위치. 재측정·재적재는 없음 | 기존 | 시뮬 |
| RI-F5 | 배치 후 기울어짐·밀림 | 검사 문자열 `LOW_SUPPORT:x`, `PENETRATION:id`, `PROTRUSION`, `HEIGHT_LIMIT` → L4 HOLD (RejectCode·Partial Repack 아님) | 기존 | 시뮬 |
| RI-F6 | 재인식을 반복해도 판정이 계속 바뀜 | 같은 box_id의 판정 변경 횟수 ≥ 상한(가정 3) → INSPECTION | 신규 | 시뮬, 구현 ○ |

## 4. 요청서 대비 코드 기준으로 바로잡은 내용

| 항목 | 요청서 | 코드 (origin/main) |
|---|---|---|
| E1 | 자동 확정하지 않음 | 컨베이어 30 s 유휴 시 자동 확정 |
| A5 | 신규 | 중복 관측은 이미 무시함 (HOLD·알림만 신규) |
| B6, B7 | 신규 / HW | `AutoScaleCycle`에 있음 (런타임 미연결) |
| F3 | HW | V4.4가 위치 신호로 낙하 감지 |
| D3 | HW | 무게 미달은 B2 경로로 이미 검출 |
| C3, C7 | 신규 / HW | `pac_perception` `measure_box`에 시뮬 구현 (`in_full_view`, `valid_ratio`) |
| C8 | `allowed_yaws_rad` 기준 | 배치 방향 집합이라 기준이 될 수 없음 → 90° 배수 기준 |
| A2 / C1 | 라벨 확신도와 크기 확신도 | `RawObservation.confidence` 하나를 함께 씀 |
| A4 | 1회 재판독 | 재판독 없음 (신규) |
| A1·A2·B1·B2 재시도 | 라벨 2회 등 | Base-view 박스당 1회 공유, B2 재측정에 무게 포함 안 됨 |
| B3 | 신규 세분화 | 두 코드가 이미 함께 기록됨 |
| D1 알림 | 운영자 알림 | notes 문구뿐 |
| F1 | EXECUTION_FAIL | `"GRIP_FAIL"` 문자열, L3 |
| F4 | 재측정 후 보정/재적재 | L0/L1/L2 기록, L4 HOLD + 보정 위치 |
| F5 | LOW_SUPPORT / BOX_COLLISION 코드 | 검사 문자열, L4 HOLD |
| INSPECTION 경로 | `on_result` / `REJECT_NG` | 2단계는 `on_observation`에서 처리 |
| SENSOR_UNCERTAIN | Anomaly처럼 표기 | RejectCode임 (pac_runtime에서는 미사용) |

## 5. 재시도 상한과 시간 예산 (가정치)

| 계열 | 상한 | 대상 | 1회 비용(가정) |
|---|---|---|---|
| label | 2 | A3, A4, A6, A7, B3 | 재판독 2.0 s |
| weigh | 2 | B2, B6, B7 | 재계량 2.5 s, 영점 4.0 s |
| frames | 3 | B4, C2, C3, C5, C7, C8, D2 | 재촬영·프레임 0.5 s, 분리 대기 3.0 s, 보조 뷰 1.5 s |

- 박스당 재인식 총 예산 20 s (가정). `SupervisorConfig.operator_time_s`(120 s HOLD)와 별개입니다. 예산을 넘으면 더 요청하지 않고 미해소로 처리하며 notes에 남깁니다.
- 기존 Base-view(박스당 1회)는 이 상한과 별도로 `StateValidator`가 그대로 씁니다. 사용 횟수는 `attempts["base_view"]`에 기록됩니다.
- F1(파지 재시도)은 기존 `ExecutionConfig.max_grip_retries` 규칙을 그대로 따릅니다.

## 6. 1단계 원신호 `PerceptionSignals`

| 필드 | 출처 | 쓰는 코드 |
|---|---|---|
| `label_payload_ok` | 라벨 판독기 (체크섬) | A3 |
| `base_label_sku` | 보조 뷰 / 초입 카메라 판독 | A7 |
| `scale_settled`, `scale_stdev_kg`, `tare_ok` | `AutoScaleCycle` | B6, B7 |
| `frame_sizes` | 다중 프레임 `measure_box` | C2 |
| `in_full_view`, `valid_ratio`, `box_count` | `measure_box` | C3, C7 |
| `arrival_yaw_rad` | `measure_box` pose | C8 |
| `damage_score` | `measure_box` 파손 단서 / 임계값의 최댓값 | D2 |

`signals_from_measurement(measurement, depth_config, frames, **scale_label)`로 만듭니다. 값은 측정값일 뿐이고 판정은 2단계에서만 합니다.

## 7. 사용

```python
from pac_runtime.state_validator import StateValidator
from pac_reinspection import ReinspectionValidator, Reacquired, load_policy

validator = ReinspectionValidator(
    StateValidator(catalog, validator_config, weight_ranges),
    load_policy("config/donghan/reinspection_policy.yaml"),
    reacquire=lambda kind, box_id, attempt: Reacquired(obs=..., signals=...),  # 1단계 재감지, 불가하면 None
    remaining=lambda: state_manager.remaining,                                 # RI-A6
)
verdict = validator.validate(raw_obs, perception, None, signals=signals)
validator.last.ri_codes, validator.last.attempts, validator.last.elapsed_s
```

`reacquire`의 종류: `relabel`, `reweigh`, `rezero`, `recapture`, `frames`, `separate`, `second_view`.

## 8. 런타임 연결 방법 (제안)

- `pac_runtime/core.py` 60행의 `StateValidator(...)`를 `ReinspectionValidator(StateValidator(...), policy, reacquire, remaining)`로 바꾸면 됩니다. 다른 호출부는 `validate`만 씁니다.
- 신호를 넘기려면 `on_observation(obs, base_view, signals=None)`처럼 인자 하나를 늘려야 합니다(태현 님 파일).
- 검증: `tools/donghan/reinspection_drop_in.py` pytest 플러그인으로 기존 런타임 테스트를 교체 상태에서 돌릴 수 있습니다.

## 9. 담당자 확인이 필요한 변경점

**태현 님 (pac_runtime)**
1. `RuntimeCore`에서 `ReinspectionValidator` 사용 여부 (core.py 60행), `on_observation`에 `signals` 인자 추가
2. 정책 결정: 센서 불확실 미해소 → PLAN + δ (이번 기본값), C5·C8·A6·A7·F6 미해소 → INSPECTION
3. `RawObservation.confidence`를 라벨 확신도와 크기 확신도로 나눌지 (A2와 C1이 같은 값을 공유)
4. B2 재측정 시 무게도 다시 잴지, Base-view 1회 제한을 늘릴지
5. 잔여 0인 SKU 도착(A6)이 지금 `StateManager.arrive`에서 아무 경고 없이 통과함 (`Counter`가 -1이 됐다가 지워짐)
6. D3를 DAMAGED로 재분류할지

**재성 님 (pac_perception, Gazebo 스크립트)**
1. `signals.py`(1단계 원신호)를 `pac_perception`에 둘지
2. `AutoScaleCycle`의 안정 여부·표준편차·영점 결과를 런타임에 넘기는 경로 (B6, B7)
3. 다중 프레임, 재촬영, "분리 대기 후 재촬영"(C5)을 Gazebo 셀에서 수행할 수 있는지
4. V4.4 낙하 감지(F3)와 배치 재확인 결과를 런타임 `ExecutionReport`로 연결할지

**공통 (팀 1명 이상 확인, 공통 기준서 26절)**
1. RI 코드를 notes 문자열로 둘지, 나중에 공통 계약(예: `Verdict.reasons`)으로 올릴지
2. 재인식 시간 예산을 `config/default.yaml`에 둘지 모듈 설정에 둘지

## 10. 한계

- C5 판단은 "어떤 SKU와도 크기가 맞지 않고, 두 SKU를 이어 놓은 길이와 맞음"입니다. 우연히 이 조건에 맞는 규격 불량 단일 박스는 기존 SPEC_MISMATCH(PLAN) 대신 INSPECTION으로 갑니다(보수적 선택).
- D2는 미해소면 파손으로 간주합니다(보수적 선택).
- 실제 라벨 판독기, RGB-D, 파손 검출기는 없습니다. 모든 신규 항목은 합성 신호와 스크립트된 재감지로만 검증했습니다.
