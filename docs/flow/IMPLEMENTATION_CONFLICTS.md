# 구현 간 충돌 검토 (2026-10-09, 정리 브랜치 기준)

같은 일을 하는 코드가 서로 다른 규칙·값을 쓰는 곳을 모았습니다. 주로 재성 님 Gazebo / PyBullet / v3 프로토타입 코드와
팀 런타임(태현 `pac_runtime`·`pac_highlevel`·`pac_robot_check`, 동한 `pac_planning`, 공통 `pac_common`)의 비교입니다.
최종 결정사항과의 충돌은 [DECISION_REVIEW](DECISION_REVIEW.md)에 따로 있습니다. **아래 항목은 코드를 바꾸지 않고 기록만 했습니다.**

심각도: **높음** = 지금 실행되는 경로에서 결과가 달라짐 · **중간** = 같은 입력에 다른 판정·값 · **낮음** = 설계·표시 수준

## 요약

| # | 영역 | 충돌 | 심각도 |
|---|---|---|---|
| C1 | 4단계 행동 결정 | Gazebo 사이클 스크립트가 `pac_highlevel`과 다른 규칙으로 버퍼·교체·보류를 직접 결정 | 높음 |
| C2 | 8단계 재고 | Gazebo 사이클이 버퍼 박스를 "아직 안 온 재고"로 셈 | 높음 |
| C3 | 8단계 상태 크기 | V4.4 commit이 여유 20 mm를 더한 크기를 ACTUAL 상태에 기록 | 중간 |
| C4 | 6단계 가반하중 | 그리퍼 정격 30 kg(Gazebo) vs 가반하중 50 kg − 그리퍼 15 kg = 35 kg(6단계) | 중간 |
| C5 | 셀 모델: 버퍼 | 버퍼 2칸(선반 1개, 칸 크기 제한) vs 4칸(팔레트 양쪽, 크기 제한 없음, 이동시간 4~5 s) | 중간 |
| C6 | 1·2단계 인식·검증 | Gazebo 경로는 `StateValidator`를 거치지 않고 SKU를 바닥 면적으로만 맞춤; 인식 구현 3갈래 | 중간 |
| C7 | 7단계 합격 기준 | "놓기 성공·안정" 판정 값이 파일마다 다름 | 중간 |
| C8 | 하중 모델 | 팀 McKee ÷ 4 vs 재성 v3 BCT × 현장계수 ÷ 1.5 | 중간 (설계) |
| C9 | 무거운-위-가벼운 | 재성 프로토타입 설정이 철회된 규칙을 켬 | 중간 (실험 결과 해석) |
| C10 | 미래 박스 정보 가정 | 보이는 5개의 무게까지 앎 vs 계량 3개 + 카메라 2개(무게 모름) + 미입고 샘플링 | 낮음 (설계) |
| C11 | 4단계 Look-ahead 설계 | 마감·재적재·비상칸·평가 단위가 다름 | 낮음 (설계) |
| C12 | 운반 경로·높이 | 6단계는 운반을 보지 않고, V4.4 브리지만 TCP ≤ 1.95 m 확인 | 낮음 |
| C13 | 적재 높이 표시 | 사이클 스크립트가 "허용 1.6 m"를 출력, 실제 계획은 1.5 m | 낮음 |
| C14 | 시간 값 | 같은 가정값(8 / 4 / 12 / 60 s)이 두 설정 파일에, 6단계 `cycle_time` 추정과 별도 | 낮음 |
| C15 | 좌표 변환 | 팔레트 → 시뮬레이터 중심 좌표 변환 함수가 여러 벌 | 낮음 |
| C16 | Gazebo 경로 생성 | V4.4/V4.5 단일 박스 경로가 6 cm 흡착컵을 빼고 플랜지 기준으로 계산 | 높음 (확인 필요) |
| C17 | 팔레트 규격 원본 | ROS 런타임이 팔레트를 `default.yaml`이 아닌 주문 파일에서 읽고, 예시 주문이 이전 규격(1.2 × 1.0 m, 1.35 m) | 높음 |
| C18 | SKU 대표 무게 | ROS 런타임은 무게 범위 최대값, 가상 데이터·Gazebo 브리지는 중간값 | 높음 |
| C19 | 학습 모델 | 저장소 모델 3개 모두 현재 팀 후보기와 계약이 맞지 않음, 데모는 검사 없이 사용 | 중간 |
| C20 | 5-② 검사기 3번째 | `pac_planning.physics` (지지율 0.80) 가 테스트에서만 쓰임 | 중간 |
| C21 | 마찰 가정 | Gazebo 0.9 / PyBullet 0.64·0.72 / 재성 v3 계산 0.4 | 중간 |
| C22 | 컨베이어 모델 | 축적 6개(태현 실시간) vs 축적 없음·정지(재성 v3) vs 한 박스씩(Gazebo), 도착 6 s < 로봇 8 s | 중간 |
| C23 | 계량 검증 | V4.3은 생성기 정답 무게 ± max(0.35 kg, 7 %), 2단계는 SKU 범위 ± 8 % | 낮음 |
| C24 | 공통 값 중복 | 지지율 0.70, 1000 kg, 데크 0.15 m가 여러 파일에 같은 값으로, 일치 검사 일부만 | 낮음 |
| C25 | 안정성 가속도 | 재성 v3 안정성 점수는 0.2 g, 하드 검사는 0.3 g | 낮음 (설계) |
| C26 | 이름·미사용 값 | `SkuSpec` 두 의미, 팔레트 ID 세 가지, `PlannerConfig.min_cog_margin` 미사용 | 낮음 |

---

## C1. 4단계 행동 결정 (높음)
- **재성**: `scripts/run_generator_cycle_v44.sh` 134~138행(정책), 355~380행(실행). 셸 스크립트가 행동을 직접 정합니다.
- **팀**: `pac_highlevel` `HighLevelDecider` + `RulePolicy` (`rules.py`), `config/taehyeon/highlevel.yaml`.

| 항목 | V4.4 사이클 | `pac_highlevel` |
|---|---|---|
| 버퍼에 맡기기 | 현재 팔레트에 자리가 **없을 때만** | 좋은 자리(지지율 ≥ 0.95)가 없으면 맡길 수 있음 |
| 버퍼에서 꺼내기 | 적재 후마다 모든 버퍼 박스 재계획, 자리 나면 즉시 | 오래 기다린 박스 우선, 2 cm 이상 낮게 들어가는 버퍼 박스 우선 |
| 팔레트 교체 | 버퍼가 차거나 박스가 칸보다 클 때 | `close.mode: dead` (앞으로 올 박스 중 놓을 곳 있는 게 없을 때) |
| 재적재 | 없음 | 놓을 곳 없을 때 BFS, 최대 2회 이동 |
| 못 놓는 박스 | 보류 구역 (`hold_box`) | `REJECT_NG` |

같은 박스 순서에서 Gazebo와 가상 셀이 다른 결정을 냅니다. 이전 정리 문서의 "Gazebo는 PLACE_CURRENT만"은 잘못된 설명이었습니다.
**제안**: 사이클이 박스마다 `HighLevelDecider.decide()`를 부르고 결과만 실행. 재성 님 확인 필요.

## C2. 버퍼 박스를 미입고 재고로 셈 (높음)
- **재성**: `run_generator_cycle_v44.sh` `remaining_json()` (156~167행)이 "앞으로 올 박스 + **버퍼에 있는 박스**"를 `remaining_by_sku`로 넘깁니다.
- **팀**: 기준서 11장 · `pac_common.StateManager`는 버퍼 박스를 `tracked_boxes`(status `BUFFERED`)로 두고, `remaining_by_sku`는 개별 ID가 없는 미입고 수량만입니다.
- **영향**: 5-③ 미래 시나리오가 버퍼 박스를 "아직 안 온 박스"로 다시 뽑아, 같은 박스를 이중으로 계산할 수 있습니다(CONTRIBUTING "현재/preview/버퍼를 잔여 수량에 중복 계수하지 않음" 위반).

## C3. ACTUAL 상태에 여유 크기를 기록 (중간)
- **재성 (현재 브리지에도 유지)**: `scripts/ahead_planner_bridge_v44.py` `commit`은 계획대로 놓이면 플래너 크기(실제 + 20 mm)로, 아니면 실제 크기로 기록합니다.
- **팀**: `PlacedBox.size`는 실제 박스 크기(기준서 10장). 후보기 쪽 간격은 `candidates.yaml` `uncertainty.lateral_clearance_m` 4 mm로 따로 둡니다.
- **영향**: 같은 상태 파일 안에서 박스 크기 의미가 섞이고, 하중·무게중심 계산에 부피가 실제와 다르게 들어갑니다. 간격 20 mm vs 4 mm는 [DECISION_REVIEW](DECISION_REVIEW.md) Q3과 같은 문제입니다.

## C4. 가반하중 기준 (중간)
- **재성**: `scripts/moveit_pick_place_v44.py` 78행 `GRIPPERS = {"pad": 30, "pad_xl": 45, "pad_fork": 60}`. 사이클은 기본 `pad` 30 kg를 넘으면 보류.
- **팀**: `config/taehyeon/robot_check.yaml` 정격 50 kg − 그리퍼 15 kg(가정) = 박스 35 kg까지 통과.
- **영향**: 30~35 kg 박스는 계획(6단계 통과)과 실행(보류)이 엇갈립니다. 그리퍼별 한계를 6단계 설정 하나로 모아야 합니다(EOAT 미선정과 연결).

## C5. 버퍼 셀 모델 (중간)
- **재성 (Gazebo)**: `config/workcell.yaml` `buffer_rack` (1.45, −0.55) 선반 하나에 2칸, 칸 크기 0.37 × 0.56 m 이하만 (`buffer_bay_slot`).
- **팀**: `highlevel.yaml` `buffer.slots: 4`, "팔레트 양쪽 선반", 칸별 이동시간 4 / 4 / 5 / 5 s, 크기 제한 없음. 재성 v3 설계는 4칸 중 1칸 비상용 예약.
- **영향**: 4단계가 Gazebo에 없는 칸이나, 칸에 안 들어가는 박스를 버퍼로 고를 수 있습니다.

## C6. 인식·검증 경로 (중간)
| 구현 | 위치 | 하는 일 |
|---|---|---|
| 태현 | `pac_runtime/perception.py` + `state_validator.py` | 라벨·무게(± 8 %)·크기 교차검증, 파손·인지 실패 분류 |
| 동한 | `pac_perception/depth_measurement.py`, `sku_resolver.py`, `pac_reinspection` | 깊이 영상 측정, 라벨 실패 시 SKU 추정, 재인식 정책 (런타임 미연결) |
| 재성 | `scripts/box_perception_v44.py`, `ahead_planner_bridge_v44.py` `match_sku` | RGB 카메라 위치·자세, 높이는 SKU 값, SKU는 바닥 면적 ± 30 mm로 맞춤 |

Gazebo 경로는 계량값과 카메라 결과를 검증 없이 그대로 씁니다(무게 불일치·라벨 불일치 판정 없음). SKU 추정 방법도 동한(`sku_resolver`)과 재성(`match_sku`)이 다릅니다.

## C7. 놓기 성공·안정 판정 값 (중간)
| 위치 | 기준 |
|---|---|
| 공통 확정 규칙 `CommitTolerance` | xy 5 mm, z 3 mm, yaw 1° (계획 pose 확정) |
| 런타임 사후 검사 `runtime.yaml` `verify` | L1 xy 20 mm / z 10 mm, 겹침·돌출 2 mm, 지지율 ≥ **0.6** |
| 후보 Hard Mask `candidates.yaml` | 지지율 ≥ **0.70** |
| Gazebo 실행 `run_pick_place_v44.py` | xy 30 mm, z 20 mm, 기울기 5° |
| PyBullet 폐루프 `verify_stack_bullet_v45.py` (재성) | 기울기 5°, 기존 스택 이동 5 mm |
| PyBullet 재생 `physics_replay.py`, `physics_check.py` (태현) | 이동 10 mm, 기울기 2° |

역할이 다른 값도 있지만, **"물리적으로 안정"의 기준**(기울기 2° vs 5°, 이동 5 mm vs 10 mm)과 **지지율**(0.6 vs 0.70)은 같은 의미인데 값이 다릅니다. 같은 팔레트가 한쪽 검증에서는 통과, 다른 쪽에서는 실패할 수 있습니다.

## C8. 박스 허용 하중 모델 (중간, 설계)
- **팀**: McKee(ECT 5 kN/m, 두께 3 mm) ÷ 안전계수 4 (`candidates.yaml` `load_model`, 태현 확정 #2).
- **재성 v3**: BCT × 습도 × 보관기간 × 0.8 ÷ 1.5, 무게 등급별 판지 (`tools/prototypes/site_params/capacity.py`, `COST_FUNCTIONS.md`). 프로토타입은 `build_catalog`를 바꿔치기해서 씁니다.
- 결과 비교 시 하중 모델이 다르면 팔레트 수 차이가 알고리즘 차이인지 구분되지 않습니다.

## C9. 철회된 규칙이 실험 설정에서 켜짐 (중간)
- `tools/prototypes/lookahead/configs/base.yaml` 35행 `heavy_on_light.enabled: true` (그 밖에 `perbox*.yaml`). `lookahead.py` 머리 주석도 "per_box heavy-on-light"라고 적혀 있습니다.
- 재성 님 비교 문서(`COMPARE_TAEHYEON_N5.md`)는 "무거운 것 위 금지: 끔"이라고 적어 문서와 설정이 다르고, 팀 결정(2026-10-09 철회)과도 다릅니다. 재성 v3 실험 수치(팀 대비 팔레트 −13 %)는 이 규칙을 켠 기준일 수 있습니다.

## C10. 미래 박스 정보 가정 (낮음, 설계)
- **태현**: `config/taehyeon/environment.yaml` `visible_boxes: 5`, 보이는 5개의 크기·무게·순서를 안다고 가정. 미입고 박스는 탐색에 넣지 않음.
- **재성 v3**: 계량 완료 3개(무게 앎) + 상류 카메라 2개(SKU만, 무게는 범위에서 뽑음) + 미입고 수량으로 순서 4가지 샘플링.
- 실제 셀 구성(계량 위치, 카메라 수)이 정해지면 하나로 맞춰야 합니다.

## C11. 4단계 Look-ahead 설계 (낮음, 설계)
| 항목 | 태현 `pac_highlevel/lookahead.py` | 재성 `tools/prototypes/lookahead/` · `ALGORITHM_V3_3.md` |
|---|---|---|
| 평가 단위 | 팔레트 부피 비율 | 시간 단위 비용 J, 평균 70 % + 최악 30 % |
| 팔레트 마감 | 막힌 팔레트 규칙 | 교체 vs 재적재 J 비교 |
| 재적재 | 놓을 곳 없을 때만 | 순이득 > θ면 매번 후보 (기회형, "사용자 결정 2026-10-09") |
| 버퍼 | 4칸 모두 사용 | 1칸 비상용 예약 |
| 안정성 | 정적 검사 | 기둥 전체 0.3 g 검사(선택) |

재성 님 비교 문서는 태현 코드를 기반으로 재성 쪽 요소를 하나씩 옮기자고 제안합니다. 운영 경로는 태현 코드 하나이므로 지금은 결과 충돌이 없지만, "기회형 재적재"와 "비상칸 예약"은 사용자 결정이라고 적혀 있어 팀 규칙과 다릅니다.

## C12. 운반 경로 (낮음)
`pac_robot_check`는 놓는 자세와 수직 접근만 봅니다. V4.4 브리지는 컨베이어 → 팔레트 운반 높이(TCP z ≤ 1.95 m, 지나가는 박스 위 10 cm)를 따로 확인해 후보를 더 거릅니다. 가상 셀·ROS 런타임에는 이 검사가 없어서 Gazebo에서만 탈락하는 후보가 생길 수 있습니다.

## C13. 적재 높이 표시 (낮음)
`run_generator_cycle_v44.sh` 138·144행이 `PAC_STACK_MAX_M`(기본 1.6 m)을 "허용"으로 출력하지만, 브리지는 이 변수를 더 이상 읽지 않고 `config/default.yaml` 1.5 m만 씁니다. 1.6 m 여부는 [DECISION_REVIEW](DECISION_REVIEW.md) Q1.

## C14. 시간 값 (낮음)
- 놓기 8 s · 버퍼 이동 4 s · 재적재 12 s · 교체 60 s가 `highlevel.yaml` `timing`과 `environment.yaml` `robot`에 같은 값으로 두 번 있습니다(`environment.yaml`이 덮어씀).
- `pac_robot_check.cycle_time`은 관절 거리로 사이클 시간을 따로 추정하고, 4단계는 고정 8 s를 씁니다. 재성 v3 설계는 고정 상수를 시간 모델 하나로 바꾸자고 합니다.

## C15. 좌표 변환 중복 (낮음)
팔레트 frame(모서리 원점) → PyBullet 시뮬레이터(팔레트 중심 원점, 박스 중심) 변환이 `tools/runtime/scripts/physics_replay.py` `centre`, `tools/realtime/live_sim_bridge.py` `sim_center`, `scripts/mission_bridge_v45.py`(`gazebo_pallet(top_center_world=(0,0,0))`)에 따로 있습니다. 앞의 두 개는 가상 셀 결과용이라 `workcell.yaml` `frame_yaw_rad`를 쓰지 않습니다. Gazebo 결과를 이 함수들로 뷰어에 보내면 팔레트가 180° 돌아 보입니다.

---

# 2차 검토 (전체 코드 대조)

## C16. Gazebo 경로가 흡착컵 길이를 빼고 계산 (높음, 확인 필요)
- `scripts/pick_place_plan_v44.py` 21·69행, `pick_place_plan_v45.py` 26행: 집기·놓기 목표를 **플랜지**(`flange_link`)가 박스 윗면 5 mm 위에 오도록 잡습니다(`grip_gap_m`).
- V4.5 미션(`run_mission_v45.sh`)과 `run_pick_place_v44.sh`가 띄우는 V4.4 로봇(`hdr50_pedestal_gripper.urdf.xacro`)에는 플랜지 끝에 **0.06 m 흡착컵**이 있습니다.
- 같은 셀을 다루는 MoveIt 실행(`moveit_pick_place_v44.py`, `TCP = "suction_tcp"`)과 6단계(`robot_check.yaml` `tcp_offset_m: 0.06`)는 컵을 반영합니다.
- 계산대로라면 컵이 박스 안으로 약 55 mm 들어갑니다. V4.2(컵 없음) 시절 경로를 V4.4 월드에서 그대로 쓰는 것으로 보이며, Gazebo에서 실제 거동 확인이 필요합니다.

## C17. 팔레트 규격 단일 원본 위반 (높음)
- `pac_runtime/order.py` `cell_from_order`는 팔레트 크기·높이·하중을 **주문 JSON**에서 읽습니다. ROS `runtime_node`와 `runtime.launch.py`가 이 경로입니다.
- 예시 `config/taehyeon/example_order.json`과 `order.py` 설명 주석은 `size_m: [1.2, 1.0, 1.35]` (이전 규격). 테스트도 이 파일을 씁니다.
- 기준서 19장은 팔레트 값을 `config/default.yaml` 한 곳에서만 쓰라고 정했고, `tests/test_config_consistency.py`는 이 파일을 검사하지 않습니다.

## C18. SKU 대표 무게 (높음)
- `pac_runtime/order.py` `cell_from_order(..., nominal_weight="max")`: 아직 안 본 박스의 무게를 **범위 최대값**으로 둡니다(ROS 런타임).
- `tools/virtual_data` `catalog.nominal_weight: midpoint`(가상 데이터 평가, 4단계 성능 수치), `ahead_planner_bridge_v44.py` 62행(Gazebo 사이클)은 **중간값**.
- 미래 시나리오의 하중·무게중심·팔레트 총중량 판단이 런타임과 평가에서 다르게 나옵니다. 평가 수치를 런타임 성능으로 옮겨 말할 수 없습니다.

## C19. 학습 모델과 후보기 계약 (중간)
| 모델 | 학습 기준 | 지금 상태 |
|---|---|---|
| `models/dual_head_ranker.json` | 삭제된 오프라인 후보기 (`REFERENCE_FULL_SINGLE_SUPPORT`), horizon 3 · 시나리오 7 | README 데모(`pac_planning.demo --model ...`)가 계약 검사 없이 사용 |
| `models/candidate_runtime_v2.json` | `pac_candidates` 소스 해시 고정, horizon 2 · 시나리오 3 | 지금 `pac_candidates`와 해시·설정 불일치 → `plan_with_backend`가 `MODEL_BACKEND_MISMATCH`로 거부. **정리 전(d995caa)에도 이미 불일치** |
| `models/team_fd683e56_smoke.json` | 위와 같음 (스모크용) | 같음 |

`config/default.yaml` 플래너 설정(horizon 3, 시나리오 7)과 v2 모델(2, 3)도 다릅니다. 지금 팀 후보기로 쓸 수 있는 학습 모델은 없고, 런타임은 휴리스틱(`NO_MODEL_HEURISTIC`)으로 동작합니다.

## C20. 세 번째 5-② 검사기 (중간)
`ros2_ws/src/pac_planning/pac_planning/physics/` (`PalletPhysicsEngine`, `min_support_ratio 0.80`)는 V1 작업셀용 분석 물리로, `tests/workcell/test_pallet_physics_v1.py`에서만 쓰입니다. 팀 기준(0.70, `pac_candidates`)과 값이 다른 같은 역할의 코드라 삭제 또는 `pac_candidates`로 흡수 대상입니다.

## C21. 마찰 계수 가정 (중간)
| 위치 | 값 |
|---|---|
| Gazebo 박스 (`make_box_sdf_v44.py`, `scale_auto_box_5kg_v43.sdf`) | μ 0.9 |
| PyBullet (`config/ahead_simulator.yaml`) | 박스-박스 0.64, 박스-팔레트 0.72 (측정값 아님) |
| 재성 v3 안정성 계산 (`tools/prototypes/lookahead/lookahead.py` 230행) | μ 0.4 가정 |

같은 적재가 시뮬레이터마다 다르게 미끄러지거나 버팁니다. 0.3 g 기준(재성 v3)은 μ 가정에 바로 영향을 받습니다.

## C22. 컨베이어 모델 (중간)
- **태현 실시간** (`pac_highlevel/realtime.py`): 6 s마다 1개 도착, 컨베이어에 최대 6개까지 쌓이며 상류에서 대기.
- **재성 v3** (`ALGORITHM_V3_3.md` 7-2장): 축적 컨베이어 가정 없음(`queue_capacity` 0), 못 놓으면 컨베이어 정지 → 단계별 대응.
- **Gazebo 사이클**: 한 박스씩 계량·적재(다음 박스는 로봇 작업 중 미리 계량).
- 도착 간격 6 s(`environment.yaml`)가 로봇 1회 8 s(`highlevel.yaml`)보다 짧아 처리량·대기시간 평가가 컨베이어 가정에 크게 좌우됩니다.

## C23. 계량 검증 기준 (낮음)
V4.3 (`scale_cycle_core_v43.py` 45·73행)은 생성기의 정답 무게와 비교해 ± max(0.35 kg, 7 %)를 넘으면 실패, 2단계 `StateValidator`는 SKU 무게 범위와 ± 8 %로 비교합니다. 실제 셀에서는 정답 무게를 알 수 없으므로 2단계 방식이 기준입니다.

## C24. 공통 값 중복 (낮음)
값은 지금 같지만 원본이 여러 개입니다.
| 값 | 위치 |
|---|---|
| 지지율 0.70 | `default.yaml` `constraint`, `candidates.yaml` `constraints` (후보기는 `default.yaml`을 읽지 않음) |
| 팔레트 1000 kg | `default.yaml`, `candidates.yaml` `default_pallet_max_weight_kg`, `virtual_data.yaml`, `environment.yaml`, 주문 JSON |
| 데크 0.15 m | `default.yaml`, `workcell.yaml`, `robot_check.yaml` (이 둘은 일치 검사 있음) |

## C25. 안정성 가속도 (낮음, 설계)
재성 v3 프로토타입의 안정성 점수는 중력을 0.2 g 기울여 계산하고(`TILT_G = 0.2`), 하드 검사 목표는 0.3 g(`a_target_g`)입니다. 0.3 g 기준이 확정되면 점수 쪽도 맞춰야 합니다.

## C26. 이름·미사용 값 (낮음)
- `SkuSpec`: 생성기(`ahead_dataset_generator.config`, 무게 범위)와 `pac_common.planning`(대표 무게 + 허용 하중)이 다른 필드를 가진 같은 이름.
- 팔레트 ID: `P001`(`default.yaml`), `PALLET-01`(`StateManager.for_order`), 주문 파일 `id_prefix`.
- `PlannerConfig.min_cog_margin`(`default.yaml` 0.02)은 코드에서 쓰이지 않습니다.

## 2차 검토에서 충돌이 없었던 것
공통 자료형 재정의(`BoxState`, `RejectCode` 등 없음), 박스 기준점(AABB 최소 모서리), 허용 yaw(0 / 90°), HDR50-22 관절 표·HOME 자세, Gazebo 월드 이름, 카메라 토픽, `default.yaml` 팔레트 값과 생성기·PyBullet·작업셀 설정(일치 검사 통과).

---

## 처리 순서 제안
1. **C1·C2·C5** (함께): Gazebo 사이클을 `HighLevelDecider` + `StateManager`로 구동하고, 버퍼 칸 수·위치·크기 제한을 셀 설정 하나로.
2. **C3·C4·C7**: 상태에는 실제 크기, 간격은 후보기 설정으로 / 그리퍼별 가반하중을 6단계 설정으로 / 안정 판정 기준값 하나로.
3. **C6**: Gazebo 관측도 `StateValidator`를 거치게 연결, SKU 추정 방법 하나로.
4. **C8~C11**: 하중 모델·정보 가정·마감/재적재 방식을 팀 회의에서 결정한 뒤 실험 설정을 맞춰 다시 비교.
5. **C12~C15**: 위 작업과 함께 정리.
