# 최종 결정사항 대비 충돌 검토 (2026-10-09)

공통 기준서 v0.3, README의 통합 결정, `docs/jaesung/audit_20261009/ALGORITHM_V3_DRAFT.md` 0장, `docs/taehyeon/interface.md` 8장, 2026-10-09 지시(PPO 미사용)를 기준으로 코드·설정·문서를 대조했습니다.
각 항목의 변경 이유와 검증은 해당 커밋 메시지에 있습니다(`git log`).

## 1. 기준으로 삼은 최종 결정

| # | 결정 | 단일 원본 |
|---|---|---|
| D1 | 목표 로봇 HDR50-22 (2026-10-08) | `config/taehyeon/robot_check.yaml`, `pac_robot_check` |
| D2 | 팔레트 1.10 × 1.10 m, 데크 0.15 m, **데크 위 적재 1.5 m**, 총중량 1000 kg | `config/default.yaml` |
| D3 | `pallet` frame: 데크 모서리 원점, z = 0 데크 윗면, `PalletState.size.z` = 데크 위 높이 | 기준서 8.1 |
| D4 | 박스 기준점 = 회전 후 AABB 최소 모서리, 로봇은 중심으로 변환 | `pac_common.frames` |
| D5 | State Manager 단일 작성자, 측정이 계획과 xy 5 mm·z 3 mm·yaw 1° 이내면 **계획 pose 확정** | `pac_common.StateManager` |
| D6 | "무거운 박스 위 금지" 철회 → SKU별 허용 하중(McKee × 안전계수 4) 누적 검사 (2026-10-09) | `config/taehyeon/candidates.yaml` |
| D7 | 주문 목록(SKU·수량)은 알고 순서만 모름 | `remaining_by_sku` |
| D8 | 4단계는 Rule / Look-ahead, **PPO 미사용** (2026-10-09) | `pac_highlevel` |
| D9 | 버퍼 4칸 | `config/taehyeon/highlevel.yaml` |
| D10 | 작업셀 배치 = Gazebo V4.2 (V4.4는 흡착 그리퍼 추가) | `config/workcell.yaml` |

## 2. 발견한 충돌과 처리

| # | 충돌 | 위치 (이전) | 처리 |
|---|---|---|---|
| C1 | 런타임 상태가 측정 pose를 그대로 저장 (D5 위반), L0 허용오차 z 5 mm·yaw 미검사 | `pac_runtime/state_manager.py`, `VerifyConfig.l0_*` | `pac_common.StateManager`로 통합, L0 = `CommitTolerance` |
| C2 | 철회된 무거운-위-가벼운 규칙이 코드 기본값으로 켜져 있음 (`CandidateConfig()`를 쓰는 곳마다 적용) | `HeavyOnLightConfig.enabled = True` | 기본값 `False`; 규칙 자체 테스트만 명시적으로 켬 |
| C3 | 실행 데모가 1.2 × 1.0 m 팔레트를 강제, HDP160 월드 사용 | `pac_execution` `original_boxes_demo.launch.py` | 패키지 삭제 (7단계 중복) |
| C4 | 6·7단계 Gazebo 셀이 이전 월드(1.2 × 1.0 m, 1.35 m, 로봇 (1.35, 0.15))에 맞춰짐 | `robot_check_gazebo.yaml`, `gazebo_driver.py`, `hdr50_pedestal_workcell.launch.py`, `gazebo_replay.py` | 삭제; `robot_check.yaml`을 V4.2 셀로 다시 씀 (`test_config_consistency`가 `workcell.yaml`·URDF와 대조) |
| C5 | V4.4 사이클 브리지: 적재 높이 1.6 m 허용, 팔레트 250 kg, 상부하중 4 kPa 임시값, 오프라인 후보기 | `ahead_planner_bridge_v44.py` | `default.yaml` 값과 McKee 모델, 팀 5·6·8단계 사용 |
| C6 | 팔레트 frame 규칙 두 가지 (V4.4: 먼 모서리·축 반전 / V4.5: 가까운 모서리·월드 축) | 두 브리지 | `workcell.yaml` `frame_yaw_rad: π` 한 곳 (먼 모서리 원점, 로봇이 놓인 박스 위로 팔을 뻗지 않음). V4.4 기록과 같은 좌표 |
| C7 | 이전 목표 로봇 HDP160-31 잔재 | `pac_robot`, `config/robot.yaml`, `config/eoat.yaml`, `check_hdp160_support.sh` | 삭제 |
| C8 | 기준서 16.6의 구현 위치가 fail-closed 어댑터(`pac_robot.Hdr50_22Adapter`) | 기준서 | `pac_robot_check.RobotFeasibility`로 정정 |
| C9 | EOAT 정의 두 가지 (`pac_eoat` 다중 흡착 TCP 0.22 m / V4.4 흡착컵 TCP 0.06 m) | `pac_eoat`, `pac_bringup` | Gazebo에 실제로 있는 V4.4 흡착컵을 6단계에 사용, `pac_eoat` 삭제 |
| C10 | PPO 정책이 1.35 m 기준으로 학습됨 (D2와 불일치) | `pac_highlevel/models` | PPO 전체 삭제 (D8) |
| C11 | 태현 확정사항 표가 1.35 m·share 모드·MaskablePPO로 남음 | `docs/taehyeon/interface.md` 8장 | 문서 갱신 |
| C12 | 팔레트 교체 시간이 두 설정에 따로 있음 | `SupervisorConfig`, `TimingConfig` | `highlevel.yaml` `timing` 하나 |

## 3. 팀 확인이 필요한 것 (임의로 확정하지 않음)

| # | 내용 | 현재 처리 |
|---|---|---|
| Q1 | V4.4 코드 주석의 "적재 1.6 m 허용 (사용자 결정 2026-10-09)"과 `default.yaml` 1.5 m | 1.5 m(단일 원본)로 통일. 1.6 m가 맞다면 `default.yaml`만 바꾸면 전체가 따라갑니다 |
| Q2 | 팔레트 frame 원점을 먼 모서리로 둠 (C6) | V4.4 사이클과 같은 결과. 기준서 8.1의 "한쪽 아래 모서리"를 구체화한 것 |
| Q3 | 로봇 내려놓기 간격: Gazebo 브리지 20 mm(박스 확대) vs 후보기 `lateral_clearance_m` 4 mm | 브리지 값 유지 (실측 근거 있음). 실제 셀에서 하나로 정할 것 |
| Q4 | 가상 데이터 평가가 팔레트 3종(1.1 × 1.1, 1.2 × 1.0, 1.2 × 0.8)을 돌려 씀 | 강건성 평가용으로 유지, 운영 값은 `default.yaml` |
| Q5 | EOAT 미선정 | 6단계는 V4.4 흡착컵 치수 사용, 질량 15 kg은 가정값 |

## 4. 결정됐지만 아직 구현되지 않은 것 (ALGORITHM_V3_DRAFT 0장)

- 랩 없이 0.3 g 충격 내성 하드 제약 (기둥 전체)
- 팔레트 닫기: 교체 비용 vs 재적재 비용 비교 (현재 `close.mode: dead`)
- 버퍼 1칸 비상용 예약 (현재 4칸 모두 사용)
- 목표 배치도 + 순서 시나리오를 입고마다 갱신하는 계획 구조
- 계량 구간 상류 이동, 상류 CCTV 박스 정보 활용

이 항목들은 알고리즘 개발 범위라 이번 정리에서는 손대지 않았습니다.
