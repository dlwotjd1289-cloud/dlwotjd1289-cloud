# 문서 주장 대장 (2026-10-09 감사)

등급: **VERIFIED** 이번 재실행으로 확인 · **LOG-ONLY** 기존 로그로만 확인 · **CLAIM-ONLY** 근거 파일 없음 · **CONTRADICTED** 실행/코드가 주장과 다름 · **STALE** 실제보다 뒤처진 기술
경로: `I` = `~/AHEAD/pac2026_integrated`, `S` = `~/AHEAD/pac2026_hdr50_proxy_scaffold`, `A` = 이 감사 폴더

| # | 출처 | 주장 | 등급 | 근거 |
|---|---|---|---|---|
| C01 | `I/README.md:41` | `pytest -q` → 307 passed, 2 skipped | CONTRADICTED (경미) | 303 passed, 2 skipped (`A/01_tests/integrated_pytest.txt`). 생성기 테스트 8개는 별도 실행 |
| C02 | `I/README.md:107`, 커밋 6fda793 | ROS 서비스 Humble 빌드·호출 검증 | CLAIM-ONLY | 커밋 메시지뿐, 빌드/호출 로그 없음. `I/ros2_ws/install` 없음. `reports/donghan/verification.json`은 `ros2: NOT_RUN`. ROS 테스트 0개 |
| C03 | `I/README.md:18` 흐름도 | 메인 흐름에 "AI Top-K" | CONTRADICTED | 기본 `model_path=""` (`pac_planning/launch/placement_planner.launch.py:11`) → 모델 없이 휴리스틱. `highlevel.yaml` `value_provider: proxy` |
| C04 | `I/reports/donghan/benchmark.json` | current-only 35.3% < AHEAD 44.4% (4 ep) | CONTRADICTED (재현 실패) | 같은 seed·인자로 재실행 시 current = AHEAD = 44.4% (`A/03_planner/experiment_default/benchmark.json`) |
| C05 | 같은 파일 | AI ranking = AHEAD (44.4%, 21 ms) | CONTRADICTED | 재실행 4 ep: ranking 38.1%. 20 ep: ranking 36.1% vs AHEAD 40.6%, ranking은 current와 차이 없음 (3/1/16, p=0.63) (`A/03_planner/experiment_holdout20_paired.txt`) |
| C06 | `I/reports/donghan/training_report.json` | holdout Top-4 recall 87.5% | CONTRADICTED (재현 실패) | 같은 seed 재실행 75.0% (16 query). 20그룹 확대 시 88.75% (80 query) |
| C07 | 같은 계열 | 미래 예측(AHEAD)이 현재만 보는 방식보다 낫다 | VERIFIED (조건부) | 20 ep 짝비교 7승 0패 13무, p=0.016. **단 ReferenceBackend·토이 SKU 4종·8박스**. 팀 backend에서는 미검증 |
| C08 | `I/reports/donghan/team_generator_training_20261008.json` | 학습 모델 test recall 1.0 | VERIFIED, 단 의미 없음 | 재실행도 1.0, 휴리스틱도 1.0·regret 0 (모델 0.0015). 테스트 2 시나리오, 배치 결과 세 모드 완전 동일 (`A/03_planner/team_training/`) |
| C09 | `I/docs/taehyeon/VALIDATION.md` §4 | 계획시간 평균 765 / p95 1073 / max 1300 ms, 7/23 예산 초과 | VERIFIED (이 PC에서 더 빠름) | 평균 574 / p95 913 / max 1004 ms, 1/24 초과 (32코어). 예산 끄면 p95 1.9 s, max 2.9 s (`A/03_planner/planner_benchmark_*.json`) |
| C10 | 같은 문서 | 후보 recall vs 오라클 97.4% | VERIFIED | 재실행 100% (76 장면), p95 17 ms (`A/03_planner/oracle_benchmark.json`) |
| C11 | `I/docs/taehyeon/reports/physics_solid.json` | 통과 후보 82/82 물리 안정 | VERIFIED | 81/81. 단 안정성으로 **거절한 후보 97개 중 39개(40%)도 실제로 안정** → 마스크 과보수 (`A/03_planner/physics_solid.json`) |
| C12 | `I/docs/taehyeon/highlevel.md` §5 | Rule 4.09 < PPO 4.25 팔레트, Rule 권장 | VERIFIED (경향 동일) | 새 데이터: Rule 4.14, PPO 4.16, Greedy 4.34, 버퍼없음 5.01. PPO vs Rule 11승 12패 4무 → 차이 없음 (`A/03_planner/highlevel_eval_test.json`) |
| C13 | 같은 표 | 채움률 25.2% | VERIFIED (더 낮음) | 21.7%. 80박스 부피 합은 평균 팔레트 **0.99개분**인데 **5.0개** 사용 (`A/03_planner/highlevel_pallet_lower_bound.txt`) |
| C14 | `I/config/taehyeon/candidates.yaml:33` | `share` 모드 (주석: per_box가 미션 문구) | CONTRADICTED vs 공식 미션 | 10 kg 박스를 5.5 kg 두 개 위, 20 kg을 10.5 kg 두 개 위, 5.4 kg을 5.0 kg 위 → 모두 통과 (`A/04_probes/probe_m18_m19.json`) |
| C15 | `I/docs/donghan/requirements_traceability.md` | 로봇 추정시간 분리 | CONTRADICTED | `robot_time_sec_by_candidate`를 채우는 코드 없음 → 항상 GEOMETRIC_PROXY (`pac_planning/features.py:260`). 상위 정책은 `place_time_s: 8.0` 가정 |
| C16 | 같은 문서 | 미션 ④ 이상 박스 대응 | CONTRADICTED (부분) | 가상 세계에서 파손 박스 위에 안 쌓기(0 N)만. 검출기·제외·별도 배출 경로 없음. `TRACKING_LOST`는 enum만 존재 |
| C17 | `I/ros2_ws/src/pac_robot/pac_robot/hdr50_22_sim_adapter.py:37` | HDR50-22 가반하중 50 kg | 값만 존재 | 어디서도 검사 안 함. 80 kg 박스도 하드마스크 통과 (`A/04_probes/probe_m18_m19.json`). 데이터셋 최대 30 kg라 실제 위반은 없음 |
| C18 | `S/README_V44.md:88` | 탑뷰 CCTV `cctv_pick_topview`, `cctv_pallet_topview` | CONTRADICTED | V4.4 월드 sdf의 카메라 센서는 `cctv_camera_1` 하나 (+손목 카메라, URDF) |
| C19 | `S/README_V44.md:50` | full cycle 3/3 PASS | LOG-ONLY (조건부) | 팔레트 중앙 단일 박스 기준. 생성기 시나리오(24개)는 9회 실행 중 완주 0회 (`A/05_workcell_logs/generator_cycle_runs.json`) |
| C20 | 이관 패키지 `02_LATEST_STATUS_AND_NEXT_ACTIONS.md` | Vacuum attach/detach 미완성, Scale weight event 미구현 | STALE (실제가 더 앞섬) | 흡착·계량 모두 Gazebo 로그 존재 (`S/logs/v43_e2e`, `S/logs/v44_e2e`) |
| C21 | `I/README.md` 남은 일 3 | PPO는 1.35 m 기준 학습 | VERIFIED (문서 정직) | 테스트 헬퍼도 `PALLET = Size3D(1.1, 1.1, 1.35)` (`I/tests/taehyeon/th_helpers.py`) |
| C22 | 이관 추적표 M11/M15 | 미완성 | VERIFIED (여전히 부분) | V4.5 IK 거절 0/121 → 도달성 검사가 실제로 걸러낸 적 없음. 충돌 검사 없음 (`pick_place_plan_v45.py:6`) |
