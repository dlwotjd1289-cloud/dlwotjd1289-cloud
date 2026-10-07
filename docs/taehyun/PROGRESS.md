# 태현 파트 진행 기록 (5-1 후보 생성 · 5-2 Hard Mask · 가상데이터)

> 이 파일은 작업이 중단되더라도 그대로 이어서 진행할 수 있도록 매 단계 갱신합니다.
> "이어서 해"라고 하면 아래 **다음 할 일**부터 진행합니다.

## 재개 방법

```bash
uv venv --python 3.10 .venv && . .venv/bin/activate   # 또는 python3.10 -m venv .venv
pip install "numpy>=1.23,<3" "PyYAML>=6,<7" "pytest>=7,<9"
scripts/taehyun/fetch_team_deps.sh        # 팀원 코드(읽기 전용)를 .deps/team 에 추출
python -m pytest -q tests/taehyun
```

## 결정 사항

| 항목 | 결정 | 이유 |
|---|---|---|
| 사용 로봇 | HDR50-22 (문서에만 기록) | 5-1/5-2는 로봇 판정 범위 밖(6단계 담당) |
| 코드 위치 | `ros2_ws/src/pac_candidates` (새 ROS 패키지) | 동한 님 `pac_planning`과 파일 충돌 없음 |
| pac_common | 복사하지 않음. `fetch_team_deps.sh`로 팀 브랜치에서 읽기 전용 추출 | 동한 님 확장 pac_common이 단일 원본, 중복 방지 |
| 연결 계약 | 동한 님 `docs/integration.md`의 v0.2 콜백 그대로 | `generate_candidates(box, state)`, `validate_constraints(box, candidate, state)` |
| 좌표 | target_pose = 회전 후 AABB 최소 모서리, z=0 적재면, PalletState.size.z = 최대 적재 높이 | 동한 님 계약 |
| 최대 높이 1.5m | 팔레트 바닥판(0.15m) 포함으로 해석 → 적재 높이 1.35m (가상데이터 변환 시) | 보수적, 설정 변경 가능 |

## 진행 상태

- [x] 팀 브랜치 분석 (재성: 제너레이터·물리 시뮬레이터, 동한: 5-3~5-6)
- [x] 패키지 골격 + 팀 코드 추출 스크립트
- [x] 5-1 후보 생성 (EMS + Extreme Point, yaw 집합, 80mm 중복 제거)
- [x] 5-2 Hard Mask (경계·겹침·높이·방향·지지율·LBCP+δ·박스 하중·무거운-위-가벼운·팔레트 하중·팔레트 CoG)
- [x] 동한 님 planner와 end-to-end 연결 확인 (scenario_001/002)
- [ ] pytest 단위/통합 테스트
- [ ] 가상데이터 생성기
- [ ] 벤치마크 리포트, 물리 시뮬레이터 교차 검증
- [ ] 문서, Draft PR

## 다음 할 일

1. `tests/taehyun/` 테스트 작성
2. `tools/virtual_data` 가상데이터 생성기

## 검증 로그

| 날짜 | 내용 | 결과 |
|---|---|---|
| 2026-10-07 | 랜덤 40박스 greedy 적재 smoke | 후보 10~115개/박스, 생성 13~40ms, 마스크 사유 정상 |
| 2026-10-07 | 동한 planner + 태현 backend (scenario_001/002) | plan 400~550ms, 7개 시나리오 완료 |
