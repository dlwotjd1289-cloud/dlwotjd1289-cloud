# 태현 파트 진행 기록 (5-① 후보 생성 · 5-② Hard Mask · 가상데이터)

> 작업이 중단되더라도 이어서 진행할 수 있도록 단계마다 갱신합니다.
> "이어서 해"라고 하면 아래 **다음 할 일**부터 진행합니다.

## 재개 방법

```bash
uv venv --python 3.10 .venv && . .venv/bin/activate      # 또는 python3.10 -m venv .venv
pip install "numpy>=1.23,<3" "PyYAML>=6,<7" "pytest>=7,<9" "pybullet>=3.2.6,<4" "networkx>=2.8,<4"
scripts/taehyun/fetch_team_deps.sh                       # 팀원 코드(읽기 전용)를 .deps/team 에 추출
python -m pytest -q tests/taehyun                        # 119 passed
scripts/taehyun/run_validation.sh                        # 전체 검증 리포트 재생성
```

PR: https://github.com/yang8988/pac-mission1-shared/pull/1 (Draft, 브랜치 `claude/pensive-pasteur-dwbu3g`)

## 결정 사항

| 항목 | 결정 | 이유 |
|---|---|---|
| 사용 로봇 | HDR50-22 (문서에만 기록) | 5-①/5-②는 로봇 판정 범위 밖(6단계 담당) |
| 코드 위치 | `ros2_ws/src/pac_candidates` (새 ROS 패키지) + `tools/virtual_data` | 동한 님 `pac_planning`과 파일 충돌 없음 |
| pac_common | 복사하지 않음. `fetch_team_deps.sh`로 팀 브랜치에서 읽기 전용 추출 | 동한 님 확장 pac_common이 단일 원본, 중복 방지 |
| 연결 계약 | 동한 님 `docs/integration.md`의 v0.2 콜백 그대로 | 공통 계약 변경 없음 |
| 좌표 | target_pose = 회전 후 AABB 최소 모서리, z=0 적재면, PalletState.size.z = 최대 적재 높이 | 동한 님 계약 |
| 최대 높이 1.5 m | 팔레트 목재(0.15 m) 포함 → 적재 1.35 m | **태현 확정 (2026-10-07)** |
| 무거운-위-가벼운 | `per_box` (미션 문구 그대로) | **태현 확정 (2026-10-07)** |
| 루트 공용 파일 | 만들지 않음 (`.gitignore`도 폴더별로 둠) | 재성 님 브랜치의 루트 파일과 충돌 방지 |

## 진행 상태

- [x] 팀 브랜치 분석 (재성: 제너레이터·물리 시뮬레이터, 동한: 5-③~⑥)
- [x] 5-① 후보 생성 (EMS + Extreme Point, yaw, 80 mm 중복 제거, likely-valid-first 순서)
- [x] 5-② Hard Mask 12개 검사 + ConstraintEvidence
- [x] 동한 님 planner end-to-end 연동 + EMS 공급
- [x] 성능 최적화 (planner 예산 모드 평균 0.6 s)
- [x] 가상데이터 생성기 + 검증기 + 테스트 fixture
- [x] 오라클 벤치마크 (재현율 100 %), 물리 교차 검증 (통과 후보 100 % 안정)
- [x] 문서: README, interface, algorithms, virtual_data, VALIDATION
- [x] Draft PR 생성
- [x] 최대 높이 해석 확정 (팔레트 포함, 적재 1.35 m)
- [x] 무거운-위-가벼운 `per_box` 확정
- [ ] 박스 허용하중 실측값 반영

## 다음 할 일 (선택)

1. 팀 확인 사항([interface.md](interface.md) 8장) 결정 후 YAML 기본값 갱신
2. 재성 님 benchmark 모드(600 시나리오) 전체로 가상데이터 생성 → 동한 님 교사 데이터 재학습에 제공
3. 2차 확장: yaw 0/90 외 방향, 팔레트 slat 지지 모델

## 검증 로그

| 날짜 | 내용 | 결과 |
|---|---|---|
| 2026-10-07 | 랜덤 40박스 greedy 적재 smoke | 후보 10~115개/박스, 마스크 사유 정상 |
| 2026-10-07 | 동한 planner + 태현 backend (scenario_001/002) | 7개 시나리오 완료 |
| 2026-10-07 | 무작위 오라클 테스트에서 경계 margin 불일치 발견 → STABILITY_EPS 통일 | 통과 |
| 2026-10-07 | planner 연동 2.4~4.4 s → 최적화 | 예산 모드 평균 594 ms |
| 2026-10-07 | 20 mm 격자 오라클 100장면 | 재현율 100 %, 98.7 % 같거나 우수 |
| 2026-10-07 | 5-③ 프로브 앞 16개 유효 누락 13 % → likely-valid-first | 0 % |
| 2026-10-07 | PyBullet 교차 검증 (solid/slatted) | 통과 89/89 안정, 탈락 대조군 67 % 붕괴 |
| 2026-10-07 | 가상데이터 30 시나리오 / 720 단계 | 검증 OK, 실제 크기 관통 0 |
