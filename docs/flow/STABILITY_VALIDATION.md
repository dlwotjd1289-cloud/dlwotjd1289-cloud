# 0.3 g 안정성 검사 검증 (2026-10-09)

0.3 g 적재 안정성 검사(팀 결정)를 다른 안정성 검사와 비교해, 0.3 g만으로 충분한지 판단하기 위한 도구입니다.
**래핑은 항상 한다**는 전제(2026-10-09)를 기본값으로 둡니다.

## 근거로 쓴 원문 (이 두 개에서 확인된 값만 사용)
- **[M]** Mazur, Gamer, Ramos & Schoder, *Standing on a common ground: a comparison of static stability approaches for pallet loading*, ITOR 34(1):501–527, doi:10.1111/itor.70059
- **[J]** Jagelčák & Kubáňová, *Risks of Goods Transport Focused on the Assessment of Semi-Trailer Dynamics on Highways for Cargo Securing*, Appl. Sci. 14(9):3846 (2024), doi:10.3390/app14093846

두 원문에 없는 값(속도를 올리는 시간 `ramp_s`, 시뮬레이션 주기 등)은 설정 파일에 `impl`로 표시했습니다.
래핑의 역학(필름 구속력, 변형 기준)은 두 원문 모두 다루지 않아, **이상적 래핑 = 박스와 팔레트가 한 덩어리**로 가정했습니다. 실제 래핑은 이보다 약하므로 래핑 후 판정은 낙관적인 쪽입니다.

## 검사 구성 (`config/stability_validation.yaml`, 항목마다 `enabled`)

| 구분 | 검사 | 판정 | 근거 |
|---|---|---|---|
| 검증 대상 | `lateral` 0.3 g | 박스마다 그 위에 얹힌 전체의 무게중심을 0.3 g × 높이만큼 ±x·±y로 옮겨도 지지 다각형 안 | 팀 결정. 재성 프로토타입 `stack_lateral_ok`와 같은 결과(무작위 600건 테스트) |
| 계산 | `sme` 정적 역학 평형 | 위 검사의 0 g 버전 (모든 층에서 무게중심이 지지됨) | [M] 표 8, 가장 일관된 방법 |
| 계산 | `pbs` 부분 지지 | 지지 면적 비율 ≥ 0.70 | [M] 2.1절, 하드마스크 값 |
| 래핑 전 (쌓는 중) | 적재 순서 시험 (PyBullet) | 한 개씩 올리고 0.3 s 시뮬레이션. 어느 박스든 x·y·z 이동 > 10 cm 또는 회전 > 10°면 그 단계에서 중단, ls = j_max / J | [M] 식 3–7, 표 7 (μs 0.5, μd 0.375 둘 다 통과해야 함) |
| 래핑 후 (운송) | 화물 단위 전도 | 박스 전체 무게중심의 팔레트 모서리까지 거리 > 가속도 × 높이(팔레트 바닥 기준) | 이상적 래핑 가정 |
| 래핑 후 프로필 | 기울임 10° (0.176 g) | prEN 17321 최소 안정성 | [J] |
| | 기울임 16.7° (0.3 g) | tan 기준 | [J] |
| | 횡 0.330 g / 0.370 g | 실측 최대 (1000 ms / 80 ms 평균) | [J] 표 5 |
| | 제동 0.492 g / 0.660 g | 실측 최대 (1000 ms / 80 ms) | [J] 표 3 |
| | EN 12195-1 0.5 / 0.6 / 0.8 g (기본 꺼짐) | 차량에 추가 고정 없이 버텨야 할 설계값 | [J]가 인용 |
| 민감도 | 판정 기준 5 / 10 / 15 (cm, °) | 결과를 다시 계산 (재시뮬레이션 없음) | [M] 그림 6 |

`wrapping.assumed: false`로 바꾸면 운송 프로필을 래핑 없는 박스에 PyBullet으로 직접 겁니다(기울임은 중력 방향 회전, 가속은 관성력).
이때 제동 프로필은 [M]의 마찰(0.375)보다 커서 어떤 적재든 박스가 미끄러지므로 꺼야 합니다.

## 점수 ([M] 표 4)
참고 판정(래핑 전 적재 시험 + 래핑 후 전도)을 정답으로 놓고 계산 검사마다 셉니다.
- 정답: 판정이 같음
- **UE (과소평가)**: 검사는 불안정, 실제는 안정 → 지나치게 엄격
- **OE (과대평가)**: 검사는 안정, 실제는 불안정 → 위험
- `OE_among_passed_pct`: 검사를 통과한 적재 중 참고 판정에서 떨어진 비율
- `reference_failures_among_lateral_passed`: 0.3 g를 통과했는데 실패한 시험별 적재 수

## 실행
```bash
# 적재 배치 만들기 (런타임 1~8 루프의 팔레트)
python3 tools/runtime/scripts/physics_replay.py --dataset <dataset> --split test --report runtime_physics.json
# 검증 (PyBullet 필요: docker/run.sh)
python3 tools/stability/run_stability_validation.py --layouts runtime_physics.json --report stability_validation.json
python3 tools/stability/run_stability_validation.py --layouts runtime_physics.json --analytic-only   # PyBullet 없이
```
래핑 가정에서는 PyBullet이 적재 시험만 돌리므로 빠릅니다(36개 박스 배치 약 3초).

## 하드마스크 연결
`pac_candidates`에 같은 0.3 g 검사를 넣었습니다(`config/taehyeon/candidates.yaml` `constraints.lateral_stability`, 거절 사유 `LATERAL_STABILITY`).
**검증이 끝날 때까지 꺼 두었습니다**(`enabled: false`). 켜면 런타임과 모든 평가의 결과가 바뀝니다.

## 첫 확인에서 나온 것 (예시 배치 2개)
- **0.35 m 박스를 3×3 기둥으로 4층(1.2 m) 쌓은 배치**: 0.3 g 검사는 불합격입니다. 기둥 하나가 높이 0.6 m의 무게중심을 0.18 m 옮기는데, 반폭이 0.175 m이기 때문입니다. 래핑하면 화물 단위로는 안정해서 **UE**입니다.
  0.3 g 검사는 이웃 박스를 벽으로 보지 않아서, 래핑을 전제로 하면 흔한 기둥 적재를 지나치게 거절할 수 있습니다. 실제 배치로 UE 비율을 꼭 확인하세요.
- **팔레트 모서리의 0.25 m 탑(높이 1.0 m)**: 0.3 g 검사와 래핑 후 전도 모두 불합격입니다(정답).
- SME·지지율은 둘 다 통과시켰습니다(OE). 중력만 보는 검사라 수평 하중을 보지 못합니다.

## 테스트
`tests/stability/` (계산 검사, 프로토타입과의 일치, 하드마스크 연결, 설정, 점수, PyBullet 시험). PyBullet 시험은 PyBullet이 없으면 건너뜁니다.
