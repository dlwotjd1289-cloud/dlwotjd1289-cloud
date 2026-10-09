# 하드마스크 13단계: 0.25 g 옆 가속 안정성 (동한)

## 요약
- **무엇**: 새 박스를 놓은 뒤, 그 박스가 영향을 주는 짐 전체가 옆으로 0.25 g가 걸려도 가만히 있는지 검사합니다. 랩을 감기 전, 적재 과정 기준입니다.
- **어디**: `pac_candidates` 하드마스크(5-2)의 마지막 단계(13). 1~12단계를 모두 통과한 후보에만 돌고, 실패하면 `COG_VIOLATION` / `LATERAL_ACCEL:<단계>`로 거절합니다.
- **켜는 법**: 기본은 꺼져 있습니다(`constraints.lateral.enabled: false`). `config/donghan/candidates_lateral.yaml`(팀 `config/taehyeon/candidates.yaml` + `lateral` 절)을 후보 설정으로 쓰면 켜집니다. 팀 기본 설정을 바꾸려면 그 파일에 `lateral` 절을 옮기면 됩니다(태현 님 확인 필요).
- **의존성**: LP는 scipy(`pip install .[stability]`), 짧은 경사 시험은 pybullet(`.[sim]`)이 필요합니다. pybullet이 없으면 경계 사례는 거절합니다(`borderline: reject`와 같음).

## 기준값과 근거
| 항목 | 값 | 근거 |
|---|---|---|
| 옆 가속 | **0.25 g**, 8방향(±x, ±y, 대각) | 동한 결정(2026-10-10). 재성 님 `STABILITY_0_3G_BASIS.md`의 0.3 g보다 낮음. 지게차 계산값은 평소 0.20~0.24 g, 서두를 때 0.31 g |
| 박스끼리 마찰 | 0.4 | 팀 값(`STABILITY_0_3G_BASIS`, 골판지 0.3~0.5 중 보수값) |
| 박스-팔레트(합판) 마찰 | 0.4 | CTU Code 부속서 7 부록 2: 골판지-목재 = 골판지-골판지 = 0.5 → 같은 보수화로 0.4 |
| 팔레트 윗면 | 합판 판자 5장(폭 143 mm), 간격 96 mm | `config/ahead_simulator.yaml`과 같음(테스트로 일치 확인). 판자 사이 틈 위에 무게중심이 오는 바닥 박스는 정지 상태에서도 불안정으로 봄 |
| 무게중심 불확실성 | 변 길이의 5 %(최소 5 mm), 미는 방향으로 | 팀 하드마스크 `cog_uncertainty_ratio` |
| 크기 불확실성 | 접촉면을 2 mm 줄임 | 팀 `size_tolerance_m` |
| 옆 박스 기댐 | 10 mm 이내의 이웃은 밀어 줄 수 있음(누르는 힘만, 마찰 없음) | 가정. 보정 실험으로 확인 |

## 검사 단계 (`lateral.py` `check`)
1. **기하 검사**(재성 님 기둥 검사와 같은 방식): 새 박스와 그 아래 하중 경로의 박스마다, 그 위에 얹힌 박스 전체의 합성 무게중심을 `a × 높이`만큼 옮겨도 받침 다각형 안에 있는지 봅니다. 모든 방향에서 버티는 g가 `band_high_g` 이상이면 **통과**.
2. **힘 평형 LP**: 영향받는 박스 묶음(하중 경로, 그 위 박스, 10 mm 이내 이웃을 이어서 모은 것)에서, 접촉점마다 힘을 미지수로 두고 "모든 박스가 힘·모멘트 평형을 이루는 힘 배분이 있는가"를 풉니다.
   - 받침 힘은 0 이상(넘어짐), 마찰은 μ × 받침 힘 이하(미끄러짐, 8방향에서 정확한 8각형), 옆 박스는 미는 힘만.
   - a를 최대화해 방향별로 버티는 최대 g를 바로 얻습니다.
   - 기하 검사에서 `band_high_g` 미만이던 방향만 풉니다. 결과가 `band_high_g` 이상이면 **통과**, `band_low_g` 미만이면 **거절**.
3. **짧은 경사 시험**(`lateral_sim.py`, 경계 사례만): 영향받는 박스 묶음만 PyBullet에 놓고 0.25초 자리 잡은 뒤, LP가 가장 약하게 본 2방향으로 중력을 0.25 g 기울입니다(0.1초에 걸쳐 올리고 0.3초 유지, 무게중심 불확실성만큼 회전력 추가). 10 mm 넘게 움직이거나 3° 넘게 기울거나 끝에 아직 0.02 m/s 넘게 움직이면 **거절**.

## 판정 구간 보정
CALIBRATION_RESULTS

## 확인용 기준 시험 (보정 전용, `reference_tilt_test`)
- 그 시점의 팔레트 전체를 놓고, 8방향 모두 0.5초에 걸쳐 0.25 g까지 올린 뒤 1초 유지합니다(무게중심 불확실성 포함).
- 영향받는 박스 중 하나라도 20 mm 넘게 움직이거나 5° 넘게 기울면 실패로 봅니다.
- 물리 설정은 `config/ahead_simulator.yaml`과 같고(240 Hz, 반복 100, 반발 0.02, 감쇠 0.04), 마찰과 팔레트 윗면만 위 표의 값입니다.

## 실행
```bash
pip install .[stability,sim]          # scipy, pybullet
python3 -m pytest tests/donghan/test_lateral_stability.py
# 보정: 검사를 켠 팀 플래너로 에피소드를 돌리며, 매 적재마다 고른 후보 + 1~12단계 통과 후보 4개를 기준 시험과 비교
python3 tools/donghan/lateral_calibrate.py collect --episodes 7 --out cal.jsonl
python3 tools/donghan/lateral_calibrate.py analyze cal.jsonl
# 검사를 끈 팀 플래너와 비교
python3 tools/donghan/lateral_calibrate.py collect --cand config/taehyeon/candidates.yaml --samples 0 --out off.jsonl
```

## 한계
- 단단한 상자 가정입니다. 골판지 변형, 실제 마찰은 반영하지 않습니다. 마찰은 `scripts/calibrate_friction_from_tilt.py`로 실측해 바꿀 수 있습니다.
- 기준 시험도 PyBullet이라 접촉이 무른 계산 방식의 한계가 있습니다.
- 옆 박스 기댐(10 mm)은 가정입니다. 0으로 두면 이웃 도움 없이 보수적으로 판정합니다.

## 기존 기하 검사와의 관계 (정리)
- 1단계 기하 검사는 재성 님 `tools/prototypes/lookahead/lookahead.py` `stack_lateral_ok`와 같은 계산입니다. 여기에 팀 불확실성 여유(무게중심 5 %, 크기 2 mm), 대각 4방향, 판자 받침을 더했습니다.
- 여유를 0으로, 4방향·통판으로 두면 두 검사의 판정이 같다는 것을 테스트로 고정했습니다(`test_geom_stage_equals_jaesung_stack_check_without_margins`).
- 미병합 브랜치 `claude/pensive-hypatia-9d5i3j`의 하드마스크 13단계(`constraints.lateral_stability`, 0.3 g 기하 검사)도 같은 이식입니다. 하드마스크 13단계는 이 구현 하나로 두고, 그 브랜치의 13단계는 병합하지 않습니다.
- 재성 님 프로토타입의 `stack_g` 옵션은 그대로 두었습니다(프로토타입 평가 재현용).
