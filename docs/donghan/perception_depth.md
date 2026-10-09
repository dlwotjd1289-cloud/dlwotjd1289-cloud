# 1단계 인식: Top-view 깊이 측정과 SKU 추정 (동한)

`docs/flow/01_perception.md`의 미구현 항목 "깊이(RGB-D) 처리"를 채우는 계산부입니다. 합성 깊이 이미지와 브라우저 시뮬레이터로만 검증했습니다. Gazebo 카메라는 아직 RGB이므로 실제 깊이 카메라 연결은 남아 있습니다.

## 흐름에서의 위치

```
무게 (AutoScaleCycle.measured_kg) ─┐
ID·라벨 (판독 결과 또는 None) ─────┼─► build_raw_observation ─► RawObservation ─► StateValidator (2단계)
Top-view RGB-D ─ measure_box ──────┤
라벨 미판독 시 ─ resolve_sku ──────┘
```

인식은 관측만 만듭니다. 이상 판정과 경로는 2단계가 정합니다. 출력은 `pac_runtime.perception.RawObservation` 그대로입니다.

## 파일 (`ros2_ws/src/pac_perception/pac_perception/`, 새 파일만 추가)

| 파일 | 내용 |
|---|---|
| `depth_measurement.py` | 깊이 이미지 → `conveyor` frame 점군 → 크기(긴 변·짧은 변·높이), 위치·yaw, 파손 단서(윗면 눌림, 기울어짐, 모서리 찌그러짐: 면적 비율 + 모서리 빈 거리), 확신도. 대기 중인 다음 박스를 컨베이어 방향으로 분리(`target_x`, 앞뒤 간격), ROI·이미지 경계에서 잘린 박스는 `in_full_view=False`로 파손 판단을 생략 |
| `sku_resolver.py` | 라벨 미판독 시 크기·무게로 SKU 추정. 허용오차는 `ValidatorConfig`와 같은 의미. 후보가 여럿이거나 없으면 라벨 없음 |
| `raw_observation.py` | 무게 + 깊이 측정 (+ 라벨) → `RawObservation` |
| `perception_config.py` | `config/donghan/perception_depth.yaml` 로더 |
| `signals.py` | `RawObservation`에 없는 원신호 (`PerceptionSignals`, 재인식에서 사용: [reinspection_policy.md](reinspection_policy.md)) |

`pac_perception/__init__.py`는 비어 있는 그대로 두었습니다. 모듈 경로로 import 합니다(`from pac_perception.depth_measurement import measure_box`).

## 가정

- `conveyor` frame: 저장소에 원점 정의가 없어 **픽업 구역 중심 (-1.08, 1.20)의 롤러 윗면 z 0.895, 축은 world와 같음**으로 가정했습니다. 팀 확정이 필요합니다.
- PICK ROI `[-0.50, 0.232, -0.375, 0.375]`: 스토퍼 면(x -0.845)에 기댄 가장 긴 박스(0.66 m)까지, 스토퍼 윗면과 레일은 제외합니다.
- 임계값(눌림 12 mm, 모서리 빈 거리 20 mm, 유효 깊이 85 % 등)은 시험용 가정치입니다.

## 시뮬레이터 (`viewer/donghan/perception_sim.html`)

브라우저로 바로 엽니다(three.js CDN). 재성 님 V4.4 작업셀 배치(컨베이어, 인라인 저울, PICK 스토퍼, 기둥 CCTV 자세·화각, 팔레트, 2칸 버퍼 테이블, 도착 흔들림)를 쓰되 버퍼·NG는 아래 계획 배치를 따르고, 위 계산부와 `StateValidator`, `pac_highlevel.rules.RulePolicy`를 JS로 옮겨 연속 투입, 정지 계량, 분류·적재 연산 중 정지, 버퍼, 팔레트 교체를 돌립니다.

- 초입 기둥 카메라(**위치 가정**, Gazebo 파일 미공개): 붙어서 들어온 박스 감지 → 뒤 박스 정지(갭 생성), 측면 라벨 2차 판독, Base-view 대체, 초입/PICK 크기 비교.
- "초입 카메라 유무 비교" 버튼: 같은 시드로 두 번 실행. 예: `demo_original8 ×6`, 붙어서 투입 30 % → 판정 오류 0 vs 8, 함께 계량 0 vs 19.
- 계산부만 node로: `node viewer/donghan/check_core.js demo_original8 3 6 0.3 5`
- **버퍼·NG 위치 (2026-10-10 계획, Gazebo 수정 예정)**: 계산부 맨 위 `LAYOUT` 블록 한 곳에만 있습니다.

  | 항목 | 위치 (world, m) | 비고 |
  |---|---|---|
  | 버퍼 테이블 2칸 | (-1.965, 0.25), (-1.535, 0.25), 윗면 0.9425 | 컨베이어 끝 옆 (V4.4 테이블 그대로 이동, GitHub V4.4는 아직 (1.235 / 1.665, -0.55)) |
  | NG 구역 | 중심 (-1.75, -0.45), 0.9 × 0.7 m, 3×2칸 2단 | 버퍼 바로 옆 컨베이어 반대쪽. 컨베이어 가장자리에서 구역까지 약 0.9 m, 중심까지 약 1.3 m. 로봇이 직접 내려놓음 |

  - 좌표는 **가정값**입니다. 재성 님 월드 파일이 올라오면 `LAYOUT`만 고치면 됩니다. 그 전에 URL로 시험할 수 있습니다: `perception_sim.html?buffer=-1.965,0.25;-1.535,0.25&ng=-1.75,-0.45`
  - `checkLayout()`이 로봇 도달 범위(가정 2.2 m), 팔레트·컨베이어·받침대·카메라 기둥·버퍼·NG 구역의 겹침과 5 cm 간격을 점검합니다. 결과는 상태 패널 "배치 점검", 이벤트 로그, `check_core.js` 첫 줄에 나옵니다.
  - NG 구역 가장 먼 칸은 로봇에서 2.12 m입니다(도달 범위 가정 2.2 m, 여유 8 cm). 버퍼 옆이라도 입구 쪽(−x)으로 두면 약 2.7 m라 닿지 않습니다.
- 로봇 팔 형상, 팔레트 배치 규칙(2 cm 높이맵), 존 축적 컨베이어는 시각화용 단순화입니다.
- **알려진 문제**: PICK 카메라는 V4.4 기둥 CCTV 자세(PICK과 팔레트 사이를 향함)를 쓰므로, 스토퍼에 닿은 큰 박스(K13)가 이미지 오른쪽 끝에 걸립니다. 이때 `measure_box`가 부분 시야로 보고 파손 판단을 생략해 큰 박스의 모서리 찌그러짐을 놓칠 수 있습니다(`demo_original10 ×10`, 시드 7에서 2건). 깊이 카메라를 PICK 쪽으로 향하게 하거나 PICK 전용 카메라를 두는 것을 검토해야 합니다.

## 테스트

`tests/donghan/test_depth_measurement.py`, `test_sku_resolver.py`, `test_stage1_to_stage2_contract.py`(출력이 `StateValidator`를 그대로 통과), 보조 `perception_testkit.py`.
