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

브라우저로 바로 엽니다(three.js CDN). 재성 님 **V4.6 작업셀**(`ahead_workcell_v4_6_two_cam.sdf`, `scripts/make_world_v46.py`, PR #5) 배치를 쓰고, 위 계산부와 `StateValidator`, `pac_highlevel.rules.RulePolicy`를 JS로 옮겨 연속 투입, 정지 계량, 분류·적재 연산 중 정지, 버퍼, NG 구역, 팔레트 교체를 돌립니다. 카메라 두 대는 RGB-D로 가정합니다(Gazebo는 RGB).

| 항목 | V4.6 값 (world, m) |
|---|---|
| 컨베이어 | x -6.985 ~ -0.80 (V4.4보다 2.185 m 김), 저울 중심 x -5.985 (투입구에서 0.65 m) |
| 카메라 1 | 저울 바로 위 (-5.985, 1.20, 2.20) 수직 하향, 화각 1.00, 1280×960 (`/pac/scale_camera/image`) |
| 카메라 2 | PICK 기둥 CCTV (-0.60, 1.78, 3.60), 화각 1.60, 1920×1080: PICK·팔레트·버퍼 두 칸 |
| 버퍼 테이블 | 중심 (-1.05, 0.44), 0.90 × 0.66 m, 칸 x -1.265 / -0.835, 윗면 0.9425, 칸에 들어가는 박스 0.37 × 0.64 m |
| NG 구역 | x -1.45 ~ -0.60, y -0.65 ~ 0.05 (중심 (-1.025, -0.30)), 로봇이 직접 내려놓음, 3×2칸 2단 |

- **카메라 1의 역할**: ① 박스가 저울에 올라서는 동안 뒤 박스가 붙어 있는지 계속 확인해 붙어 있으면 뒤 박스를 세움(갭 생성), ② 계량 시작 때 측정 기록(크기, 파손 단서, 라벨 2차 판독, Base-view 대체), ③ 계량 중 저울 위에 다른 박스가 걸치면 "계량 오염"으로 표시해 확신도 0.6(δ). V4.6은 투입구에서 저울까지 0.65 m뿐이라 뒤 박스가 앞 박스가 저울에 오른 뒤에 들어오는 경우가 많아, 진입 검사를 앞 박스가 저울 중앙에 닿을 때까지 반복합니다.
- "카메라 1 유무 비교" 버튼: 같은 시드로 두 번 실행. 예: `demo_original8 ×6`, 붙어서 투입 30 % → 판정 오류 0 vs 8, 함께 계량 0 vs 19. 진입 검사를 끄면 함께 계량된 19건을 계량 오염 표시가 19건 모두 잡음(오탐 0).
- 계산부만 node로: `node viewer/donghan/check_core.js demo_original8 3 6 0.3 5`
- **위치는 계산부 맨 위 `WC`·`LAYOUT` 블록 한 곳에만** 있습니다. PR #5가 리뷰에서 바뀌면 이 두 블록만 고치면 됩니다. 파일을 고치기 전에 URL로 버퍼·NG를 시험할 수 있습니다: `perception_sim.html?buffer=-1.265,0.44;-0.835,0.44&ng=-1.025,-0.30`
- `checkLayout()`이 로봇 도달 범위(가정 2.2 m), 팔레트·컨베이어 레일·받침대·카메라 기둥·버퍼·NG 구역의 겹침과 1 cm 간격을 점검합니다(V4.6 버퍼는 레일과 1.75 cm). 결과는 상태 패널 "배치 점검", 이벤트 로그, `check_core.js` 첫 줄에 나옵니다.
- 로봇 팔 형상, 팔레트 배치 규칙(2 cm 높이맵), 존 축적 컨베이어, 로봇 이동 시간은 시각화용 단순화입니다.
- **해소된 문제 (V4.4 → V4.6)**: V4.4 기둥 CCTV에서는 스토퍼에 닿은 큰 박스(K13)가 이미지 가장자리에 걸려 파손 판단을 건너뛰었습니다(270개 중 9건, 모서리 찌그러짐 8/10, 눌림 25/27). V4.6 카메라 2에서는 가장자리 걸림 0건(최소 여유 시뮬 90 px ≈ Gazebo 270 px), 모서리 찌그러짐 10/10, 눌림 27/27, 크기 오차 중앙값 0.4 mm(최대 2.8 mm)입니다.

## 테스트

`tests/donghan/test_depth_measurement.py`, `test_sku_resolver.py`, `test_stage1_to_stage2_contract.py`(출력이 `StateValidator`를 그대로 통과), 보조 `perception_testkit.py`.
