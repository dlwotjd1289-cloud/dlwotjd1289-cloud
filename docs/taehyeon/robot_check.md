# 6단계: 로봇 실행 가능성 (`pac_robot_check`, HDR50-22)

흐름도 6번 "하강 경로 간섭(박스+그리퍼) → Reach / IK → Collision → Payload"를 구현했습니다.
2026-10-08 태현 님이 흐름도의 미완성 부분을 맡기로 해서 태현 파트로 추가했습니다(팀원 파일은 수정하지 않음).

## 1. 무엇을 검사하나

입력은 5단계가 고른 후보(`PlacementCandidate`, 팔레트 모서리 원점, 회전 후 AABB 최소 모서리)입니다.
호출 형식은 동한 님 `docs/integration.md`의 계약을 그대로 따릅니다.

```python
from pac_robot_check import RobotFeasibility, load_robot_check_config

robot = RobotFeasibility(load_robot_check_config("config/taehyeon/robot_check.yaml"))
verdict = robot.validate_robot_motion(box, candidate, latest_state)   # pac_common ValidationResult
chosen, verdict, rejected = robot.first_executable(box, result.ranked, latest_state)  # "실패 시 다음 후보"
```

| 순서 | 검사 | 실패 코드 |
|---|---|---|
| 0 | 계획 version이 최신 상태와 같은가, frame이 `pallet`인가 | `STALE_PLAN`, `INVALID_STATE` |
| 1 | 가반하중: 박스 + 그리퍼 ≤ 정격 50 kg | `PAYLOAD_EXCEEDED` |
| 2 | 하강 경로: 그리퍼 몸체(0.34 × 0.26 m)가 박스 윗면보다 높은 이웃 박스와 겹치지 않는가 | `ROBOT_COLLISION` |
| 3 | Reach / IK: 놓는 자세(그리퍼 아래 방향, 박스 yaw 또는 +180°)에 관절 한계 안의 해가 있는가 | `IK_FAIL` |
| 4 | 접근·후퇴: 0.30 m 위에서 수직으로 내려오는 경로 전체(5 cm 간격)에 연속된 해가 있는가 | `APPROACH_FAIL` |
| 5 | 팔 충돌: 위팔·아래팔·손목을 캡슐로 보고 적재된 박스·팔레트 상판·바닥과의 거리 | `ROBOT_COLLISION` |

- 그리퍼 방향 2가지(박스 yaw, yaw+180°) × IK 해(최대 8개)를 모두 시도해, 통과한 것 중 예상 사이클 시간이 가장 짧은 것을 고릅니다.
- 통과하면 `details`에 관절값(놓는 자세·접근 자세), 그리퍼 yaw, 팔 최소 여유거리, 가장 가까운 장애물, 예상 사이클 시간, 하중 중심(TCP 아래 박스 높이의 절반)을 담습니다.
- `robot_time_sec_by_candidate(box, candidates, state)`는 동한 님 `PlanningContext.robot_time_sec_by_candidate`에 넣을 사이클 시간을 줍니다.

## 2. 근거와 가정

| 항목 | 값 | 출처 |
|---|---|---|
| 관절 원점·축·위치 한계·속도 한계 | URDF 그대로 | 현대로보틱스 `hdr_description` (humble, f026592) `hdr50_22.urdf.xacro` |
| home 자세 | j2 = 1.5707 (위팔 수직) | 같은 회사 `hdr50_22_moveit_config` |
| 정격 가반하중 | 50 kg | 모델명 HDR50-22 (구 HH050) |
| 그리퍼 크기·TCP | 0.34 × 0.26 m, 플랜지에서 0.22 m | pac2026-ahead `pac_eoat/vacuum_gripper_v1` |
| 그리퍼 질량 | 15 kg | **가정** |
| 팔 캡슐 반지름 | 위팔 0.16, 아래팔 0.13, 손목 0.10 m | **가정** (메시 대신 사용, MoveIt 메시 충돌이 최종) |
| 로봇 위치 | 팔레트 중심에서 -y 쪽 1.15 m, 받침대 0.5 m | **가정** (3장 비교로 선택) |
| 컨베이어 집는 위치 | 로봇 옆 1.3 m, 컨베이어 윗면 바닥에서 0.9 m | **가정** |
| 속도 | URDF 관절 속도 한계의 50 %, 수직 이동 0.25 m/s, 잡기·놓기 0.5 s | **가정** |

IK는 이 로봇 구조(어깨 오프셋 + 링크 2개 + 구면 손목)의 해석해입니다. 무작위 자세 300개 중 299개에서 원래 관절값을 되찾았고, 순기구학 재확인 오차는 0.01 mm 이하입니다(`test_th_robot_check.py`).
검사 1회는 약 5~45 ms입니다.

## 3. 로봇 배치 비교 (`reports/robot_reach_layouts.json`)

팔레트 3규격 × 박스 2종 × 위치 16곳 × 높이 4단(맨 위층 포함) = 384곳, 빈 팔레트 기준입니다.

| 로봇 배치 | 도달 | 맨 위층 |
|---|---|---|
| pac2026-ahead 작업셀 v2 (팔레트 모서리에서 1.68 m, 바닥) | 199/384 | 0/96 |
| 바닥, 팔레트 긴 변 옆 | 274/384 | 0/96 |
| **받침대 0.5 m, 팔레트 중심에서 1.15 m (기본값)** | **384/384** | **96/96** |

- **바닥에 둔 HDR50-22는 1.35 m 높이 맨 위층에 그리퍼를 아래로 향해 놓을 수 없습니다.** 손목 j5 한계(±125°) 때문입니다. 받침대가 필요합니다.
- pac2026-ahead 작업셀 배치에서는 1.2 × 1.0 팔레트의 먼 쪽 절반이 닿지 않습니다. 이 배치를 그대로 쓰려면 로봇 위치를 옮겨야 합니다.

## 4. 검사가 막는 실제 사례 (테스트로 고정)

- 키 큰 기둥 두 개 사이 0.2 m 틈: 박스는 들어가지만(5-② 통과) 그리퍼(0.34 m)가 기둥에 걸립니다 → `ROBOT_COLLISION`.
- 로봇 쪽에 1.3 m 벽이 있고 그 너머 바닥에 놓기: 어떤 IK 해로도 아래팔이 벽을 지납니다 → `ROBOT_COLLISION`. 벽이 0.6 m면 통과합니다.
  즉 "로봇에서 먼 쪽부터 쌓아야 한다"는 실행 순서 제약이 자동으로 반영됩니다.
- 40 kg 박스 + 그리퍼 15 kg > 50 kg → `PAYLOAD_EXCEEDED`.

## 5. 한계 (다음 담당자 / 실제 셀 확보 시)

- 동적 토크·가속도, 자기 충돌, 컨베이어 → 팔레트 이송 경로 중간은 검사하지 않습니다(양 끝 자세와 관절공간 시간만). MoveIt 2에서 메시 충돌·경로 계획으로 최종 확인해야 합니다.
- 그리퍼 질량, 셀 배치, 캡슐 반지름은 실측값으로 바꿔야 합니다(`config/taehyeon/robot_check.yaml`).
- 흡착 위치는 박스 윗면 중심 고정입니다. 작은 박스 여러 개 동시 파지나 옆면 파지는 없습니다.
