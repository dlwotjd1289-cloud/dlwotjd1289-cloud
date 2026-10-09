"""Reinspection (RI) reason catalog.

RI codes are plain strings recorded in ``Verdict.notes`` ("RI-B6: ...") and in
``ReinspectionResult.ri_codes``. The shared ``RejectCode`` enum (pac_common) is
not extended; ``reject_code`` names an existing member or is None.

``origin``: EXISTING = behaviour already in this repo before RI (origin/main
2bb4bac, file in ``where``); NEW = proposal. ``feasibility``: where it can be
exercised today. ``implemented_here``: handled by this package.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Reason:
    code: str
    title: str
    anomaly: str            # pac_runtime Anomaly kind the box ends in (one of the existing five)
    reject_code: str | None
    action: str
    origin: str             # EXISTING | NEW
    feasibility: str        # SIMULATION | GAZEBO | HARDWARE | SUPERVISOR
    implemented_here: bool
    where: str = ""


_R = Reason
REASONS = {r.code: r for r in (
    # A. identity
    _R("RI-A1", "라벨 미판독", "RECOGNITION_FAIL", "TRACKING_LOST", "Base-view 1회, 실패 시 INSPECTION", "EXISTING", "SIMULATION", True, "state_validator.py"),
    _R("RI-A2", "확신도 < min_label_confidence (라벨·크기 공용 값)", "RECOGNITION_FAIL", "TRACKING_LOST", "Base-view 1회, 실패 시 INSPECTION", "EXISTING", "SIMULATION", True, "state_validator.py"),
    _R("RI-A3", "라벨 부분 판독 / 체크섬 불일치", "RECOGNITION_FAIL", "TRACKING_LOST", "재판독(상한), 실패 시 라벨 없음 → A1 경로", "NEW", "SIMULATION", True),
    _R("RI-A4", "판독 SKU가 catalog에 없음", "UNKNOWN_SKU", "INVALID_STATE", "재판독 1회(신규), 실패 시 INSPECTION", "EXISTING", "SIMULATION", True, "state_validator.py"),
    _R("RI-A5", "box_id 중복 / 이미 PLACED", "-", None, "현재: 관측 무시. 신규: HOLD + 운영자 알림", "EXISTING", "SUPERVISOR", False, "core.py on_observation"),
    _R("RI-A6", "remaining_by_sku가 0인 SKU 도착", "UNKNOWN_SKU", "INVALID_STATE", "재판독, 실패 시 INSPECTION", "NEW", "SIMULATION", True),
    _R("RI-A7", "상단/보조 뷰 라벨 불일치", "RECOGNITION_FAIL", "TRACKING_LOST", "3번째 판독 다수결, 불일치 시 INSPECTION", "NEW", "SIMULATION", True),
    # B. sensor cross-check
    _R("RI-B1", "크기 공칭값 불일치 (축별 max(12 mm, 4 %))", "SPEC_MISMATCH", "SIZE_MISMATCH", "Base-view 재측정, 일치하면 OK, 아니면 실측값 + δ", "EXISTING", "SIMULATION", True, "state_validator.py"),
    _R("RI-B2", "무게가 SKU 범위 ±8 % 밖", "SPEC_MISMATCH", "WEIGHT_MISMATCH", "재계량(신규), 아니면 실측값 + δ", "EXISTING", "SIMULATION", True, "state_validator.py"),
    _R("RI-B3", "크기·무게 동시 불일치", "SPEC_MISMATCH", "SIZE_MISMATCH", "라벨 재판독 우선 (두 코드는 기존에도 함께 기록)", "NEW", "SIMULATION", True),
    _R("RI-B4", "높이(z)만 불일치", "SPEC_MISMATCH", "SIZE_MISMATCH", "재촬영", "NEW", "SIMULATION", True),
    _R("RI-B5", "x/y 뒤바뀜", "OK", None, "통과 (기록만)", "EXISTING", "SIMULATION", True, "state_validator.py"),
    _R("RI-B6", "저울 미안정", "OK", "SENSOR_UNCERTAIN", "재계량(상한), 미해소 시 PLAN + δ", "EXISTING", "GAZEBO", True, "scripts/scale_cycle_core_v43.py (런타임 미연결)"),
    _R("RI-B7", "저울 영점 이상", "OK", "SENSOR_UNCERTAIN", "영점 재설정 후 재계량, 미해소 시 PLAN + δ", "EXISTING", "GAZEBO", True, "scripts/scale_cycle_core_v43.py TARE (런타임 미연결)"),
    # C. perception quality
    _R("RI-C1", "확신도 < 1.0", "OK", None, "재인식 없이 δ", "EXISTING", "SIMULATION", True, "state_validator.py"),
    _R("RI-C2", "프레임 간 크기 분산 초과", "OK", "SENSOR_UNCERTAIN", "프레임 추가, 미해소 시 PLAN + δ", "NEW", "SIMULATION", True),
    _R("RI-C3", "윤곽 일부 가림 / 시야 경계", "OK", "SENSOR_UNCERTAIN", "재촬영, 미해소 시 PLAN + δ", "NEW", "SIMULATION", True, "pac_perception/depth_measurement.py in_full_view"),
    _R("RI-C4", "반사·그림자·저조도로 윤곽 실패", "RECOGNITION_FAIL", "TRACKING_LOST", "노출 조정 / Base-view", "NEW", "HARDWARE", False),
    _R("RI-C5", "두 박스가 하나로 인식", "SPEC_MISMATCH", "SIZE_MISMATCH", "분리 대기 후 재촬영, 미해소 시 INSPECTION", "NEW", "SIMULATION", True),
    _R("RI-C6", "한 박스가 두 윤곽으로 쪼개짐", "SPEC_MISMATCH", "SIZE_MISMATCH", "재촬영, 병합 규칙", "NEW", "SIMULATION", False),
    _R("RI-C7", "깊이 결측률 초과", "OK", "SENSOR_UNCERTAIN", "재촬영, 미해소 시 PLAN + δ", "NEW", "SIMULATION", True, "pac_perception/depth_measurement.py valid_ratio"),
    _R("RI-C8", "도착 자세가 예상 흔들림 범위 밖 (90° 배수 기준)", "RECOGNITION_FAIL", "SENSOR_UNCERTAIN", "재추정, 미해소 시 INSPECTION", "NEW", "SIMULATION", True),
    # D. appearance
    _R("RI-D1", "눈에 띄는 파손 (visual_damage)", "DAMAGED", "INVALID_STATE", "reject: INSPECTION / place_no_load: PLAN + 상부 0 N", "EXISTING", "SIMULATION", True, "state_validator.py"),
    _R("RI-D2", "파손 판단 경계값 부근", "DAMAGED", None, "다른 뷰로 재확인, 미해소 시 파손으로 간주", "NEW", "SIMULATION", True),
    _R("RI-D3", "무게가 범위 아래 (내용물 이탈 의심)", "SPEC_MISMATCH", "WEIGHT_MISMATCH", "기록만 (DAMAGED 재분류는 팀 결정)", "EXISTING", "SIMULATION", True, "state_validator.py (B2 경로)"),
    _R("RI-D4", "테이프 풀림 / 뚜껑 열림 / 윗면 돌출", "DAMAGED", None, "재측정", "NEW", "HARDWARE", False),
    _R("RI-D5", "젖음 / 오염", "DAMAGED", None, "INSPECTION", "NEW", "HARDWARE", False),
    # E. flow / inventory (stage 3)
    _R("RI-E1", "컨베이어 유휴 missing_timeout_s(30 s) → MISSING 자동 확정", "-", None, "남은 기대 수량을 remaining에서 제거", "EXISTING", "SUPERVISOR", False, "supervisor.py confirm_missing"),
    _R("RI-E2", "도착 순서·간격 이상", "-", "INVALID_STATE", "라벨 재판독", "NEW", "SUPERVISOR", False),
    _R("RI-E3", "대기열 수 > 미도착 기대 수", "-", "INVALID_STATE", "컨베이어 정지 후 재집계", "NEW", "SUPERVISOR", True, "supervisor.py outstanding (비교는 신규)"),
    _R("RI-E4", "같은 SKU 연속 도착 이상", "-", None, "경고 로그", "NEW", "SUPERVISOR", False),
    _R("RI-E5", "인식 후 픽업 지연으로 자세 변화", "-", "STALE_PLAN", "인식 폐기 후 재인식", "NEW", "SUPERVISOR", False),
    # F. execution feedback (stage 7)
    _R("RI-F1", "파지 실패", "-", None, "재시도 1회 → 다른 파지(yaw+π) → L3, Inspection 사유 \"GRIP_FAIL\"", "EXISTING", "SIMULATION", False, "executor.py grip, core.py on_result"),
    _R("RI-F2", "파지 후 무게가 인식 때와 다름", "-", "WEIGHT_MISMATCH", "내려놓고 재계량", "NEW", "HARDWARE", False),
    _R("RI-F3", "이송 중 낙하", "-", None, "정지, 낙하 위치 재탐지 (V4.4는 위치 신호로 감지 → Fatal)", "EXISTING", "GAZEBO", False, "scripts/moveit_pick_place_v44.py"),
    _R("RI-F4", "배치 후 측정 위치 오차", "-", None, "L0/L1/L2 기록, 형상 문제는 L4 → HOLD + 보정 위치", "EXISTING", "SIMULATION", False, "core.py verify, VerifyConfig"),
    _R("RI-F5", "배치 후 기울어짐·밀림", "-", None, "검사 문자열 LOW_SUPPORT/PENETRATION/PROTRUSION/HEIGHT_LIMIT → L4 HOLD", "EXISTING", "SIMULATION", False, "executor.py check"),
    _R("RI-F6", "재인식을 반복해도 판정이 계속 바뀜", "-", "SENSOR_UNCERTAIN", "상한 도달 시 INSPECTION 강제", "NEW", "SIMULATION", True),
)}
