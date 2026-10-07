# Changelog

## v1.2.0

- Ubuntu 22.04 / Python 3.10 기준으로 계약 정리.
- 모든 내부 길이 단위를 mm에서 m로 변경.
- orientation 저장을 degree에서 rad로 변경.
- `pac_common` canonical dataclass 추가 및 Generator가 이를 import하도록 변경.
- JSON 설정을 YAML로 변경.
- canonical test fixture를 JSON/JSONL로 변경하고 CSV는 analysis 전용으로 축소.
- 모든 Pose에 `frame_id` 포함.
- Planner-safe scenario와 exact future ground truth 물리 분리.
- pytest Contract / Reproducibility / Schema / Python 3.10 syntax 테스트 추가.
- Python 3.13 `__pycache__` 제거.
- Robot feasibility는 Generator 밖에 유지.
