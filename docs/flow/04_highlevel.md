# 4. High-level 행동 선택

**흐름도**: 1차 5개 행동 모두 Rule · 확장: 앞 3개만 MaskablePPO 학습, CLOSE·REPACK은 Rule 유지 · 불가능한 행동은 Mask

| 구현 | 위치 (태현, `ros2_ws/src/pac_highlevel/pac_highlevel/`) |
|---|---|
| 행동 정의 | `actions.py` `ActionType` (:18: PLACE_CURRENT / BUFFER_CURRENT / RETRIEVE_BUFFER / PALLET_CLOSE / PARTIAL_REPACK / REJECT_NG), 학습 대상·규칙 전용 구분 (:29–30) |
| 행동 마스크 | `world.py` `PalletizingWorld.action_mask` (:295) |
| 실행용 결정기 | `runtime.py` `HighLevelDecider.decide` (:171), `load_policy` (:86) |
| 규칙 | `rules.py` `RulePolicy` (:33), `GreedyPolicy` (:61) |
| 재적재 | `repack.py` `plan_repack` (:47) |
| 학습 정책 | `ppo.py` `MaskablePPO` (NumPy), `sb3.py`, `gym_env.py` `HighLevelGymEnv`, `trainer.py`, 특징 `features.py` `observe`, 미래가치 `value.py` (`DonghanValue`, `proxy_value`) |
| 학습·평가 스크립트 | `tools/highlevel/scripts/train_highlevel_ppo.py`, `train_highlevel_sb3.py`, `evaluate_highlevel.py`; 가상 세계 `tools/virtual_data/virtual_data/highlevel.py` |
| 설정 | `config/taehyeon/highlevel.yaml` |

- **테스트**: `tests/taehyeon/test_th_highlevel.py`, `test_th_highlevel_runtime.py`, `tests/donghan/test_highlevel_handoff.py`, `test_team_order_visibility.py`
- **문서**: `docs/taehyeon/highlevel.md`
- **현황**: 런타임 결정은 항상 N개 탐색(`lookahead.py` `LookaheadPolicy`)입니다. 컨베이어 카메라에 보이는 다음 박스(`/pac/conveyor_preview`)까지 시뮬레이션해서 고릅니다. Rule은 탐색 안에서 쓰는 규칙이자 비교 기준이고, PPO는 Rule과 유의차가 없어 연구용으로만 남깁니다(감사 보고서 `docs/jaesung/audit_20261009/AUDIT_REPORT.md`).
