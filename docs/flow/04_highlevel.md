# 4. High-level 행동 선택

**흐름도**: 5개 행동 모두 Rule(또는 Look-ahead 탐색) · CLOSE·REPACK은 Rule · 불가능한 행동은 Mask.
**PPO(학습 정책)는 사용하지 않습니다** (2026-10-09 결정, 코드·모델·학습 스크립트 삭제).

| 구현 | 위치 (태현, `ros2_ws/src/pac_highlevel/pac_highlevel/`) |
|---|---|
| 행동 정의 | `actions.py` `ActionType` (PLACE_CURRENT / BUFFER_CURRENT / RETRIEVE_BUFFER / PALLET_CLOSE / PARTIAL_REPACK / REJECT_NG) |
| 행동 마스크 | `world.py` `PalletizingWorld.action_mask` |
| 실행용 결정기 | `runtime.py` `HighLevelDecider.decide`, `load_policy("rule" \| "greedy")` |
| 규칙 | `rules.py` `RulePolicy`, `GreedyPolicy` |
| Look-ahead | `lookahead.py` `LookaheadPolicy` (보이는 다음 N개 박스 탐색, 학습 없음), `realtime.py`, `placement.py` `LayerPlacer` |
| 재적재 | `repack.py` `plan_repack` |
| 미래가치 | `value.py` (`proxy_value`, `DonghanValue`) |
| 에피소드 실행 | `rollout.py` `run_policy` |
| 평가 스크립트 | `tools/highlevel/scripts/evaluate_highlevel.py` (no_buffer / greedy / rule), `evaluate_lookahead.py`, `run_realtime.py`; 가상 세계 `tools/virtual_data/virtual_data/highlevel.py` |
| 설정 | `config/taehyeon/highlevel.yaml`, `lookahead.yaml` |

- **테스트**: `tests/taehyeon/test_th_highlevel.py`, `test_th_highlevel_runtime.py`, `test_th_lookahead.py`, `tests/donghan/test_highlevel_handoff.py`, `test_team_order_visibility.py`
- **문서**: `docs/taehyeon/highlevel.md`, `lookahead.md`
