#!/usr/bin/env bash
# One command per V4.6 verification item (Gazebo + MoveIt2/RViz + browser viewer stay up to watch).
#   bash scripts/run_scenario_v46.sh buffer_swap   # buffer put/take, fill-the-gap, pallet change,
#                                                  # CCTV view of the buffer with the arm at READY
#   bash scripts/run_scenario_v46.sh recovery      # re-place after a 40 mm off release, re-grip after
#                                                  # a suction miss
#   bash scripts/run_scenario_v46.sh full          # S0001, 24 boxes, real stack height 1.5 / 1.6 m
# Stop: bash scripts/stop_review_v44.sh      Log: logs/v46_<name>_<time>.log
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${1:?usage: run_scenario_v46.sh buffer_swap|recovery|full}"
FIX="$ROOT/test_data/v46_fixtures"
[[ -f "$FIX/dataset/simulation_observations/T_BUFFER_SWAP.jsonl" ]] || python3 "$ROOT/scripts/make_test_scenarios_v46.py"
LOGF="$ROOT/logs/v46_${NAME}_$(date +%m%d_%H%M).log"
export LAYOUT=v46
case "$NAME" in
  buffer_swap)
    # restored 12-box pallet: the run dir copy keeps the fixture unchanged
    RUN_FIX="$ROOT/logs/v46_fixture_pallet12_$(date +%m%d_%H%M%S)"
    cp -r "$FIX/pallet12" "$RUN_FIX"
    DATASET="$FIX/dataset" RESTORE_FROM="$RUN_FIX" PAC_STACK_NOMINAL_M=0.45 PAC_STACK_MAX_M=0.45 \
      setsid nohup bash "$ROOT/scripts/run_review_v44.sh" T_BUFFER_SWAP 16 > "$LOGF" 2>&1 < /dev/null & ;;
  recovery)
    DATASET="$FIX/dataset" PAC_FAULT=place_offset:box_01,suction_once:box_02 \
      setsid nohup bash "$ROOT/scripts/run_review_v44.sh" T_RECOVERY 2 > "$LOGF" 2>&1 < /dev/null & ;;
  full)
    setsid nohup bash "$ROOT/scripts/run_review_v44.sh" S0001 24 > "$LOGF" 2>&1 < /dev/null & ;;
  *) echo "unknown scenario $NAME" >&2; exit 2 ;;
esac
echo "started $NAME (log $LOGF)"
echo "watch: tail -f $LOGF   |   stop: bash $ROOT/scripts/stop_review_v44.sh"
