#!/usr/bin/env bash
# Run the look-ahead evaluator from a fresh clone of the team repo.
#   tools/prototypes/lookahead/run.sh "la_k3;stack_g=0.3;cand=hol_off;cap=general" team --episodes 2 --workers 2 --shock --out /tmp/la.json
# Holdout data: LA_DATA=tools/prototypes/lookahead/data/holdout_seed777 tools/prototypes/lookahead/run.sh ...
HERE="$(cd "$(dirname "$0")" && pwd)"; REPO="$(cd "$HERE/../../.." && pwd)"
PP=""; for d in "$REPO"/ros2_ws/src/*/; do PP="$PP:$d"; done
export PYTHONPATH="${PP#:}:$REPO/tools/virtual_data:$REPO/tools/ahead_dataset_generator/src:$REPO/scripts${PYTHONPATH:+:$PYTHONPATH}"
cd "$HERE" && exec python3 run_eval.py "$@"
