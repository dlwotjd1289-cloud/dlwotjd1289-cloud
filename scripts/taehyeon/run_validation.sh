#!/usr/bin/env bash
# Reproduce every validation report (5-1/5-2, runtime 1->8) in docs/taehyeon/reports/.
#   scripts/taehyeon/run_validation.sh [SAMPLE_PER_FAMILY]
# Needs: python3.10 venv with numpy/PyYAML/pytest (+ pybullet, networkx for
# the physics cross-check) and scripts/taehyeon/fetch_team_deps.sh once.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
PER_FAMILY="${1:-5}"
OUT="tools/virtual_data/output/validation"
REPORTS="docs/taehyeon/reports"
mkdir -p "$REPORTS"

[ -f .deps/team/paths.env ] || scripts/taehyeon/fetch_team_deps.sh

echo "== 1. unit / oracle / integration tests"
python -m pytest -q tests/taehyeon | tee "$REPORTS/pytest.txt"

echo "== 2. virtual data ($PER_FAMILY scenarios per family)"
rm -rf "$OUT"
python tools/virtual_data/scripts/generate_virtual_data.py \
  --run-generator sample --sample-per-family "$PER_FAMILY" --output "$OUT" > /dev/null
python tools/virtual_data/scripts/validate_virtual_data.py "$OUT" --recheck-rows 60 \
  | tee "$REPORTS/virtual_data_validation.txt"
cp "$OUT/analysis/summary.json" "$REPORTS/virtual_data_summary.json"
cp "$OUT/analysis/summary.md" "$REPORTS/virtual_data_summary.md"

echo "== 3. candidate recall vs 20 mm grid oracle"
python tools/virtual_data/scripts/benchmark_candidates.py --scenes "$OUT/scenes" \
  --sample 100 --step 0.02 --output "$REPORTS/oracle_benchmark.json" | head -20

echo "== 4. physics cross-check (jaesung PyBullet simulator)"
for model in solid slatted; do
  python tools/virtual_data/scripts/physics_crosscheck.py "$OUT" --steps 60 --per-step 2 \
    --collision-model "$model" --report "$REPORTS/physics_$model.json" | tail -18
done

echo "== 5. donghan planner (5-3~5-6) with this backend, 1 s soft budget"
python tools/virtual_data/scripts/planner_benchmark.py --scenes "$OUT/scenes" --sample 30 \
  --output "$REPORTS/planner_benchmark.json" | tail -14
echo "== 6. carton strength coverage: assumed capacity vs hidden true strength (full pallets)"
FULL="tools/virtual_data/output/validation_full"
rm -rf "$FULL"
python tools/virtual_data/scripts/generate_virtual_data.py \
  --run-generator sample --sample-per-family 2 --boxes-per-scenario 80 --output "$FULL" \
  --no-scenes > /dev/null
cp "$FULL/analysis/summary.md" "$REPORTS/full_pallet_summary.md"
python tools/virtual_data/scripts/strength_sweep.py --dataset "$FULL/source_dataset" \
  --safety-factors 1 2 4 8 16 --report "$REPORTS/strength_sweep.json"

echo "== 7. runtime 1->8 loop (stages 1-3, 6-8) on held-out scenarios, 4 variants"
python tools/runtime/scripts/run_runtime.py --run-generator 10 --split test \
  --report "$REPORTS/runtime_test.json" | cut -c1-200
python tools/runtime/scripts/physics_replay.py --run-generator 10 --split test \
  --report "$REPORTS/runtime_physics.json" | tail -4
python tools/runtime/scripts/export_viewer.py "$REPORTS/runtime_physics.json" "$REPORTS/runtime_viewer.html"
echo "reports -> $REPORTS"
