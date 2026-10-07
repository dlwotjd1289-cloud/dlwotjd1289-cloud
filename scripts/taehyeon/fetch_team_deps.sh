#!/usr/bin/env bash
# Extract teammates' code (read-only copies) into .deps/team so that the
# taehyeon stage 5-1/5-2 code can be tested before every branch is merged.
# Nothing in teammates' directories is modified; .deps/ is git-ignored.
#
# Usage: scripts/taehyeon/fetch_team_deps.sh [DATASET_REF] [SIM_REF]
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DATASET_REF="${1:-origin/feature/jaesung-dataset-generator}"
SIM_REF="${2:-origin/feature/jaesung-physics-simulator}"
OUT="$ROOT/.deps/team"

cd "$ROOT"
git fetch --quiet origin \
  "${DATASET_REF#origin/}" "${SIM_REF#origin/}" 2>/dev/null || true

rm -rf "$OUT"
mkdir -p "$OUT/dataset_branch" "$OUT/sim_branch"
git archive "$DATASET_REF" | tar -x -C "$OUT/dataset_branch"
git archive "$SIM_REF" | tar -x -C "$OUT/sim_branch"

# Locate the shared packages regardless of the upload prefix.
find_dir() {
  find "$OUT/$1" -type d -path "*$2" -print -quit
}
PAC_COMMON="$(find_dir dataset_branch ros2_ws/src/pac_common)"
PAC_PLANNING="$(find_dir dataset_branch ros2_ws/src/pac_planning)"
GENERATOR="$(find_dir dataset_branch tools/ahead_dataset_generator)"
PAC_SIM="$(find_dir sim_branch ros2_ws/src/pac_simulation)"
DONGHAN_ROOT="$(dirname "$(dirname "$(dirname "$PAC_PLANNING")")")"

cat > "$OUT/paths.env" <<ENV
PAC_COMMON_SRC=$PAC_COMMON
PAC_PLANNING_SRC=$PAC_PLANNING
PAC_PLANNING_ROOT=$DONGHAN_ROOT
AHEAD_GENERATOR_ROOT=$GENERATOR
PAC_SIMULATION_SRC=$PAC_SIM
ENV
{
  echo "dataset_ref=$DATASET_REF $(git rev-parse "$DATASET_REF")"
  echo "sim_ref=$SIM_REF $(git rev-parse "$SIM_REF")"
} > "$OUT/SOURCES.txt"
cat "$OUT/SOURCES.txt"
cat "$OUT/paths.env"
