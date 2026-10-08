#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; DEST="$ROOT/external/hyundai_robotics"
if [ ! -d "$DEST" ]; then echo '[FAIL] references not fetched'; exit 1; fi
echo '===== Search HDP160-31 / HP160 ====='
set +e
grep -RniE --exclude-dir=.git 'HDP160[-_ ]?31|HDP160|HP160' "$DEST"
STATUS=$?
set -e
if [ "$STATUS" -eq 0 ]; then echo '[INFO] Matching content found. Inspect before deciding support status.'; else echo '[INFO] No matching public files found.'; echo 'Do not substitute another Hyundai robot model.'; fi
echo '===== branches ====='
for repo in hdr_description hdr_ros2_driver hdr_simulation_gz; do if [ -d "$DEST/$repo/.git" ]; then echo "--- $repo"; git -C "$DEST/$repo" branch -a; fi; done
