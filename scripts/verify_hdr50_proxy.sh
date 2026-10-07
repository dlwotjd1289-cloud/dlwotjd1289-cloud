#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REF="${ROOT}/external/hyundai_robotics"

for repo in hdr_description hdr_ros2_driver hdr_simulation_gz; do
  if [ ! -d "${REF}/${repo}/.git" ]; then
    echo "[FAIL] missing ${repo}; run fetch_hyundai_refs.sh"
    exit 1
  fi
  branch="$(git -C "${REF}/${repo}" branch --show-current)"
  echo "[INFO] ${repo} branch=${branch}"
  if [ "${branch}" != "humble" ]; then
    echo "[FAIL] ${repo} is not pinned to humble"
    exit 1
  fi
done

echo
if grep -Rqs --exclude-dir=.git 'hdr50_22' "${REF}/hdr_description"; then
  echo "[OK] hdr_description supports hdr50_22"
else
  echo "[FAIL] hdr50_22 not found in hdr_description"
  exit 1
fi

if grep -Rqs --exclude-dir=.git 'hdr50_22' "${REF}/hdr_ros2_driver"; then
  echo "[OK] hdr_ros2_driver contains hdr50_22 MoveIt / driver assets"
else
  echo "[FAIL] hdr50_22 not found in hdr_ros2_driver"
  exit 1
fi

if grep -Rqs --exclude-dir=.git 'hdr50_22' "${REF}/hdr_simulation_gz"; then
  echo "[OK] hdr_simulation_gz supports hdr50_22"
else
  echo "[FAIL] hdr50_22 not found in hdr_simulation_gz"
  exit 1
fi

XACRO="${REF}/hdr_description/urdf/robots/hdr50_22/hdr50_22.urdf.xacro"
if [ -f "${XACRO}" ] && grep -q '<link name="flange"' "${XACRO}"; then
  echo "[OK] HDR50-22 exposes flange link for independent EOAT mount"
else
  echo "[FAIL] expected flange link not verified"
  exit 1
fi
