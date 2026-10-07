#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${ROOT}/external/hyundai_robotics"
BRANCH="humble"
mkdir -p "${DEST}"

clone_or_update() {
  local repo="$1"
  local url="https://github.com/hyundai-robotics/${repo}.git"
  local target="${DEST}/${repo}"

  if [ -d "${target}/.git" ]; then
    echo "[UPDATE] ${repo} (${BRANCH})"
    git -C "${target}" fetch origin "${BRANCH}"
    git -C "${target}" checkout -B "${BRANCH}" "origin/${BRANCH}"
  else
    echo "[CLONE] ${repo} (${BRANCH})"
    git clone --branch "${BRANCH}" --single-branch "${url}" "${target}"
  fi
}

clone_or_update hdr_description
clone_or_update hdr_ros2_driver
clone_or_update hdr_simulation_gz

echo
echo "References pinned to ROS 2 Humble under: ${DEST}"
