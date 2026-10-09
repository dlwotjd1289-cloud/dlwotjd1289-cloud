#!/usr/bin/env bash
# Build (first time) and enter the PAC 2026 development container.
#   docker/run.sh            # shell in the container, repository mounted at /ws/pac2026
#   docker/run.sh --build    # rebuild the image first
# Gazebo / RViz windows use the host X server (Linux). Headless: GZ_GUI=0 (see README).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${PAC_IMAGE:-pac2026:humble}"
if [[ "${1:-}" == "--build" ]] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  [[ -f "$ROOT/external/hyundai_robotics/hdr_description/package.xml" ]] || \
    git -C "$ROOT" submodule update --init --recursive
  docker build -t "$IMAGE" -f "$ROOT/docker/Dockerfile" "$ROOT"
  [[ "${1:-}" == "--build" ]] && shift
fi
X11=()
if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
  xhost +local:docker >/dev/null 2>&1 || true
  X11=(-e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi
GPU=()
[[ -e /dev/dri ]] && GPU=(--device /dev/dri)
exec docker run --rm -it --network host --ipc host \
  "${X11[@]}" "${GPU[@]}" \
  -v "$ROOT:/ws/pac2026" -w /ws/pac2026 \
  "$IMAGE" "${@:-bash}"
