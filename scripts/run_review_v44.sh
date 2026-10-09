#!/usr/bin/env bash
# V4.4 final review in one command: fresh Gazebo V4.4 world + MoveIt2 with the RViz review layout
# (far CCTV / gripper camera debug images), the AHEAD live 3D viewer in the browser (PyBullet robot
# cell repeats every Gazebo placement), then a generator scenario through the closed loop.
#   bash scripts/run_review_v44.sh [SCENARIO] [COUNT]        default S0001 5
#   NO_BROWSER=1  do not open the browser      DATASET=...  generator dataset dir
#   LAYOUT=v46    V4.6 world (two fixed cameras, scale at the conveyor start, buffer next to PICK)
#   GZ_GUI=0      no Gazebo window (server only; review in RViz + browser, saves ~2 CPU cores)
# Everything stays up after the run for review; stop with: bash scripts/stop_review_v44.sh
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCEN="${1:-S0001}"; COUNT="${2:-5}"
DATASET="${DATASET:-$ROOT/../v44_generated/sample_seed20261009}"
LOG="$ROOT/logs/v44_review/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
# LAYOUT=v46: V4.6 world (scale at the conveyor start, +2.185 m conveyor, buffer table next to PICK,
# two fixed cameras: scale top view + pole CCTV, no wrist camera). Default: V4.4.
if [[ "${LAYOUT:-v44}" == v46 ]]; then
  export PAC_LAYOUT=v46 PAC_SCALE_SHIFT_M=-2.185 PAC_NO_WRIST=1 PAC_BUFFER_X=-1.05 PAC_BUFFER_Y=0.40
  WORKCELL_LAUNCH=hdr50_workcell_v4_6.launch.py
else
  WORKCELL_LAUNCH=hdr50_workcell_v4_4_pick.launch.py
fi
# gz-transport discovery on the loopback interface only: a Wi-Fi / hotspot address change broke
# discovery for every new process (new boxes could not be created, 2026-10-09). IGN_IP= to override.
export IGN_IP="${IGN_IP-127.0.0.1}"
# Hybrid-graphics laptop: render Gazebo (GUI + camera sensors) and RViz on the NVIDIA GPU (PRIME
# render offload) instead of the integrated AMD GPU, which is the desktop default. USE_NVIDIA=0 to skip.
if [[ "${USE_NVIDIA:-1}" == 1 ]] && command -v nvidia-smi >/dev/null 2>&1 \
   && [[ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]]; then
  export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
         __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  echo ">>> [리뷰] 그래픽: NVIDIA GPU 사용 (PRIME offload)"
fi

echo ">>> [리뷰 1/5] 이전 시뮬레이션 정리 중..."
bash "$ROOT/scripts/stop_review_v44.sh" >/dev/null 2>&1 || true

echo ">>> [리뷰 2/5] Gazebo 작업셀 실행 중 ($WORKCELL_LAUNCH)..."
setsid nohup ros2 launch pac_bringup "$WORKCELL_LAUNCH" > "$LOG/gazebo.log" 2>&1 < /dev/null &
for _ in $(seq 60); do ign topic -l 2>/dev/null | grep -qx /pac/gripper/state && break; sleep 1; done
for _ in $(seq 40); do timeout 5 ros2 control list_controllers 2>/dev/null | grep -q 'trajectory.*active' && break; sleep 1; done
timeout 5 ros2 control list_controllers 2>/dev/null | grep -q 'trajectory.*active' \
  || { echo "ERROR: robot controllers not active (see $LOG/gazebo.log)" >&2; exit 1; }

echo ">>> [리뷰 3/5] MoveIt2 + RViz (CCTV·그리퍼 카메라 화면) 실행 중..."
setsid nohup ros2 launch pac_bringup hdr50_moveit_v44.launch.py > "$LOG/moveit.log" 2>&1 < /dev/null &
for _ in $(seq 60); do grep -q 'Ready to take commands' "$LOG/moveit.log" && break; sleep 1; done
grep -q 'Ready to take commands' "$LOG/moveit.log" || { echo "ERROR: MoveIt not ready (see $LOG/moveit.log)" >&2; exit 1; }

echo ">>> [리뷰 4/5] 브라우저 3D 물리 뷰어 (http://127.0.0.1:4173) 실행 중..."
setsid nohup python3 "$ROOT/scripts/run_ahead_simulator.py" --no-browser > "$LOG/viewer.log" 2>&1 < /dev/null &
for _ in $(seq 30); do curl -sf http://127.0.0.1:4173/api/state >/dev/null && break; sleep 1; done
curl -sf -X POST -H 'Content-Type: application/json' -d '{}' http://127.0.0.1:4173/api/reset >/dev/null \
  || { echo "ERROR: viewer not reachable (see $LOG/viewer.log)" >&2; exit 1; }
[[ "${NO_BROWSER:-0}" == 1 ]] || (xdg-open http://127.0.0.1:4173 >/dev/null 2>&1 &)

echo ">>> [리뷰 5/5] 시나리오 $SCEN 박스 $COUNT개 실행 (브라우저 로봇이 같은 적재를 반복)..."
VIEWER_ROBOT=1 bash "$ROOT/scripts/run_generator_cycle_v44.sh" "$DATASET" "$SCEN" "$COUNT" 2>&1 | tee "$LOG/cycle.log" || true
echo
echo "REVIEW READY: Gazebo / RViz / 브라우저(http://127.0.0.1:4173) 는 검토를 위해 켜 둔 상태입니다."
echo "  로그 $LOG ; 종료: bash scripts/stop_review_v44.sh"
