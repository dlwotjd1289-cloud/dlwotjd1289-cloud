#!/usr/bin/env bash
# Native setup on Ubuntu 22.04 (same packages as docker/Dockerfile), then build and test.
#   bash scripts/setup_ubuntu22.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
. /etc/os-release
[[ "$VERSION_ID" == "22.04" ]] || { echo "Ubuntu 22.04 required (found $VERSION_ID); use docker/run.sh" >&2; exit 1; }

if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
  sudo apt-get update && sudo apt-get install -y curl gnupg lsb-release software-properties-common
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
fi
sudo apt-get update
sudo apt-get install -y \
  ros-humble-desktop-full ros-humble-moveit ros-humble-moveit-visual-tools \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ros2controlcli ros-humble-gz-ros2-control \
  ros-humble-joint-state-publisher-gui python3-colcon-common-extensions python3-rosdep python3-pip \
  python3-pytest python3-networkx python3-opencv nlohmann-json3-dev libcurl4-openssl-dev \
  libboost-system-dev liburdfdom-tools git
pip3 install --user "Shapely>=2.0,<3" "pybullet>=3.2.6,<4" "aiohttp>=3.8,<4" "Pillow>=10,<13"

cd "$ROOT"
git submodule update --init --recursive
set +u   # ROS setup scripts read unset variables
source /opt/ros/humble/setup.bash
(sudo rosdep init 2>/dev/null || true); rosdep update --rosdistro humble
rosdep install --from-paths ros2_ws/src --ignore-src -r -y --rosdistro humble
bash scripts/build_ros_ws.sh
source scripts/env.sh
python3 -m pytest -q
echo "Done. In a new shell: source $ROOT/scripts/env.sh"
