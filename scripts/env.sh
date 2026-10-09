# Source this file: ROS 2 Humble + this repository's workspace and paths.
#   source scripts/env.sh
_pac_root="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)"
source /opt/ros/humble/setup.bash
[[ -f "$_pac_root/ros2_ws/install/setup.bash" ]] && source "$_pac_root/ros2_ws/install/setup.bash"
export PAC_REPO="$_pac_root"                                  # launch default for repo:=
export PAC_COMMON_CONFIG="$_pac_root/config/default.yaml"     # pac_common.config single source
export IGNITION_VERSION=fortress GZ_VERSION=fortress
unset _pac_root
