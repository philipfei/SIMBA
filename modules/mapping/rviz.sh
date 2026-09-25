#!/usr/bin/env bash
set -e
if [[ "${1:-}" == --help ]]; then echo 'Open PC RViz using relayed TF. Requires the PC robot-facing interface.'; exit 0; fi
root="$(cd "$(dirname "$0")/../.." && pwd -P)"
source "$root/modules/common/env.sh" pc
exec rviz2 -d "$root/src/create3_lidar_bringup/rviz/create3_slam.rviz" --ros-args -r /tf:=/tf_relay -r /tf_static:=/tf_static_relay "$@"
