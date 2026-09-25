#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" == --help ]]; then echo 'Start Pi manual sensor/safety nodes without SLAM; no automatic movement. Additional arguments are forwarded to ros2 launch.'; exit 0; fi
root="$(cd "$(dirname "$0")/../.." && pwd -P)"
exec python3 "$root/modules/common/pi.py" ros2 launch create3_lidar_bringup create3_manual.launch.py "$@"
