#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" == --help ]]; then echo 'Read one battery message; updates may take more than five seconds.'; exit 0; fi
root="$(cd "$(dirname "$0")/../.." && pwd -P)"
exec python3 "$root/modules/common/pi.py" ros2 topic echo --once /battery_state
