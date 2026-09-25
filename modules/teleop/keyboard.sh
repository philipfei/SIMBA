#!/usr/bin/env bash
set -e
if [[ "${1:-}" == --help ]]; then echo 'PC keyboard: W/S/A/D, X/Space stop, Q exits. Grant /control/manual first.'; exit 0; fi
root="$(cd "$(dirname "$0")/../.." && pwd -P)"
source "$root/modules/common/env.sh" pc
export PYTHONPATH="$root/src/create3_lidar_bringup:${PYTHONPATH:-}"
exec python3 "$root/src/create3_lidar_bringup/scripts/keyboard_teleop.py" "$@"
