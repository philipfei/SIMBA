#!/usr/bin/env bash
# Source as: source modules/common/env.sh offline|pc|pi
simba_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
simba_mode="${1:-offline}"
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [[ "$simba_mode" == offline ]]; then
  export ROS_DOMAIN_ID=91 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
  export PYTHONPATH="$simba_root/src/create3_lidar_bringup:${PYTHONPATH:-}"
elif [[ "$simba_mode" == pc ]]; then
  simba_interface="${SIMBA_PC_INTERFACE:-enx00e17c6840b1}"
  [[ "$simba_interface" =~ ^[a-zA-Z0-9_.:-]+$ ]] || { echo 'Invalid interface' >&2; return 2; }
  ip link show "$simba_interface" >/dev/null || return 2
  export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$simba_interface\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
elif [[ "$simba_mode" == pi ]]; then
  ip link show eth0 >/dev/null && ip link show wlan0 >/dev/null || return 2
  simba_install="$(readlink -f /home/create3-pi/create3_ws/install)"
  [[ -f "$simba_install/setup.bash" ]] || { echo 'Missing installed workspace' >&2; return 2; }
  source "$simba_install/setup.bash"
  export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="10" multicast="default"/><NetworkInterface name="wlan0" priority="20" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
else
  echo 'Expected offline, pc or pi' >&2
  return 2
fi
