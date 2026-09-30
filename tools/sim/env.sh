# Source this (after colcon build) before run_sim.py:  source tools/sim/env.sh
# Simulation only; the robot keeps .env.local (CycloneDDS).
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIMBA_ROOT="$(cd "$SIM_DIR/../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$SIMBA_ROOT/install/setup.bash"
# Own domain, local only, so a simulation never talks to a real robot on the network.
export ROS_DOMAIN_ID="${SIM_DOMAIN_ID:-77}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# FastDDS (shared memory between local processes). CycloneDDS on a PC either used the real network
# interface, where writes failed at times and TF was lost, or, restricted to loopback, dropped every
# sample larger than one fragment (~1.3 KB), including Nav2 paths.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
