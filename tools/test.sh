#!/usr/bin/env bash
set -e
if [[ "${1:-}" == --help ]]; then echo 'Build and test SIMBA with installed dependencies. Optional SIMBA_TEST_PREFIX points to an isolated extracted dependency environment.'; exit 0; fi
root="$(cd "$(dirname "$0")/.." && pwd -P)"
cd "$root"
source modules/common/env.sh offline
if [[ -n "${SIMBA_TEST_PREFIX:-}" ]]; then
  export PYTHONPATH="$SIMBA_TEST_PREFIX/opt/ros/jazzy/lib/python3.12/site-packages:$SIMBA_TEST_PREFIX/usr/lib/python3/dist-packages:$PYTHONPATH"
  export LD_LIBRARY_PATH="$SIMBA_TEST_PREFIX/opt/ros/jazzy/lib:$SIMBA_TEST_PREFIX/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export AMENT_PREFIX_PATH="$SIMBA_TEST_PREFIX/opt/ros/jazzy:${AMENT_PREFIX_PATH:-}"
  export PATH="$SIMBA_TEST_PREFIX/usr/bin:$PATH"
  export MATPLOTLIBRC="$SIMBA_TEST_PREFIX/usr/share/matplotlib/mpl-data/matplotlibrc"
fi
export MPLCONFIGDIR="$root/reports/.matplotlib"
mkdir -p reports
colcon build --base-paths src --packages-select create3_lidar_bringup --merge-install
colcon test --base-paths src --packages-select create3_lidar_bringup --merge-install --return-code-on-test-failure
colcon test-result --verbose
python3 -m pytest tests -q --junitxml=reports/simba_tests.xml
