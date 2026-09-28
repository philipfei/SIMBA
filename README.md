# SIMBA

SIMBA is a two-package ROS 2 Jazzy workspace for an iRobot Create 3 with an RPLIDAR A2M8. It keeps mapping/manual operation separate from fixed-map floor coverage. Launching a mode never starts robot motion.

## Repository layout

```text
SIMBA/
|-- README.md                  # All operator and developer instructions
|-- LICENSE                    # Apache-2.0
|-- requirements.yaml          # Auditable host dependencies
|-- .env.example               # DDS/RMW shell environment examples
`-- src/
    |-- simba_bringup/
    |   |-- config/
    |   |   |-- robot_params.yaml       # Geometry, speeds, frames, safety expectation
    |   |   |-- coverage_params.yaml    # Planning, measurement, health, deadlines
    |   |   |-- lidar_params.yaml       # Driver and base_footprint -> laser transform
    |   |   |-- slam_toolbox.yaml       # Mapping-only SLAM configuration
    |   |   |-- nav2_params.yaml        # Nav2 template, rewritten at launch
    |   |   `-- offline_maps.yaml       # Versioned map hash -> example start fixtures
    |   |-- launch/             # lidar, manual, slam, coverage, and PC RViz launchers
    |   |-- rviz/               # English SLAM and coverage displays
    |   `-- simba_bringup/      # Keyboard teleop, TF relay, velocity-gate entry point
    `-- simba_coverage/
        |-- simba_coverage/     # Planner, preview, supervisor, meter, gate, validation
        `-- test/               # Unit/ROS tests and a tracked trapezoid fixture map
```

The dependency direction is `simba_bringup -> simba_coverage`. Coverage code receives an explicit configuration directory and never imports or looks up `simba_bringup`. Runtime maps, reports, builds, local overrides, and secrets are untracked.

## Fixed defaults and configuration

All effective configuration uses ROS 2 YAML blocks in the form `node-selector: {ros__parameters: ...}`. The custom validator rejects missing/unknown keys, invalid ranges, and cross-file conflicts. `robot_params.yaml` stores a 0.18 m body radius and 0.02 m padding; code derives the 0.20 m collision radius. The separate 0.25 m coverage disk intentionally credits wall-adjacent floor while the body keeps its clearance. These parameters must never share one code variable.

Current defaults are 0.35 m stripe spacing, 0.12 m/s linear speed, 0.40 rad/s angular speed, 90% phase-boundary completion, at most three resweep rounds, 0.5 degree direction search, and a 0.35 m minimum retained trim segment. Edit tracked defaults on the development branch, review a fresh preview, commit, and let the Pi pull the commit. Do not edit tracked YAML on the Pi: `git pull --ff-only` deliberately refuses divergent work. Host-only paths belong in shell variables or untracked `config/local.yaml`; current launch arguments are preferred because `local.yaml` is not read automatically.

The configuration hash is calculated from parsed, canonical key/value data with sorted keys and exact hexadecimal floating-point tokens. Comments and key order do not change it. The approval manifest additionally covers the applicable coverage parameters, robot geometry/motion/frame values, map YAML/image SHA-256 values, reachable and coverable masks, and selected stripe family. It excludes the preview start pose, serial device, host, and deployment paths. A runtime AMCL pose may differ from the preview example start: the supervisor reprojects it into the same reachable component within the configured tolerance, recomputes connections, and refuses to start if the approved map, masks, stripe family, or effective configuration differs.

## Install and build

Install ROS 2 Jazzy first. The Pi obtains the LiDAR driver from apt; it is not a workspace package.

```bash
sudo apt update
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-irobot-create-msgs \
  python3-colcon-common-extensions python3-numpy python3-pil python3-yaml python3-pytest
# Pi only:
sudo apt install ros-jazzy-rplidar-ros
# PC visualization/preview:
sudo apt install ros-jazzy-rviz2 python3-matplotlib
```

Build on either host. Pure planner tests build on a PC without a connected robot or LiDAR; launch-time hardware dependencies are only needed when that launch is used.

```bash
export SIMBA_ROOT="$HOME/SIMBA"
cd "$SIMBA_ROOT"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Before deployment, compare `ros2 pkg xml nav2_controller`, installed parameter declarations, and `ros2 pkg executables` against the launch configuration. The coverage launch intentionally starts only `map_server`, `amcl`, `planner_server`, `controller_server`, two lifecycle managers, the supervisor, meter, velocity gate, LiDAR/static TF, and TF relay. It does not start BT Navigator, waypoint follower, smoother server, velocity smoother, behavior server, or collision monitor. `controller_server` remaps both relative and absolute `cmd_vel` outputs to `/cmd_vel_nav`; teleop publishes `/cmd_vel_remote`; the gate is the sole `/cmd_vel` publisher. Launch and runtime graph checks enforce this per node.

## DDS environment

Copy `.env.example` to an untracked host-specific shell file, verify interface names with `ip -br address`, then source it after ROS. YAML cannot set process environment variables.

```bash
source /opt/ros/jazzy/setup.bash
source "$SIMBA_ROOT/.env.local"   # created by the operator and ignored by Git
source "$SIMBA_ROOT/install/setup.bash"
```

Keep `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, and subnet discovery aligned. Verify, rather than hardcode, the PC dedicated interface and Pi `eth0`/`wlan0` names.

## Git deployment to the Pi

The protected pre-refactor state is tag `pre-refactor-2026-09-28`. Before the refactor branch is merged, the Pi checks out `refactor/two-package-layout`; after review and merge it checks out `main`.

Pi terminal:

```bash
cd "$HOME"
git clone git@github.com:philipfei/SIMBA.git SIMBA   # first deployment only
cd "$HOME/SIMBA"
git fetch --tags origin
git checkout refactor/two-package-layout
git pull --ff-only origin refactor/two-package-layout
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Git never transfers runtime maps. Transfer the canonical map from Pi to PC with host-key checking retained:

PC terminal:

```bash
mkdir -p "$SIMBA_ROOT/maps/room_20260923_1338"
scp -o HostKeyAlias=create3-pi.local -r \
  create3-pi@create3-pi.local:~/SIMBA/maps/room_20260923_1338/. \
  "$SIMBA_ROOT/maps/room_20260923_1338/"
sha256sum "$SIMBA_ROOT"/maps/room_20260923_1338/map.*
```

If the runtime Pi workspace remains `~/create3_ws`, replace only `$SIMBA_ROOT` in Pi shell commands; no tracked file contains that host-specific path.

## PC static coverage preview

A preview is a static geometric plan and ideal coverage estimate, not a closed-loop simulation. For the registered current map, the versioned example start is `(0.807, -1.575)` m. Every direction/offset candidate uses exactly that start and denominator. An unregistered map requires `--start X Y` and otherwise fails clearly.

PC terminal:

```bash
cd "$SIMBA_ROOT"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run simba_coverage coverage_preview \
  --map "$SIMBA_ROOT/maps/room_20260923_1338/map.yaml" \
  --config-dir "$SIMBA_ROOT/src/simba_bringup/config" \
  --output "$SIMBA_ROOT/output/preview-$(date +%Y%m%d-%H%M%S)"
```

Review `preview.png`, `summary.json`, `candidates.json`, `route.json`, and `approval_manifest.json`. Copy the reviewed manifest to `OUTPUT_ROOT/approvals/APPROVAL_HASH.json` on the Pi and pass that hash to coverage launch. Any effective config or map change requires a new preview. The optimizer scores all direction/offset candidates in one global tolerance band, logs every raw area and selection basis, and reports dominant and next-most-supported wall angles.

## Modes and terminal commands

`manual.launch.py`, `slam.launch.py`, and `coverage.launch.py` are mutually exclusive because each owns the LiDAR/static TF and velocity gate. Run one Pi launch at a time inside `tmux`, so SSH loss does not terminate a mission.

### Manual teleoperation

Pi terminal:

```bash
tmux new -s simba-manual
source /opt/ros/jazzy/setup.bash && source "$HOME/SIMBA/install/setup.bash"
ros2 launch simba_bringup manual.launch.py config_dir:="$HOME/SIMBA/src/simba_bringup/config"
```

PC terminal:

```bash
source /opt/ros/jazzy/setup.bash && source "$SIMBA_ROOT/install/setup.bash"
ros2 run simba_bringup keyboard_teleop --config-dir "$SIMBA_ROOT/src/simba_bringup/config"
```

The keyboard requests `/control/manual`, sends a 10 Hz heartbeat, and publishes `/cmd_vel_remote`. Manual has priority only while explicitly leased; coverage cannot run concurrently. Key-event timeout is 0.35 s. Use `W/A/S/D`, `X` or Space to stop, and `Q` to release ownership and quit.

### SLAM and RViz

Pi terminal:

```bash
tmux new -s simba-slam
source /opt/ros/jazzy/setup.bash && source "$HOME/SIMBA/install/setup.bash"
ros2 launch simba_bringup slam.launch.py config_dir:="$HOME/SIMBA/src/simba_bringup/config"
```

PC terminal:

```bash
source /opt/ros/jazzy/setup.bash && source "$SIMBA_ROOT/install/setup.bash"
ros2 launch simba_bringup rviz.launch.py mode:=slam
```

Save a map on the Pi without overwriting an existing directory:

```bash
MAP_DIR="$HOME/SIMBA/maps/room_$(date +%Y%m%d_%H%M)"
mkdir "$MAP_DIR"
ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/map"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$MAP_DIR/map'}"
```

Then use the Pi-to-PC `scp` command above with the new directory name.

### Fixed-map coverage and RViz

Pi terminal, after copying the approved manifest:

```bash
tmux new -s simba-coverage
source /opt/ros/jazzy/setup.bash && source "$HOME/SIMBA/install/setup.bash"
export MAP="$HOME/SIMBA/maps/room_20260923_1338/map.yaml"
export OUTPUT_ROOT="$HOME/SIMBA/output"
export APPROVED_HASH="PASTE_REVIEWED_APPROVAL_HASH"
ros2 launch simba_bringup coverage.launch.py \
  config_dir:="$HOME/SIMBA/src/simba_bringup/config" \
  map:="$MAP" output_dir:="$OUTPUT_ROOT" approved_hash:="$APPROVED_HASH"
```

PC terminal:

```bash
source /opt/ros/jazzy/setup.bash && source "$SIMBA_ROOT/install/setup.bash"
ros2 launch simba_bringup rviz.launch.py mode:=coverage
```

Set the initial pose in RViz, wait for stable AMCL/scan matching, then explicitly preview and start:

```bash
ros2 service call /coverage/preview std_srvs/srv/Trigger '{}'
ros2 service call /coverage/start std_srvs/srv/Trigger '{}'
ros2 service call /coverage/pause std_srvs/srv/Trigger '{}'
ros2 service call /coverage/resume std_srvs/srv/Trigger '{}'
ros2 service call /coverage/cancel std_srvs/srv/Trigger '{}'
ros2 service call /safety/reset std_srvs/srv/Trigger '{}'
```

Pause publishes a zero then silence. Resume keeps blacklist and temporary-blockage state within the same task generation; only a new task generation clears it. Manual input cannot preempt coverage until coverage is paused/cancelled and manual ownership is granted. The gate times out stale Nav/remote input after 0.5 s. Native Dock/Undock receives exclusive `NATIVE` ownership and no external zero stream. Serious hazards and disabled wheels latch; reset and resume are separate actions. `BUMP` preserves the native reflex and never triggers a blind software reverse.

The 1800 s mission watchdog covers boundary, main sweep, and up to three resweeps independently of battery thresholds. It stops, reports residual area, and does not retry. Per-target, planning, and return limits remain 60 s, 5 s, and 120 s. Completion is checked only after boundary, complete main sweep, and each complete resweep round.

### Dock and Undock

The PC needs `ros-jazzy-irobot-create-msgs` even when it only sends actions. Set the Create 3 namespace exactly as shown by `ros2 action list`; for the default root namespace:

```bash
export ROBOT_NS=""     # example namespaced robot: /create3_1
ros2 action send_goal "${ROBOT_NS}/undock" irobot_create_msgs/action/Undock '{}'
ros2 action send_goal "${ROBOT_NS}/dock" irobot_create_msgs/action/Dock '{}'
```

Run these only when manual/SLAM/coverage command ownership is inactive. A dock pose is not inferred from the map origin. Automatic return/dock remains hardware-dependent and must be calibrated and validated separately.

## Safety and process lifetime

The software reads the expected `backup_only` policy as a validated configuration expectation and warns on a reported mismatch; it never writes Create 3 web settings. Native reflexes remain enabled. `/stop_status` means motionless, while disabled wheels are read from `/wheel_status`. Hazard values 2/3/4 are serious; values 0/1 are not classified as cliff/stall emergencies.

Pressing `Ctrl-C` sends cancellation first, relinquishes leases, publishes an explicit zero burst for gate-owned motion, waits for stopped odometry or the bounded timeout, destroys nodes, and shuts down ROS last. Launch actions have bounded SIGTERM/SIGKILL escalation and no respawn. Verify no survivors with:

```bash
ros2 node list
pgrep -af 'simba_|rplidar|slam_toolbox|nav2'
```

An SSH or PC/RViz loss does not interrupt a Pi-hosted mission because supervisor, meter, and gate all run on the Pi in `tmux`. It blocks new remote commands. Remote stop is unavailable while connectivity is lost: use the physical Create 3 stop/control, and rely on the independent mission timeout only as a bounded fallback. Reattach with `tmux attach -t simba-coverage`.

The velocity gate checks fresh scan, odometry, hazard, wheel, battery, TF/localization trust, command lease, peer config hash, duplicate node names, and exactly one `/cmd_vel` publisher. On cancel, pause, conflict, or stale input it publishes explicit zero velocity before becoming silent. Create 3 command-timeout behavior must be reverified on the installed firmware before motion; software does not treat firmware timeout as its primary stopping mechanism.

## Validation order

1. Run static analysis/unit tests and the tracked trapezoid regression; verify all pass and any missing real-map regression reports an explicit skip.
2. Generate the real-map preview; verify equal stripe phase, wall-cluster angles, candidate log, route safety, and legend/numeric consistency.
3. On the stationary robot, verify versions, interfaces, fresh sensors, `map -> odom -> base_footprint -> laser`, localization trust, one `/cmd_vel` publisher, and clean `Ctrl-C` shutdown.
4. Supervise a short navigation with pause/cancel and network-loss tests.
5. Cover a small bounded physical area, then the full reachable map.
6. Validate return/dock separately only after a dock pose and native ownership behavior are confirmed.

Real robot behavior remains unverified until these stages are completed.

## Legacy operating-guide audit

The useful legacy material was retained here as updated procedures: ROS/DDS environment setup, interface verification, SSH hostname/host-key handling, LiDAR freshness checks, manual keys, SLAM/RViz startup, map save/serialization/transfer, topic and TF diagnostics, Dock/Undock action syntax, safety-setting semantics, and shutdown checks. Obsolete absolute workspaces, stale IP addresses, old 0.30/1.00 speed claims, continuous-zero relay behavior, and rsync deployment were intentionally not carried forward. The current source and this README are authoritative.
