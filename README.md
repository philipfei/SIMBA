# SIMBA

SIMBA is a ROS 2 Jazzy workspace for an iRobot Create 3 with an RPLIDAR A2M8. It provides manual driving, SLAM, fixed-map coverage planning, runtime coverage measurement, safety-gated velocity ownership, PC RViz visualization, and exploration with one-key return to dock. Manual, SLAM and coverage launch files do not start motion. **Exploration automatically undocks after readiness checks unless `auto_undock:=false`.**

## Repository layout

```text
SIMBA/
|-- README.md                         Operator and development commands
|-- LICENSE                           Apache-2.0 license
|-- requirements.yaml                 Required Ubuntu and ROS packages
|-- .env.local                        Tracked DDS settings for both PC and Pi hosts
`-- src/
    |-- simba_bringup/                 Hardware, launch, teleop, TF, and RViz package
    |   |-- config/
    |   |   |-- robot_params.yaml      Robot radius, padding, speeds, frames, safety expectation
    |   |   |-- coverage_params.yaml   Planning, localization, health, timeout, and recovery values
    |   |   |-- lidar_params.yaml      RPLIDAR serial settings and base-to-laser transform
    |   |   |-- slam_toolbox.yaml      Mapping-mode SLAM Toolbox parameters
    |   |   |-- exploration_params.yaml Approach poses, recovery limits, live-map Nav2 overrides
    |   |   |-- nav2_params.yaml       Nav2 template rewritten with effective robot parameters
    |   |   `-- offline_maps.yaml      Map hash and versioned offline example start points
    |   |-- launch/
    |   |   |-- lidar.launch.py        Static laser TF and optional RPLIDAR driver
    |   |   |-- manual.launch.py       LiDAR, TF relay, and velocity gate for teleoperation
    |   |   |-- slam.launch.py         Manual stack plus asynchronous SLAM Toolbox
    |   |   |-- coverage.launch.py     Map server, AMCL, planner/controller, supervisor, meter, gate
    |   |   |-- explore.launch.py      Fresh SLAM, native Undock, teleop and Nav2 return to Dock
    |   |   `-- rviz.launch.py         PC RViz launcher; mode is slam or coverage
    |   |-- rviz/
    |   |   |-- slam.rviz              Map, scan, robot, and relayed TF displays
    |   |   |-- exploration.rviz       SLAM displays plus dock-ready approach and return path
    |   |   `-- coverage.rviz          Map, route, coverage markers, and localization displays
    |   `-- simba_bringup/
    |       |-- keyboard_teleop.py      W/S/A/D client, manual lease, heartbeat, zero-stop, release
    |       |-- exploration_teleop.py   Phase-aware H/X/Q/Shift+D and managed PC RViz
    |       |-- exploration.py         Hardware-independent exploration/recovery state machine
    |       |-- exploration_coordinator.py Pi lease arbitration and bounded ROS actions
    |       |-- mode_guard.py          Launch-lifetime mode lock and DDS conflict preflight
    |       |-- tf_relay.py             Relays Create 3 TF across the Pi's two DDS interfaces
    |       `-- velocity_gate.py        Installed entry point for the coverage safety gate
    `-- simba_coverage/                 Planner, execution, measurement, and safety package
        |-- simba_coverage/
        |   |-- params.py               Loads, validates, canonicalizes, and hashes all configuration
        |   |-- geometry.py             Occupancy-grid transforms, inflation, reachability, masks
        |   |-- planning.py             Wall direction, stripe phase, boundary paths, connectors
        |   |-- planner.py              Coverage target data and gap-target generation
        |   |-- preview.py              Offline saved-map preview and approval-manifest command
        |   |-- comparison.py           Static coverage statistics and English-labelled plots
        |   |-- supervisor.py           Start/pause/resume/cancel, Nav2 actions, deadlines, recovery
        |   |-- meter.py                ROS node measuring coverage from trusted actual poses
        |   |-- measurement.py          Sampling-gap and localization-jump-safe coverage accounting
        |   |-- gate_node.py            Sole /cmd_vel publisher and safety/ownership enforcement
        |   |-- health.py               Sensor freshness, hazard, localization, and command policy
        |   |-- state.py                Mission generation and state transitions
        |   `-- ros_common.py           Shared ROS messages, actions, QoS, and input tracking
        `-- test/
            |-- data/trapezoid.*        Tracked non-rectangular regression map
            |-- test_core.py            Geometry, coverage, measurement, state, and safety tests
            |-- test_revision.py        Wall alignment, stripe ranking, config, and preview tests
            |-- test_launch_contract.py Launch node/remap/shutdown contract tests
            `-- test_ros.py             Loopback ROS service, action, gate, and supervisor tests
```

The dependency direction is `simba_bringup -> simba_coverage`. Runtime maps, output, builds, and host secrets are not tracked.

## Build

The repository may be cloned anywhere. Enter its root first; `colcon` searches from the current directory.

Install the package-discovery tools once:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep
```

Then build from the repository root:

```bash
cd /path/to/SIMBA
test -f src/simba_bringup/package.xml
test -f src/simba_coverage/package.xml
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon list
colcon build --symlink-install
source install/setup.bash
```

For Daniel's current checkout, the first command is `cd /home/daniel/NerdShit/JIP/SIMBA`. A `Summary: 0 packages finished` result means either `colcon` ran outside the SIMBA root or the Colcon discovery extensions are missing. `colcon list` must show both SIMBA packages before building. The Pi installs `rplidar_ros` from apt; it is not a third workspace package.

## DDS environment

DDS settings are required **before starting any ROS process that communicates between hosts or between the Pi and Create 3**:

- Pi manual, SLAM, coverage, Dock/Undock, and diagnostic processes: always load it. The Pi must use both `eth0` and `wlan0`.
- PC keyboard teleop, RViz, robot topics/services/actions, and live diagnostics: load it.
- PC-only build, unit tests, Git operations, and offline coverage preview from a saved map: it is not required.

The tracked `.env.local` selects `eth0+wlan0` when the hostname is `create3-pi`; every other host uses the verified PC adapter `enx00e17c6840b1`. Philip's and Daniel's PCs use the same adapter name. Verify after hardware or hostname changes:

```bash
hostname -s
ip -br address
```

Do not edit `.env.local` separately on a host. Change it in Git so every machine receives the same effective configuration.

Every new live ROS terminal must load settings in this order:

PC:

```bash
source /opt/ros/jazzy/setup.bash
source /home/philip/Documents/SIMBA/.env.local
source /home/philip/Documents/SIMBA/install/setup.bash
```

Pi:

```bash
source /opt/ros/jazzy/setup.bash
source /home/create3-pi/SIMBA/.env.local
source /home/create3-pi/SIMBA/install/setup.bash
```

Loading `.env.local` after a ROS node has started does not change that process. Stop it with `Ctrl-C`, load the environment, and restart it.

## Git deployment to the Pi

`main` is the active branch. The old version is preserved as branch `origin` and tag `pre-refactor-2026-09-28`.

First installation on the Pi:

```bash
cd /home/create3-pi
git clone --branch main --single-branch https://github.com/philipfei/SIMBA.git SIMBA
cd /home/create3-pi/SIMBA
sudo rosdep init
rosdep update
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Later updates:

```bash
cd /home/create3-pi/SIMBA
git checkout main
git pull --ff-only origin main
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## Manual teleoperation

Only one of `manual.launch.py`, `slam.launch.py`, `coverage.launch.py`, and `explore.launch.py` may run at a time. Launches acquire a per-user/per-ROS-domain local lock and check existing motion/localization nodes and `/cmd_vel` publishers before starting. The safety gate continues checking the graph at runtime. DDS discovery cannot detect a network-partitioned participant.

Pi terminal inside tmux:

```bash
tmux new -s simba-manual
source /opt/ros/jazzy/setup.bash
source /home/create3-pi/SIMBA/.env.local
source /home/create3-pi/SIMBA/install/setup.bash
ros2 launch simba_bringup manual.launch.py config_dir:=/home/create3-pi/SIMBA/src/simba_bringup/config
```

PC terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /home/philip/Documents/SIMBA/.env.local
source /home/philip/Documents/SIMBA/install/setup.bash
ros2 run simba_bringup keyboard_teleop --config-dir /home/philip/Documents/SIMBA/src/simba_bringup/config
```

Keys: `W/S/A/D` move, `X` or Space stops, and `Q` stops, releases ownership, and exits. Do not add backslashes before underscores or `--config-dir`.

If ownership is refused, inspect before resetting:

```bash
ros2 topic echo /safety/state --once --full-length
ros2 topic echo /hazard_detection --once --full-length
ros2 topic echo /wheel_status --once --full-length
```

If `wheel_status.wheels_enabled` is `false` after Undock, run the following on the Pi only after hazards are clear and the robot is stopped. Releasing E-Stop enables motor power but does not command motion. Then clear the SIMBA latch:

```bash
ros2 service call /e_stop irobot_create_msgs/srv/EStop '{e_stop_on: false}'
ros2 service call /safety/reset std_srvs/srv/Trigger '{}'
```

Run `keyboard_teleop` again. A rejected ownership request now reports the exact blocker, such as `WHEELS_DISABLED`, `SCAN_STALE`, or a `/cmd_vel` ownership conflict.

## SLAM with RViz and teleoperation

Pi terminal inside tmux:

```bash
tmux new -s simba-slam
source /opt/ros/jazzy/setup.bash
source /home/create3-pi/SIMBA/.env.local
source /home/create3-pi/SIMBA/install/setup.bash
ros2 launch simba_bringup slam.launch.py config_dir:=/home/create3-pi/SIMBA/src/simba_bringup/config
```

PC RViz terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /home/philip/Documents/SIMBA/.env.local
source /home/philip/Documents/SIMBA/install/setup.bash
ros2 launch simba_bringup rviz.launch.py mode:=slam
```

Run `keyboard_teleop` in a second PC terminal using the manual command above. RViz uses `/tf_relay` and `/tf_static_relay`; its fixed frame is `map`.

Save a new map on the Pi after pressing `Q` in teleop:

```bash
MAP_DIR="/home/create3-pi/SIMBA/maps/room_$(date +%Y%m%d_%H%M)"
mkdir "$MAP_DIR"
ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/map"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$MAP_DIR/map'}"
```

Copy it to the PC:

```bash
scp -o HostKeyAlias=create3-pi.local -r create3-pi@create3-pi.local:/home/create3-pi/SIMBA/maps/ROOM_NAME /home/philip/Documents/SIMBA/maps/
```

## Exploration: Undock, teleop, H return home, Dock

This mode uses Create 3's native `/undock` and `/dock` actions. Nav2's `ComputePathToPose` and `FollowPath` handle the return route. Each session starts **on the powered dock with a fresh SLAM map**; saved pose graphs, mid-session restart recovery and multiple docks are unsupported. Do not move the dock during the session.

**The default Pi launch starts motion automatically after readiness checks.** Undock reverses away from the dock and rotates 180 degrees. Clear the rear travel area and full turning envelope before launching. PC teleop may not yet be connected; keep the physical robot stop accessible. Immediately before dispatch, a fresh `/dock_status` (at most 2 s old) must confirm `is_docked=true`. Missing/stale status or not-docked status prevents Undock.

### Upload and build updates

On the PC, review and push the changes on `main`:

```bash
cd /home/philip/Documents/SIMBA
source /opt/ros/jazzy/setup.bash
source /home/philip/Documents/SIMBA/.env.local
source /home/philip/Documents/SIMBA/install/setup.bash
git diff
git add src README.md
git commit -m "Add exploration and one-key return to dock"
git push origin main
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

On the Pi, stop the old SIMBA session, then update and rebuild:

```bash
cd /home/create3-pi/SIMBA
source /opt/ros/jazzy/setup.bash
source /home/create3-pi/SIMBA/.env.local
source /home/create3-pi/SIMBA/install/setup.bash
git checkout main
git pull --ff-only origin main
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

These update commands assume the workspace is already installed; use the earlier first-installation instructions for a new checkout. The new direct dependencies are `nav2_msgs` and `action_msgs`; all hardware/navigation dependencies remain apt/rosdep-managed.

### Start on the Pi

Every command block in this section includes the complete setup for a fresh terminal. Source the newly built installation before running.

```bash
cd /home/create3-pi/SIMBA
source /opt/ros/jazzy/setup.bash
source /home/create3-pi/SIMBA/.env.local
source /home/create3-pi/SIMBA/install/setup.bash
ros2 launch simba_bringup explore.launch.py config_dir:=/home/create3-pi/SIMBA/src/simba_bringup/config auto_undock:=true
```

Alternatively, run the same launch inside tmux. The new pane explicitly loads its own environment even if an existing tmux server has old settings:

```bash
cd /home/create3-pi/SIMBA
source /opt/ros/jazzy/setup.bash
source /home/create3-pi/SIMBA/.env.local
source /home/create3-pi/SIMBA/install/setup.bash
tmux new-session -s simba-explore "bash -c 'cd /home/create3-pi/SIMBA && source /opt/ros/jazzy/setup.bash && source /home/create3-pi/SIMBA/.env.local && source /home/create3-pi/SIMBA/install/setup.bash && exec ros2 launch simba_bringup explore.launch.py config_dir:=/home/create3-pi/SIMBA/src/simba_bringup/config auto_undock:=true'"
```

For stationary bringup diagnostics, use this full invocation. It remains in `WAIT_READY` and never grants manual motion or starts Undock. Relaunch with `true` to start a session:

```bash
cd /home/create3-pi/SIMBA
source /opt/ros/jazzy/setup.bash
source /home/create3-pi/SIMBA/.env.local
source /home/create3-pi/SIMBA/install/setup.bash
ros2 launch simba_bringup explore.launch.py config_dir:=/home/create3-pi/SIMBA/src/simba_bringup/config auto_undock:=false
```

### Start or attach on the PC

One command opens RViz and the existing terminal teleop. RViz logs go to the printed temporary log file; the terminal remains available for keys:

```bash
cd /home/philip/Documents/SIMBA
source /opt/ros/jazzy/setup.bash
source /home/philip/Documents/SIMBA/.env.local
source /home/philip/Documents/SIMBA/install/setup.bash
ros2 run simba_bringup keyboard_teleop --config-dir /home/philip/Documents/SIMBA/src/simba_bringup/config --mode exploration --rviz
```

The two-terminal fallback, or teleop-only reconnect to an existing Pi session:

```bash
cd /home/philip/Documents/SIMBA
source /opt/ros/jazzy/setup.bash
source /home/philip/Documents/SIMBA/.env.local
source /home/philip/Documents/SIMBA/install/setup.bash
ros2 run simba_bringup keyboard_teleop --config-dir /home/philip/Documents/SIMBA/src/simba_bringup/config --mode exploration
```

RViz-only attachment in another fresh terminal:

```bash
cd /home/philip/Documents/SIMBA
source /opt/ros/jazzy/setup.bash
source /home/philip/Documents/SIMBA/.env.local
source /home/philip/Documents/SIMBA/install/setup.bash
ros2 launch simba_bringup rviz.launch.py mode:=exploration
```

RViz and teleop can attach after Undock: `/exploration/state`, `/exploration/home` and `/exploration/path` are reliable, transient-local, depth-1 topics. State includes a session ID, monotonic sequence, ROS timestamp, phase/subphase, reason, actual gate owner, home validity, retries and correction count. Teleop rejects stale status.

### Keys and handover

Use **lowercase w/s/a/d** to drive. `H` returns home, `X` or Space stops/cancels, `Q` stops and exits, and **Shift+D** detaches the UI while allowing an autonomous return to continue. Uppercase D is reserved for detach because lowercase d already turns right. The original manual-mode key handling remains unchanged. Manual heartbeat continues while waiting for grant/service acknowledgements; movement is sent only after the gate confirms MANUAL. Movement keys report the blocking phase instead of being silently ignored. The PC exploration UI uses only standard ROS messages/services and does not import Create 3 or Nav2 action messages; those are required by the Pi coordinator.

Teleop requests and holds MANUAL through `/control/manual` and publishes `/cmd_vel_remote` itself. The Pi coordinator provides that service and arbitrates grants through the existing `/coverage/lease` and `/coverage/trust` internal channels. On H, teleop stops, releases MANUAL, waits for gate NONE, then requests return. Nav2 publishes `/cmd_vel_nav`; the gate remains the sole external `/cmd_vel` publisher. During native Dock/Undock, it stays silent.

| Phase | H | X / Space | Q | Movement keys |
|---|---|---|---|---|
| WAIT_READY | Rejected; not queued | Disarm startup | Disarm and exit | Ignored |
| UNDOCK | Rejected; not queued | Cancel native action; confirm stop | Cancel and exit | Ignored |
| MANUAL | Stop/release, then return | Stop manual motion | Stop/release and exit | Drive after manual grant |
| HANDOVER | Busy | Cancel continuation and stop | Cancel and exit | Ignored; fresh key required later |
| PLAN | Busy | Cancel and stop | Cancel and exit | Ignored |
| FOLLOW | Busy | Cancel and stop | Cancel and exit | Ignored |
| DOCK | Busy | Cancel native Dock; confirm stop | Cancel and exit | Ignored |
| DOCKED | Already docked; no action | No motion | Exit | Rejected |
| FAILED | Retry only with valid home and clear blockers | Acknowledge; manual-ready only when safe | Stop outstanding action and exit | Ignored until acknowledged |

**DOCKED is terminal: relaunch to undock again.** Canceled/failed Undock invalidates the home reference; recover manually and relaunch from the dock. After Dock failure, H skips navigation only when stopped within the approach tolerances, with current clearance and fresh dock visibility; otherwise it navigates again.

Repeated cancel requests are idempotent: they preserve the original deadline and failure cause, send each native goal's cancel request once (including late acceptance), and cannot restart a canceled task. If Undock was canceled before home setup completed, `HOME_INVALID` still requires recovery to the dock and a new session.

Q, Ctrl-C and SIGTERM request cancellation and wait boundedly for stopped/action-idle status. Shift+D, terminal EOF/SIGHUP, a crash or network loss leave an existing autonomous task running; manual heartbeat loss still stops manual driving. Reattachment reads status without stealing autonomous ownership. A terminal emulator sending SIGTERM uses SIGTERM behavior.

**If the PC disconnects during return, the Pi continues returning and docking. The disconnected PC cannot stop it: use the robot's physical stop buttons immediately when needed.** A timed-out Q/cancel acknowledgement is not proof of stopping. Native cancellation waits for the action terminal result and stopped odometry; rejection or missing stop confirmation after 2 s escalates to a latched E-Stop. Publishing zero does not stop an active native behavior. No user E-Stop is automatically released.

### Navigation, SLAM and return limits

Record the docked map-frame base pose before Undock and its actual exit pose afterward. Return candidates are 0.65, 0.75 and 0.55 m outward from the **docked robot-base position**, facing its original heading. Select the first candidate, in that order, passing known-space, footprint/margin, costmap clearance and successful Navfn connectivity/path checks. Stop evaluating after finding a valid candidate. The 0.3 m exit pose is not assumed plannable, and the dock's obstacles/inflation are not erased.

The live global costmap is non-rolling and dynamically follows SLAM map dimensions/origin through Nav2 StaticLayer. It retains 2 Hz updates/1 Hz publication; the 3 × 3 m local rolling costmap retains 10 Hz/2 Hz. No AMCL, map server, coverage supervisor or meter runs in this mode. Fixed-map coverage settings/hashes are unchanged.

Navfn tolerance stays 0.05 m. Exploration goal checking uses 0.03 m XY / 0.05 rad yaw and Regulated Pure Pursuit's final in-place goal-heading rotation. Before Dock, independently require stopped odometry, position within **0.05 m**, yaw within **0.05 rad**, local clearance and **two consecutive fresh dock-visible samples**. Wait at most 5 s for visibility. Create 3 does not document a guaranteed numeric IR capture region; the candidate offsets require hardware validation.

After stopping for H, wait up to 6 s for a newly received map with a timestamp newer than the stop, then up to 2 s for a subsequent matching costmap. `allow_unknown=false` remains enabled. At a frontier, back into explored space if a known-free path cannot be obtained. One navigation retry is permitted per H, using later map/costmap evidence or a different candidate; unchanged no-path evidence is not retried identically.

Corrections above 0.10 m or 10 degrees stop navigation, then wait for 2 s stable TF and a post-stop map before replanning. TF older than 0.5 s stops navigation; continued loss for 2 s fails. A third correction episode within 30 s, no stabilization within 10 s, or more than two correction recoveries per H fails. Correction recovery and the one path-failure retry are separate. Native actions retain sensor/action/lease supervision without depending on SLAM map continuity.

A fresh SLAM session does not bound drift of a historical dock pose after loop closure. Current map-to-odom corrections are not blindly applied to the stored dock pose. There is no custom IR search or raw-velocity approach fallback: invisible/unreachable docks stop with a reason for manual recovery. Measure physical error during the acceptance runs.

### Troubleshooting

Inspect state and the safety gate from a fresh Pi terminal:

```bash
cd /home/create3-pi/SIMBA
source /opt/ros/jazzy/setup.bash
source /home/create3-pi/SIMBA/.env.local
source /home/create3-pi/SIMBA/install/setup.bash
ros2 topic echo /exploration/state --once --full-length
ros2 topic echo /safety/state --once --full-length
ros2 topic echo /dock_status --once --full-length
```

| Reason/state | Operator action |
|---|---|
| SIMBA_MODE_CONFLICT / VELOCITY_GRAPH_CONFLICT | Stop the other launch or rogue publisher; inspect listed nodes. Do not bypass the gate. |
| NOT_DOCKED / DOCK_STATUS_UNCONFIRMED | Place the robot on the powered dock; verify DDS and fresh dock status; relaunch. |
| WAIT_READY with auto_undock=false | Expected diagnostic mode; relaunch with true. |
| ACTION_SERVERS_NOT_READY / GRAPH_SETTLING / OWNERSHIP_TIMEOUT | Check Nav2 lifecycle, action availability and safety graph; relaunch after fixing readiness. |
| SCAN_STALE / ODOM_STALE / HAZARD_DETECTION_STALE / WHEEL_STATUS_STALE / BATTERY_STALE / BATTERY_INVALID / BATTERY_CRITICAL | Restore sensor/status DDS flow or charge the robot; verify message timestamps before retrying. |
| CMD_VEL_OR_NODE_OWNERSHIP_CONFLICT / COORDINATOR_CONFLICT / CONFIG_HASH_MISMATCH | Stop conflicting processes or deploy matching PC/Pi configuration; resolve the graph/configuration before reset. |
| GATE_SHUTDOWN_DURING_NATIVE / PI_SHUTDOWN / STOP_NOT_CONFIRMED | Confirm physical stopping and native action termination; a gate exiting during a native action requests E-Stop as a shutdown fallback. |
| HOME_INVALID / UNDOCK_FAILED / UNDOCK_STATUS_DISAGREEMENT | Recover physically/manual as appropriate, place on dock and relaunch. |
| WHEELS_DISABLED / hazard or E-Stop fault | Stop, clear physical cause, restore wheels only when safe, then reset the latch explicitly. |
| MAP_STALE / COSTMAP_STALE | Check SLAM map publications, matching global costmap dimensions/origin and Pi load. |
| APPROACH_UNREACHABLE / NO_PATH / frontier blockage | X to acknowledge; back into known explored space or clear the approach; H again. |
| FOLLOW_FAILED / PLAN_TIMEOUT / NO_MOVEMENT / RETURN_TIMEOUT | Inspect obstacle/path and Pi load; acknowledge, reposition if needed, then H. |
| TF_STALE / SLAM_UNSTABLE | Restore scan/TF flow or allow SLAM to settle; do not reset the recorded home from the current robot pose. |
| HANDOFF_POSE_ERROR / APPROACH_BLOCKED | Check physical approach clearance and pose error; reposition manually after acknowledgment. |
| DOCK_NOT_VISIBLE / DOCK_NOT_VISIBLE_OR_CLEAR | Check dock power and IR line of sight; reposition after X; H again. |
| DOCK_FAILED / DOCK_STATUS_DISAGREEMENT / NATIVE_ACTION_TIMEOUT | Verify fresh dock status and physical position; do not assume action success means docked. |
| ACTION_UNAVAILABLE / ACTION_ACCEPT_TIMEOUT | Restore the relevant native/Nav2 action server before retrying. |
| Native GetResult hangs | For Dock/Undock, the coordinator can reconcile completion using terminal action status for the exact accepted goal UUID, fresh dock status and stopped odometry. Dock status alone never proves completion. If terminal status is also missing, cancellation/stop deadlines still apply; check the robot application logs and recover to the dock for a fresh session. |
| CANCEL_STOP_UNCONFIRMED | Use physical stop. Resolve the outstanding native action and E-Stop latch before any movement. |
| MANUAL_HEARTBEAT_LOST / stale PC status | Reconnect and read current phase; request manual control only when manual-ready. |
| DOCKED | Normal terminal completion; relaunch from dock for another session. |

If wheels remain disabled, use this full recovery block **only with the robot stopped and hazards clear**, following the existing safety reset rules:

```bash
cd /home/create3-pi/SIMBA
source /opt/ros/jazzy/setup.bash
source /home/create3-pi/SIMBA/.env.local
source /home/create3-pi/SIMBA/install/setup.bash
ros2 service call /e_stop irobot_create_msgs/srv/EStop '{e_stop_on: false}'
ros2 service call /safety/reset std_srvs/srv/Trigger '{}'
```

### Offline checks and manual acceptance

Offline policy tests use state-table, recovery-escalation, approach-selection, graph/late-attachment and installed live-map contract groups. ROS tests use isolated loopback domain 91 with fake robot/action servers. They never request motion from real hardware:

```bash
cd /home/philip/Documents/SIMBA
source /opt/ros/jazzy/setup.bash
source /home/philip/Documents/SIMBA/.env.local
source /home/philip/Documents/SIMBA/install/setup.bash
python3 -m pytest -q src/simba_coverage/test
```

Pi load and hardware acceptance are **manual operator tests; they are not executed by the development agent**. Measure a ten-minute SLAM/Nav2/gate workload including loop closure:

| Metric | Acceptance target |
|---|---|
| System CPU normalized across cores | Mean <=70%, p95 <=85%, no thermal throttling |
| cmd_vel_nav during active FollowPath | >=18 Hz in each 10 s window; p99 interval <=150 ms; no gap >=500 ms |
| Local sensor-to-costmap incorporation | p95 <=250 ms |
| Global sensor-to-costmap incorporation | p95 <=750 ms |
| SLAM map publication interval | p95 <=3 s |
| New SLAM map incorporated globally | <=2 s |
| Safety under load | No missed leases or unexplained interruption |

Measure incorporation using controlled changes or tracing, not publication timestamps alone. Profile failures before changing safety timeouts.

Run **20 manually supervised cycles** from docked startup with varied routes/headings. Require at least **19/20 successful first-H returns and confirmed docks**, including at least **five loop-closure-before-H runs** and a correction during return. Independently measure approach error before native Dock: **<=5 cm and <=5 degrees** on every run proceeding to Dock, while internal yaw checking remains tighter. Record map/physical errors, corrections, visibility, timings and reasons. Include Dock cancellation, PC disconnect, blocked approach, unavailable IR and terminal DOCKED behavior. Confirm canceled tasks never restart, fresh operator input is required to move, and native wheel-status transitions do not cause spurious faults. Never bypass disabled-wheel protection to pass acceptance.

## Offline coverage preview

This is a static geometric estimate, not a closed-loop simulation. DDS is not required.

```bash
cd /home/philip/Documents/SIMBA
source /opt/ros/jazzy/setup.bash
source /home/philip/Documents/SIMBA/install/setup.bash
ros2 run simba_coverage coverage_preview --map /home/philip/Documents/SIMBA/maps/room_20260923_1338/map.yaml --config-dir /home/philip/Documents/SIMBA/src/simba_bringup/config --output /home/philip/Documents/SIMBA/output/preview-review
```

Review `preview.png`, `summary.json`, `candidates.json`, `route.json`, and `approval_manifest.json`. Any effective configuration or map change requires a new preview and approval hash.

## Fixed-map coverage

Pi terminal inside tmux, after placing the reviewed manifest in `OUTPUT/approvals/HASH.json`:

```bash
tmux new -s simba-coverage
source /opt/ros/jazzy/setup.bash
source /home/create3-pi/SIMBA/.env.local
source /home/create3-pi/SIMBA/install/setup.bash
ros2 launch simba_bringup coverage.launch.py config_dir:=/home/create3-pi/SIMBA/src/simba_bringup/config map:=/home/create3-pi/SIMBA/maps/room_20260923_1338/map.yaml output_dir:=/home/create3-pi/SIMBA/output approved_hash:=PASTE_APPROVED_HASH
```

PC RViz:

```bash
source /opt/ros/jazzy/setup.bash
source /home/philip/Documents/SIMBA/.env.local
source /home/philip/Documents/SIMBA/install/setup.bash
ros2 launch simba_bringup rviz.launch.py mode:=coverage
```

Set the initial pose in RViz, wait for trusted localization, then use explicit services:

```bash
ros2 service call /coverage/preview std_srvs/srv/Trigger '{}'
ros2 service call /coverage/start std_srvs/srv/Trigger '{}'
ros2 service call /coverage/pause std_srvs/srv/Trigger '{}'
ros2 service call /coverage/resume std_srvs/srv/Trigger '{}'
ros2 service call /coverage/cancel std_srvs/srv/Trigger '{}'
```

The gate is the sole `/cmd_vel` publisher. Nav2 publishes `/cmd_vel_nav`; teleop publishes `/cmd_vel_remote`. Pause/cancel sends zero before silence. The 1800 s mission watchdog stops and reports residual area without automatic retry.

## Dock and Undock

The PC requires `ros-jazzy-irobot-create-msgs`. Run these only when manual/SLAM/coverage command ownership is inactive:

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}'
ros2 action send_goal /dock irobot_create_msgs/action/Dock '{}'
```

## Shutdown and safety

Press `Q` before leaving teleop. Stop a Pi launch with `Ctrl-C`; nodes cancel actions, send zero where applicable, release ownership, and exit without respawn. Verify:

```bash
ros2 node list
pgrep -af 'simba_|rplidar|slam_toolbox|nav2'
```

Pi supervisor, meter, and velocity gate run on the Pi, so loss of PC/RViz does not stop an active mission. It also prevents remote commands; use the physical Create 3 stop control if connectivity is lost. The software reads and warns about the expected `backup_only` setting but never writes Create 3 configuration.
