# SIMBA

SIMBA is a ROS 2 Jazzy workspace for an iRobot Create 3 with an RPLIDAR A2M8. It provides manual driving, SLAM, fixed-map coverage planning, driving coverage plans made on the PC with Nav2, runtime coverage measurement, safety-gated velocity ownership, PC RViz visualization, and a closed-loop simulation for testing without the robot. Starting a launch file never starts robot motion.

## Repository layout

```text
SIMBA/
|-- README.md                         Operator and development commands
|-- LICENSE                           Apache-2.0 license
|-- requirements.yaml                 Required Ubuntu and ROS packages
|-- .env.local                        Tracked DDS settings for both PC and Pi hosts
|-- tools/
|   |-- coverage_tool/                 PC desktop planner: coverage path on a saved map, exported as a plan .yaml
|   `-- sim/                           Closed-loop simulation: real SIMBA/Nav2 stack with a fake Create 3 and LiDAR
`-- src/
    |-- simba_bringup/                 Hardware, launch, teleop, TF, and RViz package
    |   |-- config/
    |   |   |-- robot_params.yaml      Robot radius, padding, speeds, frames, safety expectation
    |   |   |-- coverage_params.yaml   Planning, localization, health, timeout, and recovery values
    |   |   |-- lidar_params.yaml      RPLIDAR serial settings and base-to-laser transform
    |   |   |-- slam_toolbox.yaml      Mapping-mode SLAM Toolbox parameters
    |   |   |-- nav2_params.yaml       Nav2 template rewritten with effective robot parameters
    |   |   |-- navigate_through_poses.xml  Nav2 behavior tree for exported plans (no motion recoveries)
    |   |   `-- offline_maps.yaml      Map hash and versioned offline example start points
    |   |-- launch/
    |   |   |-- lidar.launch.py        Static laser TF and optional RPLIDAR driver
    |   |   |-- manual.launch.py       LiDAR, TF relay, and velocity gate for teleoperation
    |   |   |-- slam.launch.py         Manual stack plus asynchronous SLAM Toolbox
    |   |   |-- coverage.launch.py     Map server, AMCL, planner/controller, supervisor, meter, gate
    |   |   `-- rviz.launch.py         PC RViz launcher; mode is slam or coverage
    |   |-- rviz/
    |   |   |-- slam.rviz              Map, scan, robot, and relayed TF displays
    |   |   `-- coverage.rviz          Map, route, coverage markers, and localization displays
    |   `-- simba_bringup/
    |       |-- keyboard_teleop.py      W/S/A/D client, manual lease, heartbeat, zero-stop, release
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

On the PC, `rosdep` does not install everything in `requirements.yaml` (for example `ros-jazzy-rviz2`, `python3-matplotlib`, `python3-scipy`, `python3-tk` for coverage_tool). Install the `pc` lists of `requirements.yaml` with `apt` as well.

A `Summary: 0 packages finished` result means either `colcon` ran outside the SIMBA root or the Colcon discovery extensions are missing. `colcon list` must show both SIMBA packages before building. The Pi installs `rplidar_ros` from apt; it is not a third workspace package.

### Tests

On the PC, after building:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest -q src/simba_coverage/test
python3 tools/coverage_tool/tests/regression.py
```

`test_ros.py` starts real ROS nodes on loopback (domain 91) with CycloneDDS, so it needs `ros-jazzy-rmw-cyclonedds-cpp` and `ros-jazzy-irobot-create-msgs`. It is skipped when they are missing. Its fake robot jumps between poses, so on a busy PC about one full run in three has one test fail with `TF_UNAVAILABLE`, `LOCALIZATION_SETTLING` or `PHASE_WAIT`. Run that test again on its own (`-k <name>`) before treating it as a real failure. Behaviour on the robot map with the full Nav2 stack is checked with `tools/sim` (see its README).

## DDS environment

DDS settings are required **before starting any ROS process that communicates between hosts or between the Pi and Create 3**:

- Pi manual, SLAM, coverage, Dock/Undock, and diagnostic processes: always load it. The Pi must use both `eth0` and `wlan0`.
- PC keyboard teleop, RViz, robot topics/services/actions, and live diagnostics: load it.
- PC-only build, unit tests, Git operations, and offline coverage preview from a saved map: it is not required.

The tracked `.env.local` selects `eth0+wlan0` when the hostname is `create3-pi`; every other host uses the verified PC adapter `enx00e17c6840b1` (the USB Ethernet adapter the team's PCs share). **A PC with a different adapter cannot talk to the robot until its name is in `.env.local`**: find it with `ip -br address` and add a hostname case for that PC. `/path/to/SIMBA` in the commands below stands for your clone. Verify after hardware or hostname changes:

```bash
hostname -s
ip -br address
```

Do not edit `.env.local` separately on a host. Change it in Git so every machine receives the same effective configuration.

Every new live ROS terminal must load settings in this order:

PC:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/SIMBA/.env.local
source /path/to/SIMBA/install/setup.bash
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

Only one of `manual.launch.py`, `slam.launch.py`, and `coverage.launch.py` may run at a time.

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
source /path/to/SIMBA/.env.local
source /path/to/SIMBA/install/setup.bash
ros2 run simba_bringup keyboard_teleop --config-dir /path/to/SIMBA/src/simba_bringup/config
```

Keys: `W/S/A/D` move, `X` or Space stops, and `Q` stops, releases ownership, and exits. Do not add backslashes before underscores or `--config-dir`.

If ownership is refused, inspect before resetting:

```bash
ros2 topic echo /safety/state --once --full-length
ros2 topic echo /hazard_detection --once --full-length
ros2 topic echo /wheel_status --once --full-length
```

Only after hazards are clear and the robot is stopped:

```bash
ros2 service call /safety/reset std_srvs/srv/Trigger '{}'
```

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
source /path/to/SIMBA/.env.local
source /path/to/SIMBA/install/setup.bash
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
scp -o HostKeyAlias=create3-pi.local -r create3-pi@create3-pi.local:/home/create3-pi/SIMBA/maps/ROOM_NAME /path/to/SIMBA/maps/
```

## Offline coverage preview

This is a static geometric estimate, not a closed-loop simulation. DDS is not required.

```bash
cd /path/to/SIMBA
source /opt/ros/jazzy/setup.bash
source /path/to/SIMBA/install/setup.bash
ros2 run simba_coverage coverage_preview --map /path/to/SIMBA/maps/room_20260923_1338/map.yaml --config-dir /path/to/SIMBA/src/simba_bringup/config --output /path/to/SIMBA/output/preview-review
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
source /path/to/SIMBA/.env.local
source /path/to/SIMBA/install/setup.bash
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

## Drive an exported coverage_tool plan

`tools/coverage_tool` plans a path on the PC and exports it as a `.yaml` file. Launching coverage with `plan:=` makes the supervisor drive that path instead of planning on the robot. The velocity gate, localization trust, bump handling, battery and deadline checks work exactly as in fixed-map coverage. No `approved_hash` is needed: the plan file passed at launch is the approval.

With `plan:=` the launch also starts Nav2's `bt_navigator`. The supervisor splits the plan into chunks of `through_poses_chunk_m` (6 m), adds a pose every `through_poses_spacing_m` (0.1 m) along each chunk, and sends each chunk as one **NavigateThroughPoses** goal, while it holds the NAV lease:

- Nav2 replans through the remaining poses once per second (`config/navigate_through_poses.xml`), so the robot drives around obstacles that the LiDAR adds to the costmaps. A pose counts as reached once the robot is within 0.10 m of it.
- Poses the robot has already passed are dropped. Nav2 only drops a pose within 0.10 m of the robot, so a pose passed just outside that radius would pull the robot back. Once the robot has reached a pose of the chunk, the supervisor projects it onto the next `plan_pass_window_m` (1 m) of the plan (it must be within `plan_pass_lateral_m`, 0.3 m, of it). Every pose behind that point and more than `plan_pass_min_distance_m` (0.1 m) away is dropped. The supervisor replaces the running goal with the rest of the chunk, at most every `plan_replace_period_s` (1 s). Nav2 takes over the new goal while driving, and the gate's lease epoch stays the same, so the robot does not stop.
- A pose the robot cannot reach is tried first and then skipped on its own: once the robot is within `plan_pose_near_m` (0.5 m) of the next pose, it has `plan_pose_timeout_s` (12 s) to reach it and may turn at most `plan_pose_turn_limit_rad` (4.7 rad, 270°) on the spot. Otherwise the supervisor drops that pose and continues with the next one, again without stopping. It is listed under `temporary_blockages` with reason `PLAN_POSE_SKIPPED` and cause `POSE_TIMEOUT` or `POSE_TURNING`.
- The robot stops briefly at the end of each chunk.
- The tree's recoveries only clear the costmaps. Nav2's spin/back-up behaviors are not used, and no `behavior_server` runs.
- If a goal fails (for example, a pose is blocked by an obstacle), the supervisor retries from the next unreached pose. After `target_attempts` failures at the same pose, it skips that pose and the plan poses within `blockage_radius_m` of it, and continues. Skipped poses are listed in the task report under `temporary_blockages` with reason `PLAN_POSE_SKIPPED`.
- `target_s` is the time allowed to reach the next plan pose, not a whole chunk.
- Pause and resume continue at the next unreached pose.

The plan must be made on the same map that the Pi loads, with **robot radius at least 0.23 m** in coverage_tool. The supervisor refuses a plan that comes within 0.20 m (`body_radius_m` + `planning_padding_m`) of a wall or unknown cell, and a plan whose `map_image_sha256` does not match the loaded map image. The tool checks clearance per grid cell, so 0.20 m in the tool is not enough.

The tracked plan `tools/coverage_tool/examples/map_ME_room1v4_coveragev3.yaml` was made this way for `map_ME_room1v4.yaml`. It starts at x=-0.04, y=1.23.

Workflow for your own room:

1. Make a map with *SLAM with RViz and teleoperation* and save it (`.yaml` + `.pgm`).
2. On the PC, plan in `tools/coverage_tool` (see its README). Keep the robot radius at 0.23 m or more (the default) and export the plan `.yaml`.
3. Try the plan without the robot in the closed-loop simulation:
   ```bash
   cd /path/to/SIMBA && source tools/sim/env.sh && cd tools/sim
   python3 run_sim.py myroom --map /path/to/map.yaml --plan /path/to/plan.yaml --start X Y YAW
   python3 analyze.py runs/myroom
   ```
   See `tools/sim/README.md`. Check that it reaches `PLAN_COMPLETE` without bumps, and look at the coverage map.
4. Copy the map (`.yaml` + `.pgm`) and the plan to the Pi, for example with `scp` to `/home/create3-pi/SIMBA/maps/`. That folder is not tracked by Git. The plan is checked against the map image, so copy the exact files.
5. Launch and start as below.

Update and build the Pi first (see *Git deployment to the Pi*). Then, in a Pi terminal inside tmux:

```bash
tmux new -s simba-plan
source /opt/ros/jazzy/setup.bash
source /home/create3-pi/SIMBA/.env.local
source /home/create3-pi/SIMBA/install/setup.bash
ros2 launch simba_bringup coverage.launch.py config_dir:=/home/create3-pi/SIMBA/src/simba_bringup/config map:=/home/create3-pi/SIMBA/tools/coverage_tool/examples/map_ME_room1v4.yaml plan:=/home/create3-pi/SIMBA/tools/coverage_tool/examples/map_ME_room1v4_coveragev3.yaml output_dir:=/home/create3-pi/SIMBA/output
```

Place the robot undocked, anywhere in the room. It does not have to be at the plan start: the supervisor first drives to it. Start RViz on the PC with `mode:=coverage`, set the initial pose, and wait until `/coverage/state` reports `"trusted": true`. Give AMCL another 20–30 s standing still, until the position variance in `/amcl_pose` (covariance entries 0 and 7) is below about 0.001 m² (σ ≈ 3 cm). Every turn on the spot raises it, and above σ 0.1 m the supervisor pauses with `POSITION_UNCERTAIN`. Then:

```bash
ros2 service call /coverage/preview std_srvs/srv/Trigger '{}'
ros2 topic echo /coverage/state --once --full-length
ros2 service call /coverage/start std_srvs/srv/Trigger '{}'
```

Check that `preview_ready` is `true` before calling start. If it is `false`, the `reason` field names the problem (for example `PLAN_TOO_CLOSE_TO_OBSTACLES` or `PLAN_MAP_MISMATCH`). RViz shows the loaded plan on `/coverage/route`. `pause`, `resume` and `cancel` work as in fixed-map coverage.

When the last pose is reached, the task finishes with reason `PLAN_COMPLETE`. If a validated dock calibration exists, the robot then returns and docks; otherwise it stops where the plan ends. Low battery without a dock calibration pauses the task. The coverage percentage reported by the meter uses SIMBA's coverage disk (`coverage_disk_radius_m`, 0.4 m, the same as coverage_tool's default); it does not decide when the plan ends. The task report in `output_dir` records `plan_file` and `plan_sha256`.

### First run on the robot

Check these the first time, and after Nav2 or configuration changes. Keep a hand on the Create 3's stop button.

1. `ros2 lifecycle get /bt_navigator` reports `active`, and `ros2 action list` shows `/navigate_through_poses`.
2. `ros2 topic info -v /odom`: Nav2's controller and `bt_navigator` subscribe RELIABLE. If the Create 3 publishes best-effort, Nav2 receives no odometry.
3. Turn the robot on the spot with teleop and watch `/amcl_pose`:
   - the position σ (square root of covariance entries 0 and 7) must stay below 0.1 m;
   - messages must keep coming while the robot moves slowly.
   
   The AMCL values in `nav2_params.yaml` (`alpha1`–`alpha5` 0.05, `update_min_d` 0.025, `update_min_a` 0.05) and `nomotion_update_period_s` 0.5 were tuned in simulation only. Raise the alphas if AMCL loses the pose on the real robot.
4. Run a short plan first. Then put a box or a person in the path of the plan and check that the robot goes around or stops, and that the gate reports `OBJECT_PROXIMITY` or `BUMP`.
5. After the run, read `temporary_blockages` in the task report. Many `PLAN_POSE_SKIPPED` entries in one place point at a plan that is too tight there.

### Known limitations

- **Obstacle on a plan pose.** An obstacle that sits *on* a plan pose can be driven into. The pose inside it is not a lethal cell (the LiDAR only sees its front), so NavFn may route in through the unseen side. The costmaps also ignore LiDAR hits closer than 0.2 m (`obstacle_min_range`). The Create 3's IR proximity sensors and bumper stop the robot, and the supervisor pauses after a repeated bump. Nav2 jazzy has no `RemoveInCollisionGoals` node; a costmap check in the supervisor would fix this.
- **Hairpin tips.** Regulated Pure Pursuit aims 0.2 m ahead (`lookahead_dist`), so the robot turns before the tip of a hairpin. The pose there is then skipped after `plan_pose_timeout_s`, and a little floor near the tip is missed.
- **Resume near walls.** Resume and the start of each chunk check that the robot is within `planner_start_tolerance_m` (0.15 m) of a safe cell. Close to a wall this can refuse with "Actual pose exceeds planner_start_tolerance_m". Move the robot away from the wall with teleop and resume.
- **Simulation only.** The plan-driving behaviour (0.10 m radius, dropping passed poses, skipping) and the AMCL tuning have been validated in `tools/sim`, not yet on the robot.

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
