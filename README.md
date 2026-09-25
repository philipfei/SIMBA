# SIMBA

Create 3 + RPLIDAR A2M8 + Raspberry Pi 5, using ROS 2 Jazzy on Ubuntu 24.04.
Edit source here. The Pi is a deployment mirror. Do not edit its generated install.
No launch starts a mission. Robot navigation and docking remain **hardware-unverified**.

## Setup [PC]

```bash
cd /home/philip/Documents/SIMBA
source modules/common/env.sh offline
python3 tools/check_dependencies.py --role pc
```

[requirements.yaml](requirements.yaml) records exact observed versions and their
sources. Mismatches, missing packages and unverified versions are reported, not
fixed. PC extracted test packages are not system installations. Pi versions are
historical until checked on the connected Pi. Never install an amd64 package on arm64.

Install missing packages deliberately using Ubuntu/ROS apt packages. For example,
with Jazzy's apt repository already configured:

```bash
sudo apt update
sudo apt install python3-numpy python3-yaml python3-pil python3-matplotlib \
  python3-pytest python3-colcon-common-extensions python3-rosdep cmake \
  rsync openssh-client ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rviz2
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
./tools/test.sh
```

`rosdep` may install runtime packages as well as build dependencies. Review its
output. Use a new terminal for robot work: offline tests use domain 91 and localhost;
robot operation uses domain 0 and subnet discovery. Do not mix these environments.

## SSH, paths and source updates [PC]

```bash
ssh create3-pi@create3-pi.local
# Only after verifying the current IP belongs to the Pi:
ssh -o HostKeyAlias=create3-pi.local create3-pi@VERIFIED_IP
```

Do not assume an old DHCP address is still the Pi. Verify changed SSH fingerprints
before removing an obsolete key with `ssh-keygen -R create3-pi.local` (and the verified
IP entry if needed). Never disable host-key checks. Keep passwords and keys out of Git;
change a password if it was exposed. Use `exit` to close SSH.

| Pi path | Use |
|---|---|
| `/home/create3-pi/create3_ws/src/create3_lidar_bringup` | Source mirror |
| `/home/create3-pi/create3_ws/install` | Active, managed install link |
| `/home/create3-pi/create3_ws/.simba-deploy/candidates` | Isolated versions and logs |
| `/home/create3-pi/create3_ws/maps` | Saved maps; never deleted by deployment |
| `/home/create3-pi/create3_ws/coverage_reports` | Runtime coverage reports and calibration |

```bash
ssh create3-pi@create3-pi.local 'ls -lh /home/create3-pi/create3_ws/maps'
./modules/deployment/sync.sh          # Read-only remote preflight and rsync differences
./modules/deployment/sync.sh --apply  # Sync, SHA-256 audit, build, test, check, activate
```

Edit `modules/deployment/config.yaml` for the SSH host and failed-candidate retention
(default 3). Use `scp` for unrelated documents, not to bypass source deployment:
`scp LOCAL_FILE create3-pi@create3-pi.local:/home/create3-pi/`;
reverse source/destination to download a file. Deployment never starts robot motion.

A failed apply prints its stage and logs. Source may have changed; the previous
working install remains active. Fix and explicitly retry, or inspect/revert source
manually. There is no automatic source rollback. An old normal `install/` directory
or a symlink-install pointing into mutable source causes a preflight refusal;
see [maintenance guidance](docs/development.md#deployment-maintenance).

For the first migration of an old symlink-installed workspace, first stop keyboard
control and all project launches. Then run on the PC:

```bash
./modules/deployment/migrate_legacy.py                 # Read-only inspection
./modules/deployment/migrate_legacy.py --apply --stopped
./modules/deployment/sync.sh --apply
```

`--stopped` explicitly confirms that maintenance can proceed. The migration snapshots
the **Pi's existing source**, rebuilds an independent copy of that old version, verifies
old executable/configuration bytes, then atomically exchanges the directory and a
managed install link using Linux `renameat2(RENAME_EXCHANGE)`. The original install
is retained as `.legacy-install-<candidate>/`; original build and maps are untouched.
Only after successful validation does it enable owner-write on source-mirror directories
needed for later rsync. No new SIMBA source is deployed until the final sync command.
Build/validation failures leave the original installation in place. Read the printed
log path if a stage fails; the helper does not launch nodes or install system packages.


**After every successful apply, open a NEW clean SSH session/terminal and source
ROS and the resolved workspace setup before launching anything.** Do not reuse a
shell that loaded the previous overlay. Running processes are not updated or restarted.
Old versions remain available. PC module commands use a clean, pinned remote environment.

## Robot environment and basic commands [Pi]

Use a fresh shell. The current robot uses no ROS namespace prefix. Verify interfaces before setting DDS:

```bash
source /opt/ros/jazzy/setup.bash
ip -br addr
installed=$(readlink -f /home/create3-pi/create3_ws/install)
source "$installed/setup.bash"
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="10" multicast="default"/><NetworkInterface name="wlan0" priority="20" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
ros2 topic list -t
ros2 topic echo --once /battery_state
ros2 topic echo --once /dock_status
ros2 action list -t
```

Battery updates may take about 5 seconds. Topic names depend on firmware; inspect
available interfaces first. `ros2 interface show irobot_create_msgs/action/DriveDistance`
and `.../RotateAngle` show definitions; they are not additional manual-control paths.

Native actions below **move the robot**. First stop teleop, revoke manual ownership,
stop project launches, and verify the robot is stationary with no external velocity
publisher. Do not send them alongside an active coverage manager:

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}'
ros2 action send_goal /dock irobot_create_msgs/action/Dock '{}' --feedback
```

Alternatively [PC], `./modules/robot/dock.py undock` or `dock` checks for project
controllers/publishers before sending the action. Dock must be powered and nearby.
During a coverage task use `/coverage/return_to_dock`, not a competing native action.
`./modules/robot/status.sh` reads the battery from the PC.

## Manual control [PC]

1. If docked, explicitly undock using the independent native-action procedure above.
2. Start **one** sensor/safety launch: `./modules/teleop/start.sh`. For mapping,
   use the mapping launch below instead; do not start both.
3. In another terminal grant control, then start the keyboard:

```bash
./modules/common/pi.py ros2 service call /control/manual std_srvs/srv/SetBool '{data: true}'
./modules/teleop/keyboard.sh
```

W/S/A/D request forward/reverse/turn; X or Space stops; Q stops and exits. Keep
keyboard focus. Key release or lost command stream stops through timeouts.
Manual limits are +0.25/-0.15 m/s and 0.50 rad/s. Do not publish directly to
`/cmd_vel` or run concurrent Twist/TwistStamped publishers.

Finish with Q, then revoke ownership and stop the launch with Ctrl+C:

```bash
./modules/common/pi.py ros2 service call /control/manual std_srvs/srv/SetBool '{data: false}'
```

## Mapping and map download [PC]

1. Undock explicitly if needed. Stop any fixed-map AMCL/navigation launch.
2. Start `./modules/mapping/start.sh` (Pi SLAM, LiDAR, TF relay and safety gate).
3. Open `./modules/mapping/rviz.sh` in another terminal. Default PC interface:
   `enx00e17c6840b1`; override `SIMBA_PC_INTERFACE` only after checking `ip -br addr`.
4. Grant manual ownership and use the keyboard as above. Drive slowly and verify
   scan points stay on walls during turns; duplicated/bent walls indicate a problem.
5. Stop the robot, leave SLAM running, then save into a **new** map directory:

```bash
./modules/mapping/save_map.py --name room_NEW_TIMESTAMP
# If already saved on the Pi, download without saving again:
./modules/mapping/save_map.py --name room_NEW_TIMESTAMP --download-only
```

The script calls SLAM Toolbox `save_map` and `serialize_map`, downloads the four
files and checks SHA-256 before publishing the local folder. Existing local maps
are never overwritten. A failed transfer can be retried with `--download-only`.
`map.yaml` + `map.pgm` serve fixed-map localization; `.posegraph` + `.data` retain
SLAM state. Keep all four. Results are in `maps/NAME/` on this PC.

For direct [Pi] saving after creating a new directory:

```bash
mkdir /home/create3-pi/create3_ws/maps/room_NEW_TIMESTAMP
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/create3-pi/create3_ws/maps/room_NEW_TIMESTAMP/map'}}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/create3-pi/create3_ws/maps/room_NEW_TIMESTAMP/map'}"
```

Finish teleop, revoke control and stop the mapping launch. Verify wheel speeds are
zero before the independent Dock procedure. Never infer fresh scans from a process
or publisher count: check `ros2 topic hz /scan` and message timestamps.

## Offline coverage preview [PC; no robot]

```bash
source modules/common/env.sh offline
./modules/coverage/preview.py \
  --map maps/room_20260923_1338/map.yaml --output reports/current_preview
```

The current map has the registered example start `(0.807, -1.575)` m. This is not a
measured robot pose. For a **new map**, first choose a safe map-frame start in metres:

```bash
./modules/coverage/register_map.py --map maps/NEW_MAP/map.yaml --start X Y
./modules/coverage/preview.py --map maps/NEW_MAP/map.yaml --output reports/new_preview
```

No registered start means no evaluation. `--update` permits an intentional change
only after typing `UPDATE` in an interactive terminal. It records old/new starts
and hashes and marks old results superseded. Use a new output folder for each run.

```bash
./modules/coverage/preview.py --map maps/NEW_MAP/map.yaml \
  --check-result reports/new_preview/summary.json
```

Outputs: PNG/SVG, world-coordinate route JSON, all candidate scores, summary and
coverage masks. Labels are English. Green is **ideal**, not measured, coverage;
yellow is remaining coverable area. The map is not modified. No dock pose or return
route is invented. Use `--baseline CURRENT_REGISTERED_ROUTE_JSON` for four-panel mode.
Historical baselines without registration hashes are rejected, not silently repaired.
See `examples/README.md` for preserved, historical-only results.

Change settings in `src/create3_lidar_bringup/config/coverage_settings.yaml`, then
regenerate preview. Defaults: 0.20 m collision radius, 0.25 m virtual coverage disk,
0.35 m stripes, 0.12 m/s and 0.40 rad/s autonomous caps, 90% target, up to 3 resweeps.
The larger virtual disk intentionally credits wall-adjacent floor; it is not a
physical cleaning-tool claim. Boundary following never cancels the main sweep;
completion is checked at phase boundaries. Fixed-map AMCL and mapping SLAM are
separate modes. Fixed-map preview is geometric simulation, not a dynamics simulator.

<a id="fixed-map-operation-and-todo-pcpihardware-unverified"></a>

## Fixed-map operation and TODO [PC/Pi; hardware-unverified]

First verify stationary sensors, TF, localization and command ownership; then test
short point-to-point navigation and pause/cancel with an operator present. Complete
an independent docking calibration/trial before a full coverage mission, since
`/coverage/start` requires validated dock metadata. Proceed to a small area, then a room.

[PC] `./modules/navigation/start.sh`, then `./modules/navigation/rviz.sh`.
Launch does not move the robot. Use RViz **2D Pose Estimate** for the actual pose.
For a short test, RViz **2D Goal Pose** stores a goal; explicitly call
`/coverage/test_navigation` (Trigger) to execute it.

[Pi] Select a region using `region_begin`, RViz **Publish Point** in boundary order,
then `region_commit`. Preview before explicitly starting:

```bash
ros2 service call /coverage/region_begin std_srvs/srv/Trigger '{}'
ros2 service call /coverage/region_commit std_srvs/srv/Trigger '{}'
ros2 service call /coverage/preview std_srvs/srv/Trigger '{}'
ros2 service call /coverage/start std_srvs/srv/Trigger '{}'
ros2 service call /coverage/pause std_srvs/srv/Trigger '{}'
ros2 service call /coverage/resume std_srvs/srv/Trigger '{}'
ros2 service call /coverage/cancel std_srvs/srv/Trigger '{}'
```

Connections may use safe known floor outside the selected region. Trigger services
`calibrate_docked`, `calibrate_dock_body`, `calibrate_staging` record the actual docked
pose, explicitly selected dock-body polygon, and a stationary, undocked staging pose
with the dock visible. Recalibration invalidates approval; explicit `return_to_dock`
must succeed before full coverage start. Metadata is separate `dock_<map hash>.yaml`;
the map origin is not the dock and new maps need calibration.

The Pi runs supervisor, meter and safety gate. PC loss does not interrupt autonomy;
manual commands time out and no new PC commands arrive until reconnection. Controller
output is `/cmd_vel_nav`, manual input `/cmd_vel_remote`; only the gate publishes
`/cmd_vel`. Pause retains the task ID, blacklist, temporary blocks and deadline.
Restart does **not** resume. After a serious fault, remove the cause, call
`/safety/reset` (Trigger), then explicitly resume; reset is not resume.

Runtime trust is checked at 10 Hz: position sigma <=0.10 m, yaw sigma <=10 degrees,
>=60 laser endpoints with >=65% within 0.15 m of a known obstacle, fresh scan/odom/TF,
and AMCL age <=2 s. Two stable seconds are required. Failure stops motion and
measurement; only trusted actual poses paint coverage, never accepted goals.

Timeouts: mission 1800 s (including pauses/planning/resweeps), target 60 s, planning
5 s, return 120 s, plus independent no-progress checks. Overall timeout stops and
reports residual area without automatic retry/return. Battery: start >=30%, return
<=25%, critical stop <=15%, freshness 15 s. Native Dock/Undock use silent ownership
handoff; cancellations require acknowledgement.

Preserve `backup_only`, enabled native bump reflexes and the selected periodic hazard
publication (100 ms); this software does not write robot settings. Verify live values
in the robot web UI, Application -> Configuration. BUMP/BACKUP_LIMIT are not
CLIFF/STALL/WHEEL_DROP; never fight bump reflexes with blind reversing. `/stop_status`
means motionless, not emergency stop; disabled wheels are a separate condition.

TODO: real Nav2 validation; return-to-staging and native docking validation; charging
confirmation; persistent checkpoints; charge/undock/relocalize/resume states. Current
low-battery return ends the task. No automatic charge-and-resume or restart resume
exists. Details and acceptance criteria: [development.md](docs/development.md).

## Diagnostics and physical checks

Before motion verify operator stop access, payload clearance, no stairs/drops,
unchanged dock position, stable USB and power. Use a proper Pi 5 supply; never connect
raw robot battery voltage to Pi 5 V pins. Confirm the robot USB/BLE switch is set to
USB. Firmware I.0.0 CycloneDDS is historical: verify the installed firmware in the web
UI, not from this README. Robot link: Pi eth0 `192.168.186.3/24` to robot `192.168.186.2`.

LiDAR: A2M8, 115200 baud, stable path
`/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`.
TF is `map -> odom -> base_footprint -> laser`; LiDAR z=0.0994 m, yaw=pi, inverted=false,
angle compensation enabled. Recheck payload/TF/timestamps; do not invent a base_link
mount. Typical prior scans were 14–15 Hz. Low, thin, glass or overhanging objects may
not be visible at this scan height. Never run two map->odom publishers.

<details>
<summary>System, network, ROS and USB checks</summary>

```bash
cat /etc/os-release
uname -m
dpkg --print-architecture
hostname
whoami
hostname -I
ip -br addr
ip -br link
ip route
ip route get 192.168.186.2
ping -c 4 create3-pi.local
ping -c 4 192.168.186.2
curl -I http://192.168.186.2
df -h
free -h
uptime
cat /sys/class/thermal/thermal_zone0/temp  # Millidegrees Celsius on Pi
systemctl status ssh
sudo ss -lntp
networkctl status eth0
sudo netplan get
ls -l /etc/netplan
sudo ufw status verbose
journalctl -p warning -b
journalctl -f
sudo journalctl -u ssh -b
sudo dmesg --follow
lsusb
ls -l /dev/serial/by-id/
env | grep -E 'ROS|RMW|CYCLONE'
ros2 pkg list
ros2 pkg executables
apt list --installed
ros2 interface list
ros2 node list
ros2 topic list -t
ros2 topic list --no-daemon --spin-time 10
ros2 service list -t
ros2 action list -t
ros2 topic info /odom -v
ros2 topic info /cmd_vel -v
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic bw /scan
ros2 topic echo --once /odom
```

Use `ros2 topic echo TOPIC` for available `/imu`, `/hazard_detection`, `/cliff_intensity`,
`/wheel_status`, `/wheel_ticks`, `/mouse`, `/interface_buttons`, `/tf`, `/tf_static`.
Stop continuous displays with Ctrl+C. Restart stale discovery with `ros2 daemon stop`
and `ros2 daemon start`, then recheck the graph. Ping alone does not establish DDS or SSH
identity. Check domain, RMW, interface routing, firmware, application restart after
manual configuration changes and firewall when topics are missing. RViz uses
`-r /tf:=/tf_relay -r /tf_static:=/tf_static_relay`.

If needed, deliberately enable SSH with `sudo systemctl enable --now ssh`. Configure
networking only during maintenance: eth0's historical static address is
192.168.186.3/24, DHCP disabled, optional=true. Back up Netplan, use `sudo netplan generate`
and `sudo netplan try` before `sudo netplan apply`; never apply blindly over the only
SSH connection. Check cables/adapters when the robot link fails.

Use `passwd` to change the login password. Stop robot work before `sudo reboot` or
`sudo poweroff`. Package upgrades are maintenance operations, not a troubleshooting
autofix; review updates rather than running unattended `full-upgrade -y`.
</details>

Local-only delivery comes first. A private GitHub repository named SIMBA will be
created and populated only after review; no remote repository is created by these tools.
