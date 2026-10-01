# Closed-loop simulation

Drives an exported coverage_tool plan through the **real** SIMBA stack on a PC, without the robot:
- `coverage.launch.py plan:=…` with Nav2 (map server, AMCL, planner, controller, `bt_navigator`);
- the coverage supervisor, coverage meter and velocity gate;
- a fake Create 3 and LiDAR in place of the hardware.

Use it to check a plan, a config change or a code change before taking it to the robot.

| File | Purpose |
|---|---|
| `fake_create3.py` | Kinematic Create 3: drives on `/cmd_vel`, publishes `/odom`, TF, a ray-cast 300-beam `/scan`, hazards (bump), battery, wheel and dock status, serves `/e_stop`. Logs the true pose. |
| `run_sim.py` | Starts the launch and the fake robot and acts as the operator (initial pose, preview, start, resume after listed pauses). Writes everything to `runs/<name>/`. |
| `analyze.py` | `summary.json` and `plot.png` for one run. |
| `coverage_map.py` | Covered vs. missed floor per run, next to the plan followed exactly, using SIMBA's coverable area and meter disk. |
| `amcl_error.py` | AMCL position error against the true pose. |
| `env.sh` | ROS environment for the simulation (own domain 77, localhost only, FastDDS). |

**What is simulated, and what isn't**
- Walls come from the map image. `--obstacle X,Y,R` adds round obstacles that exist in the world but not in the map Nav2 loads.
- Hitting a wall or obstacle stops the fake robot and raises a BUMP hazard.
- **Not simulated:** IR proximity (`OBJECT_PROXIMITY`), odometry drift and wheel slip, and LiDAR motion blur. Odometry is perfect, so AMCL does better than on the robot.
- Everything runs in real time: a full run of the example plan takes about 12 minutes.

## Run it

Build first, from the repository root, as in the main README (*Build*). The simulation also needs `python3-scipy`, which is listed in `requirements.yaml`.

```bash
cd /path/to/SIMBA
source tools/sim/env.sh
cd tools/sim
# Plan map_ME_room1v4_coveragev3.yaml, robot starts at (0.9, -3.0)
python3 run_sim.py clear
python3 analyze.py runs/clear
# Two obstacles that are not in the map: a chair leg next to the path and a box on a plan pose
python3 run_sim.py obstacles --obstacle 0.713,0.762,0.03 --obstacle 1.16,-3.38,0.15
python3 analyze.py runs/obstacles
python3 coverage_map.py runs/compare.png clear obstacles:"with obstacles"
```

**Options of `run_sim.py`** (`--help` for all):

| Option | What it does |
|---|---|
| `--plan`, `--map` | Other plan and map. |
| `--start X Y YAW` | True start pose. |
| `--config DIR` | Another config directory. Copy `src/simba_bringup/config` and edit the copy to try parameters. |
| `--resume-on` | Pause reasons after which it calls `/coverage/resume`. |

Only one simulation can run per DOMAIN_ID. `run_sim.py` refuses to start while another one is running in the same domain. Use `SIM_DOMAIN_ID=78 source env.sh` for a second one, but don't run two at once: the lifecycle start-up then times out.

## Reading the results

`runs/<name>/` contains:

| File | Contents |
|---|---|
| `result.json` | Final supervisor state, timeline, resumes, errors |
| `summary.json` | See the fields below |
| `plot.png` | Plan, driven path, obstacles, skipped poses |
| `truth.csv` | True pose at 10 Hz |
| `amcl.json` | AMCL pose and sigma |
| `feedback.json` | NavigateThroughPoses feedback |
| `tf.json` | `map→odom` timing |
| `launch.log` | Output of all launched nodes |
| `robot.log` | Output of the fake robot |
| `output/task_*.json` | The supervisor's task report |

Key fields in `summary.json`:
- `meter_fraction`: coverage as the robot's meter measured it.
- `bumps`, `min_clearance_m`: collisions and the closest approach to a wall.
- `poses_within_0.10m`: how many plan poses the robot drove within 10 cm of.
- `skipped_poses`: `PLAN_POSE_SKIPPED` poses, each with its cause.
- `in_place_turns_over_90deg`: turning on the spot (lots of it means trouble reaching poses).
- `nav_goals_poses`: pose count of each NavigateThroughPoses goal.

In the coverage map, green is covered, red is coverable but missed, and grey is free floor that no robot position can reach.

Reference, with the repository defaults at the time of writing: the `clear` run finishes the v3 plan in 580–630 s with 93–94 % meter coverage and no bumps. One or two poses are skipped; one is the plan start, a hairpin tip for a robot arriving from the south (see *Known limitations* in the main README).

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `AMCL never published a pose`, or lifecycle errors in `launch.log` | A process from an earlier run is still alive in the same domain, or stale FastDDS shared memory. Check with `pgrep -af "coverage.launch\|fake_create3\|run_sim"`, stop those processes, then run `fastdds shm clean`. |
| Occasional `TF_UNAVAILABLE` / `SCAN_STALE` pauses | Timing hiccups of a loaded PC; `run_sim.py` resumes after them. |
| `test_ros.py` fails to find CycloneDDS | The unit tests use CycloneDDS on loopback; install `ros-jazzy-rmw-cyclonedds-cpp`. |
