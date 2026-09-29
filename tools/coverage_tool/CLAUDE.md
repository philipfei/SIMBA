# Coverage path planner — project context for Claude Code

Owner: Dielis (TU Delft, JIP project). Python desktop tool that plans full-area coverage paths for a
differential-drive robot (iRobot Create3, ROS 2 / Nav2) on a ROS lidar occupancy map (`.pgm` + `.yaml`)
and exports Nav2 waypoints (YAML/JSON). Runs in WSL (Ubuntu) on Dielis's laptop.

## Run

```bash
python3 -m pip install -r requirements.txt      # numpy scipy matplotlib contourpy pyyaml pillow (NO scikit-image)
python3 app.py [map.yaml]                          # Tkinter GUI (needs python3-tk)
python3 plan_cli.py map.yaml --start X Y -o plan.yaml --png plan.png
python3 tests/regression.py [--quick]              # acceptance test, run after every planner change
```

## Architecture (`coverage/`)

| File | Role |
|---|---|
| `pipeline.py` | `plan()`: reachability → build candidates → `_finish` (centre-line fill, local search, sanitize, connect, exact cost) → cheapest wins → extra polish. `brush_reachable()` = exact reachable-cell definition. |
| `cost.py` | The cost function (seconds): drive, bends (`bend_s_per_rad`), stop+rotate above `smooth_turn_deg`, hairpins, overlap (FLOOR swept twice only; brush over obstacles is free), missed (heavy), missed edge cells (light). |
| `localsearch.py` | `RouteOptimizer`: exact-Δcost local search over pieces — drop, trim, straighten, flip, move, block-move, 2-opt, detour; then perturb-and-reoptimise until the time budget ends. |
| `optimizer.py` | Wall loops, boustrophedon region decomposition, per-region lanes vs spiral vs clipped rings (`plan_region`), `plan_regions` (rims on/off, ring_targets), `plan_hybrid`, lane generation with inward shift along bumpy edges. |
| `spiral.py` | Distance-field rings (contourpy), spiral tree/ordering, centre-line fill (hand-written Zhang–Suen skeleton), obstacle rims. `WALL_LEVEL=0.49`, `SIMPLIFY=0.2` (x cell). |
| `grid.py` | Map loader (map_server conventions), clearance map, `segment_safe`, `reachable_from` (4-connected on purpose), A* (8-dir, no corner cutting). |
| `planning.py`, `planner.py` | Dielis's original modules; still used for `stroke_union`, `visible_many`, `connect`, `split_path`, `poly_target`. Only change: 2-D cross product fix for NumPy 2. |
| `export.py`, `render.py`, `settings.py` | Nav2 export, drawing, defaults (`DEFAULTS`) + JSON load/save. |

Candidates in Auto mode: Spiral around obstacles, Spiral outer walls only, Mixed regions (2 split angles),
Straight lanes one direction, Wall loop + boustrophedon cells (2 angles), Rings or lanes per region.
Turning cost is curvature-based (cost.vertex_cost / corner_radius), resolution independent. Export = pose at
every path vertex (<= 0.5 m apart), sent with NavigateThroughPoses.

## Design decisions Dielis asked for (keep them)

- **Everything is decided by the cost function, nothing hard-coded.** Full coverage emerges from a very heavy
  `missed_weight` (10000 ≈ 167 s per 5 cm cell), not from a forced rule. Do not add "always cover" flags.
- **Robot radius = collision/safety distance; coverage radius = brush/spacing.** Spacing = 2·r_cov·(1−overlap),
  overlap 0–50 %, computed automatically.
- **Reachable cells are exact:** robot poses with clearance ≥ robot radius, 4-connected to start; a cell is reachable
  if some pose is within r_cov with line of sight. Verified against brute force — keep it that way.
- **Straight lines as much as possible.** Brush overlapping walls/obstacles is fine.
- **Missed cells must be very expensive** (Dielis checks for red pixels). `edge_missed_weight` defaults to 10000
  (strict); lowering it trades cells within `edge_band_m` of a wall for straighter lines.
- Path shape is free (spiral, lanes, mixed) — whatever is cheapest. Wall loop first by default.
- Obstacle handling (loop around vs outer walls only) is a user option.

## Gotchas

- Single-goal pieces (detours) have a heading `Piece.h` set by `orient()`/`_slot_cost()`: the turn at a point
  is charged at its junctions. Without it, U-turns at detour points look free and the order gets bad.

- `grid.astar`: `gr, gc = g; sq2 = math.sqrt(2)` must come BEFORE the `moves = [...]` list (UnboundLocalError twice).
- `reachable_from` must stay 4-connected to match A* (no diagonal squeezing).
- NumPy 2: `np.cross` rejects 2-D vectors — use explicit `a[0]*b[1]-a[1]*b[0]`.
- Don't use scikit-image (not installed on Dielis's machine).
- Planning time is budget-driven (4 s/candidate + 15 s polish), results vary slightly run to run.

## Acceptance criteria (tests/regression.py)

Every case: 0 open-floor reachable cells missed (independent brute-force check), 0 unsafe segments.
Edge cells left are reported, allowed by design. Last run: 8/8 pass, 0 cells missed, 25–95 s per case.

## Status / next steps

- Only tested on synthetic maps in `examples/` — **not yet on the real map or robot.** First task: run on the real map.
- On the SIMBA robot the export is driven by `coverage.launch.py plan:=FILE` (the Pi's coverage_supervisor loads it,
  `src/simba_coverage/simba_coverage/plan_file.py`). SIMBA refuses plans closer than 0.20 m to walls with an exact
  capsule check, so plan with robot radius >= 0.23 m. `map_image_sha256` in the export lets the robot reject plans
  made on another map. `examples/map_ME_room1v4_coveragev3.yaml` is the 0.23 m plan for the real room map.
- `send_to_nav2.py` only fits a plain Nav2 stack (SIMBA runs no bt_navigator). Untested. Before trusting it:
  same map in Nav2 as in the planner, set initial pose (`nav.setInitialPose`), match Nav2 `robot_radius`/inflation
  to the planner's robot radius, check goal tolerances vs waypoint spacing, consistent `use_sim_time`.
- Calibrate `stop_penalty_s`, `bend_s_per_rad`, `angular_rad_s` from real Create3 logs.
- Planning speed (8 candidates ≈ 1–2 min).
