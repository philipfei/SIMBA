# PC validation and planning review

Validation date: 2026-09-24. No Pi/robot connection, deployment, native settings change or physical motion.
Build succeeded. **66 pytest cases passed**; colcon also counts three CTest suite results (69 total, zero errors/failures/skips).
Installed/source manifests agree across 35 source files; 28 installed module/script/resource contents were independently SHA-256 checked. All 8 original map files match the pre-implementation baseline.

## Comparison

All four panels use recorded example start map (0.807, -1.575) m, the whole-map selection, the same reachable component and the same 16.5575 m² denominator. The 90% phase-end completion policy is applied consistently. Percentages are ideal geometric unions, not actual robot measurements.

| Variant | Path (m) | Ideal union | Remaining (m²) | Time estimate (min) |
|---|---:|---:|---:|---:|
| 1  Original planner | 97.40 | 96.8% | 0.5250 | 39.32 |
| 2  Equal spacing | 60.60 | 91.0% | 1.4950 | 16.22 |
| 3  Wall-aligned candidates | 60.31 | 92.9% | 1.1825 | 15.88 |
| 4  Boundary + interior | 62.55 | 100% | 0.0000 | 18.31 |

Time estimates include path length/speed and heading change/angular cap, but exclude acceleration, controller delays and native docking. They are not execution-time guarantees.
The original panel replays its saved main route under the common stopping policy; its earlier 125.1 m report also included a residual pass and is not the baseline length used here.

## Wall evidence and ranking

Selected stripe direction: 5.71768045 deg; phase: 0.20 m. Dominant wall cluster: 5.71768045 deg. Next cluster: None (None means no distinct supported second cluster).
Separated long-wall segment angles: 95.50 deg, 96.00 deg. Individual wall evidence is retained in comparison.json; a single global direction does not imply parallel side walls.
The aligned run evaluated 28 original direction/offset candidates. One global Nmax=6079 and T=6 were used. Winner selection: exact_maximum. All raw scores, eligibility and rejection reasons are in candidates.json.

## Artifacts and configuration

- comparison.png / comparison.svg: English-labelled four-panel view, including measured stripe dimension annotations.
- routes.json / masks.npz: map-frame routes and per-panel covered/remaining masks.
- comparison.json / candidates.json / validation.json: effective values, hashes, wall evidence, candidate scores and checks.
- Source config: create3_lidar_bringup/config/coverage_settings.yaml. Source fixture registry: config/offline_map_cases.yaml.

## Tested behavior and remaining validation

Tests cover global tolerance ranking, deterministic short-fragment chains, canonical config hashing and stale-preview rejection, registered starts, trapezoidal/rotated walls, metric stripe phase, selected-region isolation, safe boundary polylines, open-arc goals, actual-pose-only measurement, phase acknowledgements, pause recovery, command ownership, hazards and sensor dropout.
PC ROS tests use loopback-only domain 91 with fake sensors/actions. Real Nav2 tracking/map reload, Pi timing, stopping distance, USB stability, native cancellation and docking remain unverified. Closed loops are split into open Nav2 targets; no robot was used to validate their physical execution.
The unpacked project-local test dependency prefix produces a harmless colcon local_setup warning; this is not a Pi deployment environment.
Keep staged physical acceptance separate: stationary localization/ownership, short navigation and cancel/pause, bounded region, full room, then independently validated docking.
