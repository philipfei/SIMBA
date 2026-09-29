"""Regression / acceptance test. Run from the project root:

    python3 tests/regression.py            # all cases (~8-10 min)
    python3 tests/regression.py --quick    # first 2 cases

For every case it checks:
  - open floor missed: reachable cells OUTSIDE the edge band that no path covers,
    re-checked independently (every uncovered cell against every safe robot pose,
    with line of sight). Must be 0.
  - edge cells left: missed cells inside the edge band (allowed by design, reported).
  - unsafe: path or connector segments that fail the robot-radius collision check. Must be 0.
"""
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402
from scipy.ndimage import label  # noqa: E402

from coverage import pipeline, planning  # noqa: E402
from coverage.grid import load_ros_map  # noqa: E402
from coverage.settings import default_settings  # noqa: E402

EX = ROOT / 'examples'
MAPS = {'bumpy': EX / 'bumpy.yaml', 'office': EX / 'office.yaml', 'hall': EX / 'rotated_hall.yaml',
        'clutter': EX / 'clutter.yaml'}
CASES = [  # map, robot radius, coverage radius, pattern mode
    ('bumpy', 0.25, 0.30, 'auto'), ('bumpy', 0.25, 0.25, 'auto'),
    ('office', 0.25, 0.25, 'auto'), ('office', 0.30, 0.15, 'regions'),
    ('hall', 0.25, 0.25, 'spiral'), ('clutter', 0.20, 0.25, 'auto'),
    ('clutter', 0.30, 0.15, 'auto'), ('clutter', 0.15, 0.30, 'regions'),
]


def start_in(g, r):
    ok = g.passable(r)
    lab, _ = label(ok)
    big = np.bincount(lab.ravel())[1:].argmax() + 1
    c = np.argwhere(lab == big)
    return g.world(c[len(c) // 2])


def run(case):
    m, rr, rc, mode = case
    g = load_ros_map(str(MAPS[m]))
    s = default_settings()
    s['geometry'].update(robot_radius_m=rr, coverage_disk_radius_m=rc)
    s['strategy']['mode'] = mode
    r = pipeline.plan(g, start_in(g, rr), None, s)
    S = r['stats']
    bad = sum(not g.segment_safe(a, b, rr) for t in r['targets']
              for a, b in zip(planning.points_of(t), planning.points_of(t)[1:]))
    bad += sum(not g.segment_safe(a, b, rr) for c in r['connections'] for a, b in zip(c, c[1:]))
    edge = g.clearance < s['cost']['edge_band_m']
    miss = np.argwhere(r['region'] & ~r['covered'] & ~edge)
    P = g.world(np.argwhere(r['reachable']))
    open_missed = 0
    for mm in miss:
        xy = g.world(mm)
        near = P[np.linalg.norm(P - xy, axis=1) <= rc + 1e-9]
        if len(near) and planning.visible_many(g, near, np.tile(xy, (len(near), 1))).any():
            open_missed += 1
    edge_left = S['uncovered_reachable_cells'] - S['uncovered_interior_cells']
    ok = open_missed == 0 and bad == 0
    print(f"{'PASS' if ok else 'FAIL'} {m:8} robot {rr} brush {rc} {mode:8}: open floor missed {open_missed}, "
          f"edge cells left {edge_left}, unsafe {bad}, stops {S['sharp_turns']}, cost {S['cost_total_s']:.0f}s, "
          f"{S['planning_time_s']:.0f}s -> {S['pattern']}", flush=True)
    return ok


if __name__ == '__main__':
    cases = CASES[:2] if '--quick' in sys.argv else CASES
    results = [run(c) for c in cases]
    print(f'{sum(results)}/{len(results)} passed')
    sys.exit(0 if all(results) else 1)
