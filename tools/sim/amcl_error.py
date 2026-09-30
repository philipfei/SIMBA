"""AMCL position error against ground truth for one run; the clock offset between both logs is fitted.

    python3 amcl_error.py runs/<name> [FROM-TO ...]     e.g. 30-90 90-200 (seconds of the run)
"""
import csv
import json
import sys
from pathlib import Path
import numpy as np

if len(sys.argv) < 2:
    sys.exit(__doc__)
run = Path(sys.argv[1])
a = np.array(json.load(open(run / 'amcl.json')))
rows = list(csv.DictReader(open(run / 'truth.csv')))
T = np.array([[float(r['t']), float(r['x']), float(r['y'])] for r in rows])
best = None
for off in np.arange(-15, 15, .05):
    idx = np.clip(np.searchsorted(T[:, 0], a[:, 0] + off), 0, len(T) - 1)
    e = np.linalg.norm(a[:, 1:3] - T[idx, 1:3], axis=1)
    if best is None or np.median(e) < best[0]:
        best = (np.median(e), off, e)
med, off, e = best
print(f'clock offset {off:.2f} s  median AMCL error {med:.3f} m  p90 {np.percentile(e, 90):.3f} m  max {e.max():.3f} m')
for lo, hi in [(float(x), float(y)) for x, y in (s.split('-') for s in sys.argv[2:])]:
    m = (a[:, 0] > lo) & (a[:, 0] < hi)
    if m.any():
        print(f'{lo:.0f}-{hi:.0f} s: median {np.median(e[m]):.3f}  p90 {np.percentile(e[m], 90):.3f}  '
              f'max {e[m].max():.3f}  (n={m.sum()})')
