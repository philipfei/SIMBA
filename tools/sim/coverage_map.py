"""Covered vs missed floor per run, next to the plan followed exactly.

Uses SIMBA's own coverable area (Grid.reachable + Grid.coverable with the configured collision
radius) and meter disk (coverage_disk_radius_m), applied to each run's ground-truth path, so the
percentages match what the robot's coverage meter reports.

    python3 coverage_map.py OUT.png RUN[:LABEL] [RUN[:LABEL] ...]
    e.g. python3 coverage_map.py runs/compare.png clear obstacles:"with obstacles"
"""
import csv
import json
import sys
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch
from simba_coverage.geometry import Grid
from simba_coverage.params import load_settings

HERE = Path(__file__).resolve().parent


def main(out, runs):
    first = json.load(open(HERE / 'runs' / runs[0][0] / 'result.json'))
    settings = load_settings(first['config'])
    radius = settings['geometry']['coverage_disk_radius_m']
    grid = Grid.load(first['map'])
    plan = np.array([[p['x'], p['y']] for p in yaml.safe_load(open(first['plan']))['poses']])
    h, w = grid.cells.shape
    region = grid.world([[-.5, -.5], [-.5, w - .5], [h - .5, w - .5], [h - .5, -.5]]).tolist()
    reachable = grid.reachable(plan[0], settings.collision, settings['recovery']['planner_start_tolerance_m'])
    coverable = grid.coverable(reachable, region, radius)
    cell = grid.resolution ** 2

    def covered_by(points):
        pts = [points[0]]
        for a, b in zip(points[:-1], points[1:]):
            n = max(1, int(np.ceil(np.linalg.norm(b - a) / (grid.resolution / 2))))
            pts.extend(a + (b - a) * i / n for i in range(1, n + 1))
        hit = np.zeros_like(coverable)
        for p in pts:
            hit |= grid.disk(p, radius)
        return hit & coverable

    panels = [('Plan followed exactly\n(reference)', plan)]
    for name, label in runs:
        rows = list(csv.DictReader(open(HERE / 'runs' / name / 'truth.csv')))
        panels.append((label, np.array([[float(r['x']), float(r['y'])] for r in rows])))
    extent = [grid.origin[0], grid.origin[0] + w * grid.resolution, grid.origin[1], grid.origin[1] + h * grid.resolution]
    # 0 wall/unknown, 1 free but not coverable, 2 missed, 3 covered
    cmap = ListedColormap(['#2b2b2b', '#d9d9d9', '#e0413a', '#4caf50'])
    fig, axes = plt.subplots(1, len(panels), figsize=(4.2 * len(panels), 7.6), squeeze=False)
    ref = None
    for ax, (label, path) in zip(axes[0], panels):
        hit = covered_by(path)
        img = np.where(grid.free, 1, 0)
        img[coverable] = 2
        img[hit] = 3
        missed = (coverable & ~hit).sum() * cell
        extra = '' if ref is None else f'\nvs. exact plan: {(ref & ~hit).sum() * cell:.2f} m² lost'
        ref = hit if ref is None else ref
        ax.imshow(img, cmap=cmap, vmin=0, vmax=3, origin='lower', extent=extent, interpolation='nearest')
        ax.plot(plan[:, 0], plan[:, 1], '-', color='white', lw=.6, alpha=.9)
        if path is not plan:
            ax.plot(path[:, 0], path[:, 1], '-', color='#1f4fd1', lw=.9)
        ax.set_title(f'{label}\ncovered {100 * hit.sum() / coverable.sum():.1f}%  ·  missed {missed:.2f} m²{extra}',
                     fontsize=9)
        ax.set_xticks([])
        ax.set_yticks([])
    fig.legend(handles=[Patch(color='#4caf50', label=f'covered (meter disk r={radius} m)'),
                        Patch(color='#e0413a', label='coverable but missed'),
                        Patch(color='#d9d9d9', label='free, but no robot position can cover it'),
                        Patch(color='#2b2b2b', label='wall / unknown'),
                        plt.Line2D([], [], color='#9e9e9e', lw=2, label='planned path (white on the map)'),
                        plt.Line2D([], [], color='#1f4fd1', lw=2, label='driven path (ground truth)')],
               loc='lower center', ncol=3, fontsize=9, frameon=False)
    fig.suptitle(f'Coverage of {Path(first["map"]).stem} — coverable area {coverable.sum() * cell:.2f} m²', fontsize=11)
    fig.tight_layout(rect=[0, .07, 1, .93])
    fig.savefig(out, dpi=120)
    print(out)


if __name__ == '__main__':
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    main(sys.argv[1], [tuple(a.split(':', 1)) if ':' in a else (a, a) for a in sys.argv[2:]])
