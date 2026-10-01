"""Headless coverage planner (cost-optimised): python plan_cli.py map.yaml --start 2 2 [--area x1 y1 x2 y2 ...] -o path.yaml"""
import argparse

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from coverage import export, pipeline, render
from coverage.grid import load_ros_map
from coverage.settings import default_settings, load_settings


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('map', help='ROS map .yaml')
    ap.add_argument('--start', nargs=2, type=float, required=True, metavar=('X', 'Y'))
    ap.add_argument('--area', nargs='+', type=float, help='polygon x1 y1 x2 y2 ... (default: whole map)')
    ap.add_argument('--settings', help='settings .json (saved from the app)')
    ap.add_argument('--mode', choices=['auto', 'regions', 'spiral'], help='auto = cheapest of all patterns')
    ap.add_argument('--overlap', type=float, help='loop overlap in %% (0-50)')
    ap.add_argument('--obstacles', choices=['around', 'outer'], help='loop around obstacles or follow outer walls only')
    ap.add_argument('--robot-radius', type=float)
    ap.add_argument('--coverage-radius', type=float)
    ap.add_argument('-o', '--output', default='coverage_path.yaml', help='.yaml or .json')
    ap.add_argument('--png', help='also save a preview image')
    ap.add_argument('--cells-png', help='also save the boustrophedon cells to this PNG')
    ap.add_argument('--path-png', help='also save the robot path and coverage to this PNG')
    a = ap.parse_args()

    grid = load_ros_map(a.map)
    settings = load_settings(a.settings) if a.settings else default_settings()
    if a.overlap is not None:
        settings['spiral']['overlap_pct'] = max(0.0, min(50.0, a.overlap))
    if a.mode:
        settings['strategy']['mode'] = a.mode
    if a.obstacles:
        settings['spiral']['obstacle_mode'] = a.obstacles
    if a.robot_radius:
        settings['geometry']['robot_radius_m'] = a.robot_radius
    if a.coverage_radius:
        settings['geometry']['coverage_disk_radius_m'] = a.coverage_radius
    polygon = None
    if a.area:
        if len(a.area) % 2 or len(a.area) < 6:
            ap.error('--area needs at least 3 x/y pairs')
        polygon = [list(a.area[i:i + 2]) for i in range(0, len(a.area), 2)]
    result = pipeline.plan(grid, a.start, polygon, settings, progress=print)
    n = export.save(result, a.output, a.map, settings)
    for k, v in result['stats'].items():
        print(f'  {k}: {v:.2f}' if isinstance(v, float) else f'  {k}: {v}')
    print('Candidates (cost in s):')
    for c in result['candidates']:
        print(f"  {c['total_s']:8.0f}  {c['name']}")
    print(f'Wrote {n} poses to {a.output}')
    for path, draw in ((a.cells_png, lambda ax: render.draw_cells(ax, grid, result['cells'])),
                       (a.path_png, lambda ax: render.draw_robot_path(ax, grid, result))):
        if path:
            fig, ax = plt.subplots(figsize=(10, 8))
            render.draw_map(ax, grid)
            draw(ax)
            fig.savefig(path, dpi=130, bbox_inches='tight')
            print(f'Wrote {path}')
    if a.png:
        fig, ax = plt.subplots(figsize=(10, 8))
        render.draw_map(ax, grid)
        render.draw_polygon(ax, polygon)
        render.draw_result(ax, grid, result)
        fig.savefig(a.png, dpi=130, bbox_inches='tight')
        print(f'Wrote {a.png}')


if __name__ == '__main__':
    main()
