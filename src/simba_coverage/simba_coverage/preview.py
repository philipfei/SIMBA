"""Generate an immutable English-labelled static coverage preview."""
import argparse
import json
from pathlib import Path
import numpy as np
from .geometry import Grid
from .params import load_settings, offline_case, approval_manifest
from .planning import optimize, boundary_routes, trim_stripes, connect
from .comparison import route_segments, union_segments, evaluate_panel


def _whole_map_polygon(grid):
    height, width = grid.cells.shape
    return grid.world([[-0.5, -0.5], [-0.5, width - 0.5],
                       [height - 0.5, width - 0.5], [height - 0.5, -0.5]]).tolist()


def _write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def _draw(grid, start, denominator, panel, output, selected, manifest):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    background = np.where(grid.cells < 0, 0, np.where(grid.cells == 100, 1, 2))
    height, width = background.shape
    extent = [0, width * grid.resolution, 0, height * grid.resolution]
    fig, axis = plt.subplots(figsize=(9, 10), layout='constrained')
    axis.imshow(background, origin='lower', extent=extent,
                cmap=ListedColormap(['#bcc2ca', '#26313f', '#ffffff']),
                vmin=0, vmax=2, interpolation='nearest')
    overlay = np.zeros(background.shape)
    overlay[panel['covered']] = 1
    overlay[panel['remaining']] = 2
    axis.imshow(np.ma.masked_where(overlay == 0, overlay), origin='lower', extent=extent,
                cmap=ListedColormap(['#c3e5d3', '#ffd273']), vmin=1, vmax=2,
                alpha=0.8, interpolation='nearest')
    colors = {'boundary': '#764ab5', 'sweep': '#086994',
              'connector': '#d69739', 'resweep': '#d54879'}
    handles = []
    for kind, color in colors.items():
        present = False
        for segment in panel['segments']:
            if segment['kind'] == kind:
                points = grid.local(segment['points'])
                axis.plot(points[:, 0], points[:, 1], color=color, linewidth=0.9)
                present = True
        if present:
            handles.append(Line2D([], [], color=color, label=kind.capitalize()))
    handles.extend([Patch(color='#c3e5d3', label='Ideal covered'),
                    Patch(color='#ffd273', label='Remaining in denominator'),
                    Line2D([], [], color='red', marker='*', linestyle='None', label='Example start')])
    axis.scatter(*grid.local(start), marker='*', color='red', s=75, zorder=10)
    stats = panel['stats']
    axis.set(aspect='equal', xlabel='Map-local x (m)', ylabel='Map-local y (m)',
             title=(f"Boundary + interior | Ideal {stats['coverage_label']}\n"
                    f"Direction {selected['angle_deg']:.2f} deg | Offset {selected['offset_m']:.3f} m\n"
                    f"Path {stats['path_length_m']:.1f} m | Remaining {stats['remaining_m2']:.3f} m^2"))
    axis.legend(handles=handles, fontsize=8)
    fig.supxlabel('STATIC IDEAL ESTIMATE ONLY. This is not a closed-loop simulation.', fontsize=9)
    fig.savefig(output / 'preview.png', dpi=180)
    fig.savefig(output / 'preview.svg')
    plt.close(fig)
    assert manifest['components']['coverable_mask']


def preview(map_path, output, config_dir=None, start=None):
    settings = load_settings(config_dir)
    grid = Grid.load(map_path)
    case, fixture_hash = offline_case(grid, settings.config_dir, start)
    example_start = [float(value) for value in case['start_xy_m']]
    reachable = grid.reachable(example_start, settings.collision)
    region = _whole_map_polygon(grid)
    coverable = grid.coverable(reachable, region, settings['geometry']['coverage_disk_radius_m'])
    if not coverable.any():
        raise ValueError('The whole-map coverable denominator is empty')
    plan = optimize(grid, reachable, coverable, example_start, settings, aligned=True)
    boundary, boundary_audit = boundary_routes(grid, reachable, region, example_start, settings)
    segments = route_segments(boundary, connect(grid, boundary, example_start, reachable, settings.collision))
    boundary_covered = union_segments(grid, coverable, segments,
                                      settings['geometry']['coverage_disk_radius_m'])
    interior, trim_residual = trim_stripes(grid, plan['targets'], coverable & ~boundary_covered, settings)
    current = boundary[-1].end if boundary else example_start
    segments += route_segments(interior, connect(grid, interior, current, reachable, settings.collision))
    panel = evaluate_panel('Boundary + interior', grid, reachable, coverable,
                           example_start, settings, segments)
    manifest = approval_manifest(settings, map_path, reachable, coverable, plan['selected'])
    destination = Path(output).resolve()
    if destination.exists():
        raise ValueError('Output already exists; choose a new directory')
    destination.mkdir(parents=True)
    summary = {
        'map_id': grid.identity,
        'fixture_hash': fixture_hash,
        'example_start_map_xy': example_start,
        'example_start_not_localized': True,
        'region': 'whole_map',
        'collision_radius_m': settings.collision,
        'coverage_disk_radius_m': settings['geometry']['coverage_disk_radius_m'],
        'selected': plan['selected'],
        'walls': plan['walls'],
        'boundary_audit': boundary_audit,
        'trim_residual': trim_residual,
        'statistics': panel['stats'],
        'approval_manifest': manifest,
        'approval_hash': manifest['approval_hash'],
        'git_commit': _git_value(settings.config_dir, 'rev-parse', 'HEAD'),
        'git_dirty': bool(_git_value(settings.config_dir, 'status', '--porcelain')),
    }
    _write_json(destination / 'summary.json', summary)
    _write_json(destination / 'approval_manifest.json', manifest)
    _write_json(destination / 'candidates.json', plan['candidates'])
    _write_json(destination / 'route.json', {'segments': panel['segments']})
    np.savez_compressed(destination / 'masks.npz', reachable=reachable, coverable=coverable,
                        covered=panel['covered'], remaining=panel['remaining'])
    _draw(grid, example_start, coverable, panel, destination, plan['selected'], manifest)
    return summary


def _git_value(config_dir, *arguments):
    import subprocess
    root = Path(config_dir).resolve().parents[2]
    try:
        result = subprocess.run(['git', '-C', str(root), *arguments], check=True,
                                capture_output=True, text=True, timeout=2)
        return result.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return 'unknown'


def preview_main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True)
    parser.add_argument('--config-dir')
    parser.add_argument('--start', type=float, nargs=2, metavar=('X', 'Y'))
    parser.add_argument('--output', required=True)
    arguments = parser.parse_args()
    try:
        result = preview(arguments.map, arguments.output, arguments.config_dir, arguments.start)
        print(json.dumps({'approval_hash': result['approval_hash'],
                          'components': result['approval_manifest']['components'],
                          'selected': result['selected'],
                          'statistics': result['statistics']}, indent=2))
    except (ValueError, OSError, KeyError) as error:
        parser.exit(2, str(error) + '\n')


if __name__ == '__main__':
    preview_main()
