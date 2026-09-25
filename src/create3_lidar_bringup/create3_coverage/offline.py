"""Registered offline starts, immutable previews and explicit stale-result checks."""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import runpy
import sys
import tempfile
import numpy as np
import yaml
from .geometry import Grid
from .settings import canonical_hash, load_settings, offline_case
from .planning import optimize, boundary_routes, trim_stripes, connect

ROOT = Path(__file__).resolve().parents[3]
REGISTRY = ROOT / 'src/create3_lidar_bringup/config/offline_map_cases.yaml'
INDEX = ROOT / 'reports/preview_index.json'
AUDIT = ROOT / 'maps/registration_audit.json'


def now():
    return datetime.now(timezone.utc).isoformat()


def atomic(path, value, yaml_format=False):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = yaml.safe_dump(value, sort_keys=True) if yaml_format else json.dumps(value, indent=2, allow_nan=False) + '\n'
    fd, tmp = tempfile.mkstemp(prefix='.' + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def read_json(path, default):
    return json.loads(path.read_text()) if path.exists() else default


@contextmanager
def locked():
    INDEX.parent.mkdir(parents=True, exist_ok=True)
    with (INDEX.parent / '.registry.lock').open('a') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        yield


def register(grid, start, settings, update=False, confirm=None, registry=REGISTRY):
    grid.reachable(start, settings.collision)
    cases = yaml.safe_load(Path(registry).read_text()) if Path(registry).exists() else {}
    cases = cases or {}
    old = cases.get(grid.identity)
    if old and old['start_xy_m'] == list(start):
        return {'status': 'unchanged', 'registration_hash': canonical_hash(old)}
    new = dict(old) if old else {'frame': 'map', 'region': 'whole_map',
                                'description': 'Recorded offline example; not a localized robot pose'}
    new['start_xy_m'] = list(start)
    old_hash = canonical_hash(old) if old else None
    new_hash = canonical_hash(new)
    index = read_json(INDEX, [])
    affected = [r['output'] for r in index if old_hash and r.get('registration_hash') == old_hash and r.get('map_id') == grid.identity]
    event = {'time': now(), 'map_id': grid.identity, 'previous_start': old['start_xy_m'] if old else None,
             'new_start': list(start), 'previous_hash': old_hash, 'new_hash': new_hash, 'affected_outputs': affected}
    if old:
        if not update:
            raise ValueError('Start already registered; use --update for an explicitly confirmed change')
        print(json.dumps(event, indent=2))
        if confirm is None:
            if not sys.stdin.isatty():
                raise ValueError('Updating requires an interactive terminal; no changes made')
            confirm = lambda: input('Type UPDATE to replace this registration: ')
        if confirm() != 'UPDATE':
            raise ValueError('Update cancelled; no changes made')
    cases[grid.identity] = new
    atomic(registry, cases, True)
    # Hash validation remains authoritative even if a later audit write is interrupted.
    for item in index:
        if item.get('map_id') == grid.identity and item.get('registration_hash') == old_hash:
            item.update(status='superseded', superseded_by=new_hash)
    atomic(INDEX, index)
    audit = read_json(AUDIT, [])
    audit.append(event)
    atomic(AUDIT, audit)
    return {'status': 'updated' if old else 'registered', 'registration_hash': new_hash}


def verify_baseline(path, grid, fixture_hash, settings):
    data = json.loads(Path(path).read_text())
    summary = data.get('summary', {})
    recorded = data.get('registration_hash', summary.get('registration_hash'))
    if not recorded:
        raise ValueError(f'{path}: historical_unverifiable: missing_registration_hash. Use single-map preview or a new registered baseline; historical data is not re-registered.')
    if recorded != fixture_hash:
        raise ValueError(f'{path}: stale registration_hash; regenerate using the current registered start')
    if summary.get('map_id') != grid.identity:
        raise ValueError(f'{path}: incompatible map_id')
    if data.get('config_hash', summary.get('config_hash')) != settings.hash:
        raise ValueError(f'{path}: stale config_hash')
    return data


def draw_single(grid, start, denominator, panel, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    bg = np.where(grid.cells < 0, 0, np.where(grid.cells == 100, 1, 2))
    h, w = bg.shape
    extent = [0, w*grid.resolution, 0, h*grid.resolution]
    fig, ax = plt.subplots(figsize=(8, 10), layout='constrained')
    ax.imshow(bg, origin='lower', extent=extent, cmap=ListedColormap(['#bcc2ca', '#26313f', '#ffffff']), vmin=0, vmax=2, interpolation='nearest')
    mask = np.zeros(bg.shape)
    mask[panel['covered']] = 1
    mask[panel['remaining']] = 2
    ax.imshow(np.ma.masked_where(mask == 0, mask), origin='lower', extent=extent, cmap=ListedColormap(['#c3e5d3', '#ffd273']), vmin=1, vmax=2, alpha=.8, interpolation='nearest')
    colors = {'boundary': '#764ab5', 'sweep': '#086994', 'connector': '#d69739', 'resweep': '#d54879'}
    handles = []
    for kind, color in colors.items():
        found = False
        for segment in panel['segments']:
            if segment['kind'] == kind:
                points = grid.local(segment['points'])
                ax.plot(points[:, 0], points[:, 1], color=color, linewidth=.9)
                found = True
        if found:
            handles.append(Line2D([], [], color=color, label=kind.capitalize()))
    handles.append(Patch(color='#c3e5d3', label='Ideal covered'))
    handles.append(Patch(facecolor='white', edgecolor='#26313f', label='Free outside denominator'))
    handles.append(Line2D([], [], color='red', marker='*', linestyle='None', label='Example start'))
    if panel['remaining'].any():
        handles.append(Patch(color='#ffd273', label='Remaining in denominator'))
    assert int(panel['remaining'].sum()) == panel['stats']['remaining_cells']
    ax.scatter(*grid.local(start), marker='*', color='red', s=75, zorder=10)
    stats = panel['stats']
    ax.set(aspect='equal', xlabel='Map-local x (m)', ylabel='Map-local y (m)',
           title=f"Boundary + interior | Ideal {stats['coverage_label']}\nPath {stats['path_length_m']:.1f} m | Remaining {stats['remaining_m2']:.3f} m²")
    ax.legend(handles=handles, fontsize=8)
    fig.supxlabel('OFFLINE IDEAL GEOMETRY ONLY. No robot motion or measured coverage.', fontsize=9)
    fig.savefig(output / 'preview.png', dpi=180)
    fig.savefig(output / 'preview.svg')
    plt.close(fig)


def preview(map_path, output, config=None, baseline=None, registry=REGISTRY):
    settings = load_settings(config)
    grid = Grid.load(map_path)
    case, fixture_hash = offline_case(grid, registry)
    start = case['start_xy_m']
    reachable = grid.reachable(start, settings.collision)
    h, w = grid.cells.shape
    region = grid.world([[-.5, -.5], [-.5, w-.5], [h-.5, w-.5], [h-.5, -.5]]).tolist()
    if case['region'] != 'whole_map':
        region = case['region']
    denominator = grid.coverable(reachable, region, settings['geometry']['coverage_disk_radius_m'])
    if not denominator.any():
        raise ValueError('Empty coverable region')
    if baseline:
        verify_baseline(baseline, grid, fixture_hash, settings)
    out = Path(output).resolve()
    if out.exists():
        raise ValueError('Output already exists; use a new directory to preserve reviewed results')
    api = runpy.run_path(str(ROOT / 'src/create3_lidar_bringup/tools/preview_coverage.py'))
    if baseline:
        report = api['run_preview'](map_path, out, config, registry, baseline)
        report.update(registration_hash=fixture_hash, status='current', baseline_status='verified')
        atomic(out / 'summary.json', report)
    else:
        plan = optimize(grid, reachable, denominator, start, settings, aligned=True)
        boundary, audit = boundary_routes(grid, reachable, region, start, settings) if settings['boundary']['enabled'] else ([], [])
        segments = api['route_segments'](boundary, connect(grid, boundary, start, reachable, settings.collision))
        painted = api['union_segments'](grid, denominator, segments, settings['geometry']['coverage_disk_radius_m'])
        interior, dropped = trim_stripes(grid, plan['targets'], denominator & ~painted, settings)
        xy = boundary[-1].end if boundary else start
        segments += api['route_segments'](interior, connect(grid, interior, xy, reachable, settings.collision))
        panel = api['evaluate_panel']('Boundary + interior', grid, reachable, denominator, start, settings, segments)
        report = dict(panel['stats'], map_id=grid.identity, registration_hash=fixture_hash,
                      config_hash=settings.hash, start_map_xy=start, region=region,
                      collision_radius_m=settings.collision, coverage_disk_radius_m=settings['geometry']['coverage_disk_radius_m'],
                      selected=plan['selected'], walls=plan['walls'], boundary_contours=audit,
                      trim_residual=dropped, status='current', actual_coverage_ratio=None)
        out.mkdir(parents=True, exist_ok=False)
        atomic(out / 'summary.json', report)
        atomic(out / 'route.json', {'registration_hash': fixture_hash, 'config_hash': settings.hash, 'summary': report, 'segments': panel['segments']})
        atomic(out / 'candidates.json', plan['candidates'])
        np.savez_compressed(out / 'masks.npz', reachable=reachable, denominator=denominator, covered=panel['covered'], remaining=panel['remaining'])
        draw_single(grid, start, denominator, panel, out)
    index = read_json(INDEX, [])
    index.append({'output': str(out), 'map_id': grid.identity, 'registration_hash': fixture_hash,
                  'config_hash': settings.hash, 'status': 'current', 'created_at': now()})
    atomic(INDEX, index)
    return report


def check_result(path, grid, settings, registry=REGISTRY):
    _, expected = offline_case(grid, registry)
    data = json.loads(Path(path).read_text())
    if data.get('map_id') != grid.identity or data.get('registration_hash') != expected or data.get('config_hash') != settings.hash:
        raise ValueError(f'{path}: stale or historical_unverifiable; regenerate preview')
    return {'status': 'current', 'registration_hash': expected}


def register_main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True)
    parser.add_argument('--start', required=True, type=float, nargs=2, metavar=('X', 'Y'))
    parser.add_argument('--update', action='store_true')
    parser.add_argument('--config')
    args = parser.parse_args()
    try:
        with locked():
            result = register(Grid.load(args.map), args.start, load_settings(args.config), args.update)
        print(json.dumps(result, indent=2))
    except (ValueError, OSError) as error:
        parser.exit(2, str(error) + '\n')


def preview_main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True)
    parser.add_argument('--output')
    parser.add_argument('--config')
    parser.add_argument('--baseline', help='Explicit four-panel mode; requires a current registration hash')
    parser.add_argument('--check-result', help='Read-only staleness check of summary.json')
    args = parser.parse_args()
    if not args.check_result and not args.output:
        parser.error('--output is required unless --check-result is used')
    try:
        if args.check_result:
            result = check_result(args.check_result, Grid.load(args.map), load_settings(args.config))
        else:
            with locked():
                result = preview(args.map, args.output, args.config, args.baseline)
        print(json.dumps(result, indent=2))
    except (ValueError, OSError) as error:
        parser.exit(2, str(error) + '\n')
