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
    """Render the standard preview with Pillow, without a Matplotlib runtime dependency."""
    import base64
    from io import BytesIO
    from PIL import Image, ImageDraw, ImageFont

    height, width = grid.cells.shape
    scale = 10
    margin, header, footer = 40, 125, 105
    colors = {
        'unknown': np.array([188, 194, 202], dtype=np.uint8),
        'occupied': np.array([38, 49, 63], dtype=np.uint8),
        'free': np.array([255, 255, 255], dtype=np.uint8),
        'covered': np.array([195, 229, 211], dtype=np.uint8),
        'remaining': np.array([255, 210, 115], dtype=np.uint8),
        'boundary': '#764ab5', 'sweep': '#086994',
        'connector': '#d69739', 'resweep': '#d54879',
    }
    pixels = np.empty((height, width, 3), dtype=np.uint8)
    pixels[:] = colors['unknown']
    pixels[grid.cells == 100] = colors['occupied']
    pixels[grid.cells == 0] = colors['free']
    pixels[panel['covered']] = colors['covered']
    pixels[panel['remaining']] = colors['remaining']
    map_image = Image.fromarray(np.flipud(pixels), 'RGB').resize(
        (width * scale, height * scale), Image.Resampling.NEAREST)
    canvas = Image.new('RGB', (width * scale + 2 * margin,
                               height * scale + header + footer), 'white')
    canvas.paste(map_image, (margin, header))
    draw = ImageDraw.Draw(canvas)

    font_path = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    def font(size):
        return ImageFont.truetype(str(font_path), size) if font_path.is_file() else ImageFont.load_default()

    def image_point(world):
        x, y = grid.local(world)
        return margin + x / grid.resolution * scale, header + (height - y / grid.resolution) * scale

    present = []
    for kind in ('boundary', 'sweep', 'connector', 'resweep'):
        segments = [item for item in panel['segments'] if item['kind'] == kind]
        if segments:
            present.append(kind)
        for segment in segments:
            points = [image_point(point) for point in segment['points']]
            if len(points) >= 2:
                draw.line(points, fill=colors[kind], width=3, joint='curve')

    sx, sy = image_point(start)
    star = []
    import math
    for index in range(10):
        radius = 10 if index % 2 == 0 else 4
        angle = -math.pi / 2 + index * math.pi / 5
        star.append((sx + radius * math.cos(angle), sy + radius * math.sin(angle)))
    draw.polygon(star, fill='#d7191c')
    draw.rectangle((margin, header, margin + width * scale, header + height * scale),
                   outline='#202020', width=2)

    stats = panel['stats']
    title = f"Boundary + interior | Ideal {stats['coverage_label']}"
    details = (f"Direction {selected['angle_deg']:.2f} deg | Offset {selected['offset_m']:.3f} m | "
               f"Path {stats['path_length_m']:.1f} m | Remaining {stats['remaining_m2']:.3f} m^2")
    draw.text((margin, 18), title, fill='black', font=font(24))
    draw.text((margin, 55), details, fill='black', font=font(15))
    draw.text((margin, 82),
              f"Map-local extent: {width * grid.resolution:.2f} m x {height * grid.resolution:.2f} m",
              fill='#303030', font=font(14))

    legend = [(kind.capitalize(), colors[kind]) for kind in present]
    legend += [('Ideal covered', tuple(colors['covered'])),
               ('Remaining in denominator', tuple(colors['remaining'])),
               ('Example start', '#d7191c')]
    x, y = margin, header + height * scale + 20
    for label, color in legend:
        draw.rectangle((x, y, x + 18, y + 12), fill=color, outline='#303030')
        draw.text((x + 25, y - 3), label, fill='black', font=font(13))
        x += 25 + int(draw.textlength(label, font=font(13))) + 22
        if x > canvas.width - 180:
            x, y = margin, y + 25
    draw.text((margin, canvas.height - 27),
              'STATIC IDEAL ESTIMATE ONLY. This is not a closed-loop simulation.',
              fill='#333333', font=font(14))

    png_path = output / 'preview.png'
    canvas.save(png_path, format='PNG', optimize=True)
    buffer = BytesIO();canvas.save(buffer, format='PNG', optimize=True)
    encoded = base64.b64encode(buffer.getvalue()).decode('ascii')
    (output / 'preview.svg').write_text(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{canvas.width}" height="{canvas.height}" '
        f'viewBox="0 0 {canvas.width} {canvas.height}"><image width="100%" height="100%" '
        f'href="data:image/png;base64,{encoded}"/></svg>\n', encoding='utf-8')
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
