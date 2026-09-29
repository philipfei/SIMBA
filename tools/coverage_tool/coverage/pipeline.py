"""End-to-end coverage plan: generate candidate paths, score them with the
cost function (coverage/cost.py) and return the cheapest one."""
import math
import time

import numpy as np
from scipy.ndimage import binary_dilation, label

from . import cost as costmod
from . import localsearch, optimizer, planning, spiral
from .settings import set_settings


def _log(progress, msg):
    if progress:
        progress(msg)


def _disk_struct(radius, res):
    n = int(math.ceil(radius / res))
    yy, xx = np.mgrid[-n:n + 1, -n:n + 1]
    return np.hypot(xx, yy) * res <= radius + 1e-9


def coverage_region(grid, reachable, polygon):
    """Free cells inside the polygon that belong to the same free space as the robot."""
    region = grid.polygon_mask(polygon)
    labels, _ = label(grid.free, structure=np.ones((3, 3)))
    ids = np.unique(labels[reachable & grid.free])
    ids = ids[ids > 0]
    return grid.free & region & np.isin(labels, ids)


def brush_reachable(grid, den, reach, radius, chunk=4000):
    """Cells the brush can cover: within `radius` of some position the robot can
    reach (robot-radius clearance, connected to the start) AND in clear line of
    sight from that position. Uses exactly the same test as the coverage check."""
    from scipy.spatial import cKDTree
    cand = den & binary_dilation(reach, _disk_struct(radius, grid.resolution))
    ok = cand & reach                      # the robot can stand on these cells
    rest = np.argwhere(cand & ~reach)
    if not len(rest):
        return ok
    poses = grid.world(np.argwhere(reach))
    tree = cKDTree(poses)
    xy = grid.world(rest)
    todo = np.arange(len(rest))
    for k in (1, 8, None):                 # nearest pose first, then more, then all in range
        if not len(todo):
            break
        if k is None:
            lists = tree.query_ball_point(xy[todo], radius + 1e-9)
        else:
            d, idx = tree.query(xy[todo], k=k, distance_upper_bound=radius + 1e-9)
            idx = np.atleast_2d(idx.T).T if k == 1 else idx
            lists = [[j for j in row if j < len(poses)] for row in np.atleast_2d(idx.reshape(len(todo), -1))]
        cells = np.repeat(todo, [len(l) for l in lists])
        pose_ids = np.fromiter((j for l in lists for j in l), int, count=len(cells))
        seen = np.zeros(len(rest), bool)
        for a in range(0, len(cells), chunk):
            c, p = cells[a:a + chunk], pose_ids[a:a + chunk]
            vis = planning.visible_many(grid, poses[p], xy[c])
            seen[c[vis]] = True
        hit = rest[seen]
        ok[hit[:, 0], hit[:, 1]] = True
        todo = todo[~seen[todo]]
    return ok


# ------------------------------------------------------------- candidates
def _spiral_candidate(grid, reach, polygon, start, settings, mode):
    s = type(settings)({k: dict(v) if isinstance(v, dict) else v for k, v in settings.items()})
    s['spiral']['obstacle_mode'] = mode
    targets, info = spiral.spiral_targets(grid, reach, polygon, start, s)
    if mode == 'outer' and targets:
        rims = spiral.obstacle_rims(grid, reach, polygon, s)
        targets = spiral.insert_loops(grid, targets, rims, start, s)
    return targets, info


def _sanitize(grid, targets, collision):
    """Final safety pass: split any target at a segment that fails the collision
    check (connectors then route around that spot with A*)."""
    out = []
    for t in targets:
        pts = planning.points_of(t)
        kind = getattr(t, 'kind', 'spiral')
        if len(pts) < 2 or all(grid.segment_safe(a, b, collision) for a, b in zip(pts, pts[1:])):
            out.append(t)
            continue
        part = [pts[0]]
        for q in pts[1:]:
            if grid.segment_safe(part[-1], q, collision):
                part.append(q)
            else:
                if len(part) > 1:
                    out.append(planning.poly_target(part, kind))
                part = [q]
        if len(part) > 1:
            out.append(planning.poly_target(part, kind))
    return out


def _finish(grid, targets, start, den, reach, coverable, settings, progress, budget_s, fill=True):
    """Centre-line fill, cost-driven route optimisation, connectors, coverage and exact cost."""
    radius = float(settings['geometry']['coverage_disk_radius_m'])
    collision = settings.collision
    if fill:
        for _ in range(2):  # second pass picks up strips the first centre lines just missed
            f = spiral.centre_line_fill(grid, targets, den, reach, settings)
            if not f:
                break
            targets = spiral.insert_cheapest(targets, f, start)

    # optimise the whole route on the cost function (drop / flip / move / 2-opt / detours)
    opt = localsearch.RouteOptimizer(grid, localsearch.pieces_from_targets(targets), start, reach, den,
                                     coverable, settings, int(settings['optimizer']['neighbours']))
    opt.run(budget_s)
    targets = _sanitize(grid, opt.targets(settings['planning']['max_segment_length_m']), collision)
    coverable = coverable & opt.cov_ok.reshape(coverable.shape)   # minus cells no safe pose can see

    final, links, skipped = [], [], 0
    xy = list(start)
    for t in targets:
        try:
            link = planning.connect(grid, [t], xy, reach, collision)[0]
        except ValueError:
            skipped += 1
            continue
        final.append(t)
        links.append(link)
        xy = t.end
    covered = planning.stroke_union(grid, den, final, radius)
    route = []
    for t, c in zip(final, links):
        route.extend(c + planning.points_of(t))
    cell = grid.resolution ** 2
    open_ = coverable & ~covered
    ev = costmod.evaluate(route, final, covered.sum() * cell, open_.sum() * cell, settings,
                          costmod.swept_floor(grid, den, final, radius),
                          (open_ & costmod.edge_mask(grid, settings)).sum() * cell)
    return {'targets': final, 'connections': links, 'skipped': skipped, 'covered': covered, 'route': route,
            'cost': ev, 'coverable': coverable, 'uncovered_reachable': int(open_.sum()),
            'uncovered_interior': int((open_ & ~costmod.edge_mask(grid, settings)).sum())}


def plan(grid, start_xy, polygon, settings, progress=None):
    t0 = time.monotonic()
    set_settings(settings)
    radius = float(settings['geometry']['coverage_disk_radius_m'])
    collision = settings.collision
    spacing = spiral.path_spacing(settings)

    _log(progress, 'Finding reachable space...')
    reach, start = grid.reachable_from(start_xy, collision)
    if start is None:
        raise ValueError('No collision-free space for this robot radius.')
    den = coverage_region(grid, reach, polygon)
    if not den.any():
        raise ValueError('Selected area contains no free space reachable from the start.')
    # cells the brush can physically reach from some safe robot position
    _log(progress, 'Finding every cell the brush can reach...')
    coverable = brush_reachable(grid, den, reach, radius)

    mode = settings['strategy']['mode']
    builders = []
    if mode in ('auto', 'spiral'):
        chosen = settings['spiral']['obstacle_mode']
        for om, label_ in (('around', 'Spiral, around obstacles'), ('outer', 'Spiral, outer walls only')):
            if mode == 'auto' or om == chosen:
                builders.append((label_, lambda om=om: _spiral_candidate(grid, reach, polygon, start, settings, om)))
    if mode in ('auto', 'regions'):
        F = spiral.distance_field(grid, reach, polygon, radius, 'around')
        wall = optimizer.dominant_angle(grid, F, (F > 0) & (F < 2 * grid.resolution))
        walls = (wall, (wall + 90) % 180)
        for a in walls:
            builders.append((f'Mixed regions (split at {a:.0f} deg)',
                             lambda a=a: optimizer.plan_regions(grid, reach, den, coverable, polygon, start,
                                                                settings, a, progress, walls)))
        builders.append(('Straight lanes, one direction',
                         lambda: optimizer.plan_regions(grid, reach, den, coverable, polygon, start,
                                                        settings, None, progress, walls)))
        # wall loop first, then boustrophedon cells inside it, one zigzag of lanes per cell
        for a in walls:
            builders.append((f'Wall loop + boustrophedon cells ({a:.0f} deg)',
                             lambda a=a: optimizer.plan_boustrophedon(grid, reach, den, coverable, polygon, start,
                                                                      settings, a, progress, wall_loop=True)))

        # regional choice: per region the spiral's rings or straight lanes, whichever is cheaper
        rings = {}

        def hybrid(a):
            if 't' not in rings:
                rings['t'] = _spiral_candidate(grid, reach, polygon, start, settings, 'around')[0]
            return optimizer.plan_hybrid(grid, reach, den, coverable, polygon, start, settings, a,
                                         rings['t'], progress, walls)
        builders.append((f'Rings or lanes per region (split at {wall:.0f} deg)', lambda: hybrid(wall)))

    candidates, best = [], None
    for name, build in builders:
        _log(progress, f'Candidate: {name}...')
        targets, info = build()
        if not targets:
            candidates.append({'name': name, 'total_s': math.inf, 'note': 'no path'})
            continue
        _log(progress, f'Candidate: {name}: centre lines, connections and cost...')
        res = _finish(grid, targets, start, den, reach, coverable, settings, progress,
                      float(settings['optimizer']['search_time_s']))
        res['info'], res['name'] = info, name
        c = res['cost']
        cov_c = res['coverable']
        candidates.append({'name': name, 'total_s': c['total_s'], 'drive_turn_s': c['drive_turn_s'],
                           'coverage_pct': 100.0 * (res['covered'] & cov_c).sum() / max(1, cov_c.sum())})
        if best is None or c['total_s'] < best['cost']['total_s']:
            best = res
    if best is None:
        raise ValueError('The area is too small for this robot: no safe path fits inside it.')
    polish = float(settings['optimizer']['polish_time_s'])
    if polish > 0:
        _log(progress, f'Optimising {best["name"]} further on the cost function...')
        before = best['cost']['total_s']
        res = _finish(grid, best['targets'], start, den, reach, coverable, settings, progress, polish, fill=False)
        if res['cost']['total_s'] <= before:
            res['info'], res['name'] = best['info'], best['name']
            best = res
            for c in candidates:
                if c['name'] == best['name']:
                    c['total_s'] = res['cost']['total_s']

    final, links, covered, c = best['targets'], best['connections'], best['covered'], best['cost']
    coverable = best['coverable']
    kinds = [getattr(t, 'kind', 'spiral') for t in final]
    work_len = sum(planning.length(planning.points_of(t)) for t in final)
    link_len = sum(planning.length(x) for x in links)
    info = best['info']
    stats = {
        'pattern': best['name'],
        'cost_total_s': c['total_s'],
        'cost_drive_s': c['drive_s'],
        'cost_turn_s': c['turn_s'],
        'cost_bend_s': c['bend_s'],
        'edge_missed_m2': float(c['edge_missed_m2']),
        'cost_reverse_s': c['reverse_s'],
        'cost_overlap_s': c['overlap_s'],
        'cost_missed_s': c['missed_s'],
        'sharp_turns': c['stops'],
        'hairpins': c['hairpins'],
        'overlap_m2': c['overlap_m2'],
        'missed_m2': c['missed_m2'],
        'coverage_pct': 100.0 * covered.sum() / den.sum(),
        'coverage_of_reachable_pct': 100.0 * (covered & coverable).sum() / max(1, coverable.sum()),
        'area_m2': float(den.sum() * grid.resolution ** 2),
        'covered_m2': float(covered.sum() * grid.resolution ** 2),
        'path_spacing_m': spacing,
        'overlap_pct': float(settings['spiral']['overlap_pct']),
        'regions': info.get('regions', 0),
        'rings': info.get('rings', 0),
        'loops': info.get('loops', 0),
        'working_length_m': work_len,
        'connector_length_m': link_len,
        'total_length_m': work_len + link_len,
        'estimated_time_s': c['drive_turn_s'],
        'goals': len(final),
        'wall_segments': kinds.count('wall'),
        'spiral_segments': kinds.count('spiral'),
        'lane_segments': kinds.count('lane'),
        'centre_line_segments': kinds.count('fill'),
        'gap_goals': kinds.count('gap'),
        'skipped_unreachable': best['skipped'],
        'uncovered_reachable_cells': best['uncovered_reachable'],
        'uncovered_interior_cells': best['uncovered_interior'],   # not counting the edge band
        'planning_time_s': time.monotonic() - t0,
    }
    _log(progress, 'Done.')
    return {
        'start': start, 'polygon': polygon, 'targets': final, 'connections': links,
        'covered': covered, 'region': den, 'coverable': coverable, 'reachable': reach, 'stats': stats,
        'candidates': sorted(candidates, key=lambda x: x['total_s']),
        'region_patterns': info.get('region_patterns', []),
        'cells': _display_cells(grid, reach, polygon, settings, info),
        'plan_cells': _plan_cells(info),
        'coverage_radius': radius, 'robot_radius': collision,
    }


def _display_cells(grid, reach, polygon, settings, info):
    """Boustrophedon cells of the whole safe area (where the robot centre may go),
    swept along the plan's split angle, or the main wall direction."""
    angle = info.get('decomposition_angle')
    if angle is None:
        radius = float(settings['geometry']['coverage_disk_radius_m'])
        F = spiral.distance_field(grid, reach, polygon, radius, 'around')
        angle = optimizer.dominant_angle(grid, F, (F > 0) & (F < 2 * grid.resolution))
    L, _ = optimizer.boustrophedon_cells(grid, reach, polygon, settings, angle, merge_spacings=1.0)
    return {'labels': L, 'angle': angle, 'patterns': []}


def _plan_cells(info):
    """The regions the chosen plan actually used, with the pattern of each (or None)."""
    L = info.get('cells')
    if L is None or not L.any():
        return None
    return {'labels': L, 'angle': info.get('decomposition_angle'), 'patterns': info.get('region_patterns', []),
            'title': 'Regions used by the plan'}
