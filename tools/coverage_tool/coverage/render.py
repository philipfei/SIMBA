"""Matplotlib drawing shared by the GUI and the command-line tool."""
import numpy as np
from matplotlib.colors import ListedColormap
from matplotlib.transforms import Affine2D

from .planning import points_of

COLORS = {'spiral': '#2a6fdb', 'sweep': '#2a6fdb', 'wall': '#e8870e', 'boundary': '#e8870e', 'gap': '#c2185b', 'fill': '#00897b', 'lane': '#6a3fb5', 'connector': '#7a7a7a',
          'region': '#1b9e4b', 'start': '#d62728'}


def _extent(grid):
    h, w = grid.shape
    ox, oy, _ = grid.origin
    return [ox, ox + w * grid.resolution, oy, oy + h * grid.resolution]


def _transform(ax, grid):
    ox, oy, yaw = grid.origin
    return Affine2D().rotate_around(ox, oy, yaw) + ax.transData


def draw_map(ax, grid):
    img = np.full(grid.shape, 1, np.uint8)       # unknown
    img[grid.cells == 0] = 2                     # free
    img[(grid.cells != 0) & (grid.cells != -1)] = 0  # occupied
    ax.imshow(img, cmap=ListedColormap(['#2b2b2b', '#c8c8c8', '#ffffff']), vmin=0, vmax=2,
              origin='lower', extent=_extent(grid), transform=_transform(ax, grid),
              interpolation='nearest', zorder=0)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')


def draw_mask(ax, grid, mask, color, alpha):
    rgba = np.zeros(grid.shape + (4,))
    rgba[mask] = list(_hex(color)) + [alpha]
    ax.imshow(rgba, origin='lower', extent=_extent(grid), transform=_transform(ax, grid),
              interpolation='nearest', zorder=1)


def _hex(c):
    c = c.lstrip('#')
    return [int(c[i:i + 2], 16) / 255 for i in (0, 2, 4)]


def draw_result(ax, grid, result, show_coverage=True, show_connectors=True):
    if show_coverage:
        coverable = result.get('coverable', result['region'])
        draw_mask(ax, grid, coverable & ~result['covered'], '#e53935', 0.45)            # missed
        draw_mask(ax, grid, result['region'] & ~coverable, '#f4b400', 0.35)               # too close to walls to reach
        draw_mask(ax, grid, result['covered'], COLORS['region'], 0.18)
    if show_connectors:
        for c in result['connections']:
            if len(c) > 1:
                a = np.asarray(c)
                ax.plot(a[:, 0], a[:, 1], '--', color=COLORS['connector'], lw=0.9, zorder=2)
    for t in result['targets']:
        kind = getattr(t, 'kind', 'spiral')
        a = np.asarray(points_of(t))
        if kind == 'gap' or len(a) < 2 or np.allclose(a[0], a[-1]) and len(a) == 2:
            ax.plot(a[0, 0], a[0, 1], 'o', color=COLORS['gap'] if kind == 'gap' else COLORS['spiral'], ms=4, zorder=4)
        else:
            ax.plot(a[:, 0], a[:, 1], '-', color=COLORS.get(kind, COLORS['spiral']), lw=1.6, zorder=3)
    if result['targets']:
        e = result['targets'][-1].end
        ax.plot(e[0], e[1], 's', color='black', ms=6, zorder=5)
    s = result['start']
    ax.plot(s[0], s[1], 'o', color=COLORS['start'], ms=8, mec='white', zorder=6)


def draw_polygon(ax, polygon, closed=True):
    if not polygon:
        return
    a = np.asarray(polygon + ([polygon[0]] if closed and len(polygon) > 2 else []))
    ax.plot(a[:, 0], a[:, 1], '-o', color=COLORS['region'], lw=1.5, ms=3, zorder=7)


# ------------------------------------------------------- boustrophedon cells
_CELL_COLORS = ['#4e79a7', '#f28e2b', '#e15759', '#76b7b2', '#59a14f', '#edc948', '#b07aa1', '#ff9da7',
                '#9c755f', '#bab0ac', '#86bcb6', '#d37295', '#8cd17d', '#b6992d', '#499894', '#a0cbe8']


def draw_cells(ax, grid, cells, alpha=0.55, numbers=True):
    """Show the boustrophedon cell decomposition.

    cells = result['cells'] (dict with 'labels', 'angle', 'patterns') or a
    label array (cell id per grid cell, 0 = no cell). Every cell gets its own
    colour, a thin border and its number; when the plan used the cells, the
    pattern chosen for each cell is written under the number."""
    from scipy.ndimage import center_of_mass
    info = cells if isinstance(cells, dict) else {'labels': cells}
    L = np.asarray(info['labels'])
    ids = [int(i) for i in np.unique(L) if i > 0]
    rgba = np.zeros(grid.shape + (4,))
    for k, i in enumerate(ids):
        rgba[L == i] = _hex(_CELL_COLORS[k % len(_CELL_COLORS)]) + [alpha]
    border = (L > 0) & _edge(L)
    rgba[border] = [0.1, 0.1, 0.1, 0.9]
    ax.imshow(rgba, origin='lower', extent=_extent(grid), transform=_transform(ax, grid),
              interpolation='nearest', zorder=1)
    patterns = info.get('patterns') or []
    if numbers:
        for i in ids:
            rc = np.array(center_of_mass(L == i))
            m = np.argwhere(L == i)                       # centre may fall outside a non-convex cell
            rc = m[np.argmin(np.linalg.norm(m - rc, axis=1))]
            x, y = grid.world(rc)
            txt = str(i)
            if i - 1 < len(patterns):
                kind, ang = patterns[i - 1]
                txt += '\n' + (f'lanes {ang:.0f}°' if kind == 'lanes' and ang is not None else kind)
            ax.text(x, y, txt, ha='center', va='center', fontsize=7, zorder=8,
                    bbox=dict(boxstyle='round,pad=0.15', fc='white', ec='none', alpha=0.8))
    a = info.get('angle')
    title = info.get('title', 'Boustrophedon cells') + f': {len(ids)}' + \
        (f', lanes at {a:.0f}°' if a is not None else '')
    ax.set_title(title, fontsize=9)
    ax.set_aspect('equal')


def _edge(L):
    """Cells with a 4-neighbour that belongs to another cell (or to no cell)."""
    P = np.pad(L, 1, constant_values=-1)
    c = P[1:-1, 1:-1]
    return (P[:-2, 1:-1] != c) | (P[2:, 1:-1] != c) | (P[1:-1, :-2] != c) | (P[1:-1, 2:] != c)


# ------------------------------------------------- robot path and coverage
def coverage_count(grid, result, radius=None):
    """How many times the brush passes over each floor cell (0 = missed).
    Every working piece counts once, like in the cost function."""
    from .cost import piece_cells
    from .planning import points_of
    r = radius or result.get('coverage_radius') or 0.25
    count = np.zeros(grid.shape, np.int32)
    flat = count.ravel()
    chains, prev = [], None                       # rejoin lines split into several Nav2 goals
    for t in result['targets']:
        pts = [list(map(float, p)) for p in points_of(t)]
        if prev is not None and getattr(t, 'kind', None) == getattr(prev, 'kind', None) and len(chains[-1]) > 1 \
                and np.hypot(*np.subtract(chains[-1][-1], pts[0])) < 1e-6 \
                and np.hypot(*np.subtract(pts[0], pts[-1])) > 1e-6:
            chains[-1].extend(pts[1:])
        else:
            chains.append(pts)
        prev = t
    for c in chains:
        flat[piece_cells(grid, result['region'], c, r)] += 1
    return count


def robot_route(result):
    """The complete route in driving order: list of (points, is_transit)."""
    from .planning import points_of
    out = []
    for t, c in zip(result['targets'], result['connections']):
        if len(c) > 1:
            out.append((c, True))
        out.append((points_of(t), False))
    return out


def draw_robot_path(ax, grid, result, radius=None, arrows=12, show_goals=True, colorbar=True):
    """The route the robot actually drives, with what it covers.

    Background: how often the brush passes each floor cell
      red = missed (reachable), light green = once, yellow = twice, orange = 3+,
      grey hatch = not reachable for the brush.
    Line: the full route in driving order, coloured from start (dark) to end
    (yellow); transit between pieces is dashed. Arrows show the driving
    direction, dots are the Nav2 goals the robot receives."""
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from . import export
    cb = None
    count = coverage_count(grid, result, radius)
    region = result['region']
    coverable = result.get('coverable', region)
    layers = [(coverable & (count == 0), '#e53935', 0.55), (region & ~coverable, '#f4b400', 0.30),
              (count == 1, '#8fd18f', 0.45), (count == 2, '#f2d541', 0.55), (count >= 3, '#f08a24', 0.6)]
    for mask, color, alpha in layers:
        draw_mask(ax, grid, mask, color, alpha)

    segs, transit = [], []
    for pts, is_transit in robot_route(result):
        p = np.asarray(pts, float)
        for a, b in zip(p, p[1:]):
            if np.hypot(*(b - a)) > 1e-9:
                segs.append([a, b])
                transit.append(is_transit)
    if segs:
        segs = np.asarray(segs)
        L = np.hypot(*(segs[:, 1] - segs[:, 0]).T)
        s = np.concatenate([[0], np.cumsum(L)[:-1]]) / max(L.sum(), 1e-9)   # progress 0..1 along the route
        transit = np.asarray(transit)
        cmap = plt.get_cmap('viridis')
        for sel, style, lw in ((~transit, 'solid', 2.0), (transit, (0, (3, 2)), 1.1)):
            if sel.any():
                lc = LineCollection(segs[sel], colors=cmap(s[sel]), linewidths=lw, linestyles=style, zorder=3)
                ax.add_collection(lc)
        # direction arrows spread evenly along the working path
        work = np.flatnonzero(~transit)
        if arrows and len(work):
            cum = np.cumsum(L[work])
            for q in np.linspace(0, cum[-1], arrows + 2)[1:-1]:
                k = work[np.searchsorted(cum, q)]
                a, b = segs[k]
                m = (a + b) / 2
                d = (b - a) / max(np.hypot(*(b - a)), 1e-9) * 0.01
                ax.annotate('', xy=m + d, xytext=m - d, zorder=5,
                            arrowprops=dict(arrowstyle='-|>', color=cmap(s[k]), lw=1.2, mutation_scale=14))
        if colorbar:
            sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, 100))
            cb = ax.figure.colorbar(sm, ax=ax, fraction=0.035, pad=0.02)   # returned so a GUI can remove it
            cb.set_label('route progress (%)', fontsize=8)
            cb.ax.tick_params(labelsize=7)
    if show_goals:
        g = export.waypoints(result)
        if g:
            ax.plot([p['x'] for p in g], [p['y'] for p in g], '.', color='black', ms=3, zorder=4)
    s0 = result['start']
    ax.plot(s0[0], s0[1], 'o', color=COLORS['start'], ms=8, mec='white', zorder=6)
    if result['targets']:
        e = result['targets'][-1].end
        ax.plot(e[0], e[1], 's', color='black', ms=6, zorder=6)
    n = int((region & (count > 0)).sum())
    twice = int((count >= 2).sum())
    reach_n = int((coverable & (count > 0)).sum())
    ax.set_title(f'Robot path: covers {100 * reach_n / max(1, coverable.sum()):.1f} % of the reachable floor '
                 f'({100 * n / max(1, region.sum()):.1f} % of the area); '
                 f'{100 * twice / max(1, n):.0f} % of it more than once', fontsize=9)
    ax.set_aspect('equal')
    return cb
