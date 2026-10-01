"""Mixed-pattern planner driven by the cost function.

1. Wall loop: follow the outer walls at the robot's safety limit (and one loop
   around every free-standing obstacle).
2. The interior (one path spacing inside those loops) is split into regions
   with a boustrophedon cell decomposition; tiny regions are merged.
3. Every region tries straight lanes at many angles and an inward spiral.
   Each option is scored with cost.evaluate (drive time, turn stops, hairpins,
   double coverage, missed area) and the cheapest is kept.
4. Regions and obstacle loops are visited in a greedy order that picks, for
   every region, the entry corner / lane direction closest to the robot.
"""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt, label

from . import cost as costmod
from . import planner, planning, spiral
from .planning import points_of, poly_target, split_path, stroke_union

LANE_ANGLES = np.arange(0.0, 180.0, 15.0)
FULL_EVAL_ANGLES = 3          # lane angles that get a full cost evaluation per region
LANE_INSET = 0.75            # lanes end this many path spacings inside the wall loop
MERGE_SPACINGS = 5.0          # regions smaller than (this x spacing)^2 are merged into a neighbour


# ------------------------------------------------------------------ helpers
def _unit(angle_deg):
    a = math.radians(angle_deg)
    return np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])


def _cells_of(grid, pts):
    """Row/col index of world points (vectorized); -1 when outside the map."""
    loc = grid.local(pts)
    rc = np.floor(loc[:, ::-1] / grid.resolution).astype(int)
    h, w = grid.shape
    bad = (rc[:, 0] < 0) | (rc[:, 1] < 0) | (rc[:, 0] >= h) | (rc[:, 1] >= w)
    rc[bad] = -1
    return rc


def _inside(mask, rc):
    ok = rc[:, 0] >= 0
    out = np.zeros(len(rc), bool)
    out[ok] = mask[rc[ok, 0], rc[ok, 1]]
    return out


def dominant_angle(grid, F, band):
    """Main wall direction (deg, 0-90) from the distance-field gradient near walls."""
    gy, gx = np.gradient(F)
    m = band & (np.hypot(gx, gy) > 1e-9)
    if not m.any():
        return 0.0
    a = np.arctan2(gy[m], gx[m]) + grid.origin[2]
    return (math.degrees(math.atan2(np.sin(4 * a).sum(), np.cos(4 * a).sum())) / 4.0) % 90.0


def _targets(pieces, max_seg, kind):
    out = []
    for piece in pieces:
        out.extend(split_path([list(map(float, p)) for p in piece], max_seg, kind))
    return out


def _zigzag(grid, seq, spacing, collision, max_seg):
    """Join lanes into one continuous zigzag: the short hop from the end of one
    lane to the start of the next is driven with the brush on (it covers the
    strip along the wall between lane ends). A hop that is long or not safe
    starts a new piece instead (the optimiser connects it)."""
    out, cur = [], []
    for a, b in seq:
        a, b = list(map(float, a)), list(map(float, b))
        if cur and math.dist(cur[-1], a) <= 2.0 * spacing and grid.segment_safe(cur[-1], a, collision):
            if math.dist(cur[-1], a) > 1e-9:
                cur.append(a)
            cur.append(b)
        else:
            if len(cur) > 1:
                out.extend(split_path(cur, max_seg, 'lane'))
            cur = [a, b]
    if len(cur) > 1:
        out.extend(split_path(cur, max_seg, 'lane'))
    return out


# ----------------------------------------------------------- decomposition
def decompose(grid, Q, angle_deg, min_cells):
    """Boustrophedon decomposition of mask Q with sweep lines at angle_deg."""
    res = grid.resolution
    u, n = _unit(angle_deg)
    idx = np.argwhere(Q)
    xy = grid.world(idx)
    pu, pn = xy @ u, xy @ n
    u0, n0 = pu.min(), pn.min()
    W = int(round((pu.max() - u0) / res)) + 1
    H = int(round((pn.max() - n0) / res)) + 1
    jj, ii = np.meshgrid(np.arange(W), np.arange(H))
    pts = ((u0 + jj * res).reshape(-1, 1) * u + (n0 + ii * res).reshape(-1, 1) * n)
    rot = _inside(Q, _cells_of(grid, pts)).reshape(H, W)
    labrot = np.zeros((H, W), int)
    for k, cell in enumerate(planner.decompose(rot), 1):
        for y, a, b in cell:
            labrot[y, a:b + 1] = k
    ri = np.clip(np.rint((pn - n0) / res).astype(int), 0, H - 1)
    ci = np.clip(np.rint((pu - u0) / res).astype(int), 0, W - 1)
    if (labrot == 0).any():
        _, (ny, nx) = distance_transform_edt(labrot == 0, return_indices=True)
        labrot = labrot[ny, nx]
    L = np.zeros(grid.shape, int)
    L[idx[:, 0], idx[:, 1]] = labrot[ri, ci]
    return _merge_small(L, min_cells)


def _merge_small(L, min_cells):
    while True:
        sizes = np.bincount(L.ravel())
        sizes[0] = 0
        ids = [i for i in np.argsort(sizes) if 0 < sizes[i] < min_cells]
        if not ids:
            break
        merged = False
        for i in ids:
            border = {}
            m = L == i
            for sl_a, sl_b in [((slice(None, -1), slice(None)), (slice(1, None), slice(None))),
                               ((slice(1, None), slice(None)), (slice(None, -1), slice(None))),
                               ((slice(None), slice(None, -1)), (slice(None), slice(1, None))),
                               ((slice(None), slice(1, None)), (slice(None), slice(None, -1)))]:
                nb = L[sl_b][m[sl_a]]
                for j, c in zip(*np.unique(nb[(nb > 0) & (nb != i)], return_counts=True)):
                    border[j] = border.get(j, 0) + c
            if border:
                L[m] = max(border, key=border.get)
                merged = True
                break
        if not merged:
            break
    # relabel 1..n
    ids = np.unique(L[L > 0])
    out = np.zeros_like(L)
    for k, i in enumerate(ids, 1):
        out[L == i] = k
    return out, len(ids)


# ---------------------------------------------------------------- patterns
def _lane_at(grid, Qi, u, n, s, c, collision):
    """Safe straight pieces of the lane at offset c (list of [a, b])."""
    pts = s[:, None] * u + c * n
    ids = np.flatnonzero(_inside(Qi, _cells_of(grid, pts)))
    pieces = []
    for run in np.split(ids, np.flatnonzero(np.diff(ids) > 1) + 1):
        if len(run) < 2:
            continue
        start = last = pts[run[0]]
        for q in pts[run[1:]]:
            if grid.segment_safe(last, q, collision):
                last = q
            else:
                if np.linalg.norm(last - start) > 1e-9:
                    pieces.append([start, last])
                start = last = q
        if np.linalg.norm(last - start) > 1e-9:
            pieces.append([start, last])
    return pieces


def _lane_score(pieces):
    """Fewer pieces first (a lane chopped up by wall bumps is many short lines), then length."""
    L = sum(float(np.linalg.norm(np.asarray(b) - np.asarray(a))) for a, b in pieces)
    return (len(pieces), -L)


def lane_pieces(grid, Qi, angle_deg, spacing, collision, max_shift=0.0, offsets=None):
    """Straight lanes at angle_deg over region Qi: list of lanes, each a list of [a, b] along +u.

    The outermost lanes lie on the robot's safety limit. Along a bumpy wall such a
    lane is chopped into short pieces at every bump, so each outer lane may move up
    to `max_shift` inward if that keeps it in fewer, longer straight pieces (the
    brush then passes over the bumps instead of the robot weaving around them)."""
    res = grid.resolution
    u, n = _unit(angle_deg)
    xy = grid.world(np.argwhere(Qi))
    pu, pn = xy @ u, xy @ n
    s = np.arange(pu.min(), pu.max() + res / 4, res / 2)
    if offsets is not None:           # lanes at given offsets (shared lane grid)
        return [p for p in (_lane_at(grid, Qi, u, n, s, c, collision) for c in offsets) if p]
    lo, hi = pn.min(), pn.max()
    if max_shift > 0 and hi - lo > 2 * res:
        steps = np.arange(0.0, min(max_shift, 0.5 * (hi - lo)) + 1e-9, res / 2)
        best_lo = min(steps, key=lambda d: (_lane_score(_lane_at(grid, Qi, u, n, s, lo + d, collision)), d))
        best_hi = min(steps, key=lambda d: (_lane_score(_lane_at(grid, Qi, u, n, s, hi - d, collision)), d))
        lo, hi = lo + best_lo, hi - best_hi
    span = max(0.0, hi - lo)
    k = int(math.ceil(span / spacing - 1e-9))
    cs = [lo + span / 2] if k == 0 else list(lo + np.arange(k + 1) * span / k)
    lanes = []
    for c in cs:
        pieces = _lane_at(grid, Qi, u, n, s, c, collision)
        if pieces:
            lanes.append(pieces)
    return lanes


def lane_shift(settings):
    """How far an outer lane may leave the safety limit: the brush's extra reach
    beyond the robot body plus the edge band that may be left (cost function)."""
    r = float(settings['geometry']['coverage_disk_radius_m'])
    return max(0.0, r - settings.collision) + float(settings['cost'].get('edge_band_m', 0.0))


def order_lanes(lanes, reverse_lanes, flip):
    seq = []
    for i, pieces in enumerate(lanes[::-1] if reverse_lanes else lanes):
        if (i % 2 == 0) != flip:
            seq.extend(pieces)
        else:
            seq.extend([[b, a] for a, b in reversed(pieces)])
    return seq


def _quick_lane_cost(lanes, settings):
    seq = order_lanes(lanes, False, False)
    pts = [p for piece in seq for p in piece]
    return costmod.route_terms(pts, settings)


def _full_cost(grid, targets, reach, rem, coverable, settings):
    """Cost of a region plan (entered at its first point)."""
    if not targets:
        return math.inf
    collision = settings.collision
    route, xy = [], targets[0].start
    for t in targets:
        try:
            route.extend(planning.connect(grid, [t], xy, reach, collision)[0] + points_of(t))
        except ValueError:
            continue
        xy = t.end
    r = float(settings['geometry']['coverage_disk_radius_m'])
    covered = stroke_union(grid, rem, targets, r)
    cell = grid.resolution ** 2
    open_ = rem & coverable & ~covered
    return costmod.evaluate(route, targets, covered.sum() * cell, open_.sum() * cell, settings,
                            costmod.swept_floor(grid, rem, targets, r),
                            (open_ & costmod.edge_mask(grid, settings)).sum() * cell)['total_s']


class Region:
    """One interior region with its chosen pattern; yields entry variants."""

    def __init__(self, kind, angle=None, lanes=None, Qi=None, cost=math.inf):
        self.kind, self.angle, self.lanes, self.Qi, self.cost = kind, angle, lanes, Qi, cost
        self._variants = None
        self._pts = None

    def entry_options(self, grid, xy, settings):
        """[(distance from xy to entry, targets or None)]; None = build on selection."""
        if self.kind == 'lanes':
            if self._variants is None:
                max_seg = settings['planning']['max_segment_length_m']
                sp, col = spiral.path_spacing(settings), settings.collision
                self._variants = [_zigzag(grid, order_lanes(self.lanes, rev, flip), sp, col, max_seg)
                                  for rev in (False, True) for flip in (False, True)]
            return [(np.linalg.norm(np.asarray(v[0].start) - xy), v) for v in self._variants if v]
        if self.kind == 'rings':
            ends = np.array([p for t in self.lanes for p in (t.start, t.end)], float)
            return [(float(np.min(np.linalg.norm(ends - xy, axis=1))), None)]
        if self.kind == 'spiral':
            if self._pts is None:
                self._pts = grid.world(np.argwhere(self.Qi))
            return [(float(np.min(np.linalg.norm(self._pts - xy, axis=1))), None)]
        return [(np.linalg.norm(np.asarray(self.lanes[0].start) - xy), self.lanes)]

    def build(self, grid, xy, settings):
        if self.kind == 'rings':
            return _greedy_order(self.lanes, xy)
        return _region_spiral(grid, self.Qi, xy, settings)


def _greedy_order(targets, xy):
    """Nearest-neighbour order of loose pieces, each driven in its better direction."""
    left, out, xy = list(targets), [], np.asarray(xy, float)
    while left:
        best = None
        for i, t in enumerate(left):
            for rev in (False, True):
                a = np.asarray(t.end if rev else t.start)
                d = float(np.linalg.norm(a - xy))
                if best is None or d < best[0]:
                    best = (d, i, rev)
        _, i, rev = best
        t = left.pop(i)
        if rev:
            t = poly_target(list(reversed(points_of(t))), getattr(t, 'kind', 'spiral'))
        out.append(t)
        xy = np.asarray(t.end, float)
    return out


def _region_spiral(grid, Qi, xy, settings):
    F = distance_transform_edt(np.pad(Qi, 1))[1:-1, 1:-1] * grid.resolution - grid.resolution / 2
    roots, _ = spiral.build_tree(grid, F, settings)
    targets, _ = spiral.order_targets(grid, roots, xy, settings)
    for t in targets:
        t.kind = 'spiral'
    return targets


def plan_region(grid, Qi, rem, reach, coverable, settings, extra_angles=(), max_shift=0.0, spiral_targets=None,
                angles=None):
    """Cheapest pattern for one region: straight lanes (many angles) or a spiral.
    `spiral_targets` (optional) = ready-made spiral pieces clipped to this region."""
    spacing = spiral.path_spacing(settings)
    collision = settings.collision
    max_seg = settings['planning']['max_segment_length_m']
    n_full = FULL_EVAL_ANGLES
    if angles is None:
        angles = list(LANE_ANGLES) + list(extra_angles)
    else:                      # a short list: evaluate every angle fully
        n_full = len(angles)
    angles = sorted(set(round(a % 180, 3) for a in angles))
    screened = []
    for a in angles:
        lanes = lane_pieces(grid, Qi, a, spacing, collision, max_shift)
        if lanes:
            q = _quick_lane_cost(lanes, settings)
            screened.append((q['drive_s'] + q['turn_s'] + q['reverse_s'], a, lanes))
    screened.sort(key=lambda x: x[0])
    best = None
    for _, a, lanes in screened[:n_full]:
        c = _full_cost(grid, _zigzag(grid, order_lanes(lanes, False, False), spacing, collision, max_seg),
                       reach, rem, coverable, settings)
        if best is None or c < best.cost:
            best = Region('lanes', a, lanes, Qi, c)
    idx = np.argwhere(Qi)
    start = grid.world(idx[len(idx) // 2]).tolist()
    if spiral_targets is None:
        sp = _region_spiral(grid, Qi, start, settings)
        c = _full_cost(grid, sp, reach, rem, coverable, settings)
        if sp and (best is None or c < best.cost):
            best = Region('spiral', None, None, Qi, c)
    elif spiral_targets:
        c = _full_cost(grid, spiral_targets, reach, rem, coverable, settings)
        if best is None or c < best.cost:
            best = Region('rings', None, spiral_targets, Qi, c)
    if best is None:  # region too small for any pattern: one goal in its middle
        p = grid.world(idx[len(idx) // 2]).tolist()
        t = poly_target([p, p], 'lane')
        t.points = [p, list(p)]
        best = Region('point', None, [t], Qi, 0.0)
    return best


# ------------------------------------------------------------- wall loops
def wall_loops(grid, F, settings):
    """Outer wall loops and obstacle loops at the safety limit (world polylines)."""
    v0 = spiral.wall_level(grid.resolution)
    lab, _ = label(F >= v0, structure=np.ones((3, 3)))
    outer, rims = [], []
    r_cov = float(settings['geometry']['coverage_disk_radius_m'])
    for ln in spiral._contours(F, v0):
        o, _ = spiral._analyse(ln, F, lab, 'right')
        area = 0.5 * np.sum(o[:, 0] * np.roll(o[:, 1], -1) - np.roll(o[:, 0], -1) * o[:, 1])
        q, _ = spiral._analyse(ln, F, lab, settings['spiral']['wall_side'])
        w = spiral._to_world(grid, q)
        if np.max(np.linalg.norm(w - w.mean(axis=0), axis=1)) <= 0.5 * r_cov:
            continue
        (outer if area > 0 else rims).append(w)
    return outer, rims


def _loop_targets(grid, ring, xy, settings, kind='wall'):
    route = spiral._loop_route(ring, xy, 0.0)
    simple = spiral._simplify(route, spiral.SIMPLIFY * grid.resolution)
    out = []
    for part in spiral._safe_parts(grid, [q.tolist() for q in simple], settings.collision):
        out.extend(split_path(part, settings['planning']['max_segment_length_m'], kind))
    return out


# ------------------------------------------------------------------- plan
def plan_regions(grid, reach, den, coverable, polygon, start, settings, decomposition_angle, progress=None,
                 wall_angles=(), rims=True, ring_targets=None):
    """rims=True: loop around every obstacle, lanes stay inside those loops.
    rims=False: only the outer walls get a loop; lanes run straight up to the
    obstacles (the robot's safety limit) and the regions are split around them.
    ring_targets: a spiral; every region then chooses between the part of that
    spiral inside it (its rings, incl. loops around obstacles) and straight lanes."""
    res = grid.resolution
    r_cov = float(settings['geometry']['coverage_disk_radius_m'])
    spacing = spiral.path_spacing(settings)
    F = spiral.distance_field(grid, reach, polygon, r_cov, 'around')
    outer, rim_loops = wall_loops(grid, F, settings)
    if not rims:
        rim_loops = []

    targets, xy = [], np.asarray(start, float)
    nodes = []  # (kind, payload)
    if settings['strategy']['wall_loop_first']:
        while outer:
            i = min(range(len(outer)), key=lambda i: np.min(np.linalg.norm(outer[i] - xy, axis=1)))
            ts = _loop_targets(grid, outer.pop(i), xy, settings)
            targets += ts
            if ts:
                xy = np.asarray(ts[-1].end)
    nodes += [('loop', r) for r in outer + rim_loops]

    # everything the loops already cover
    loop_targets = list(targets)
    for r in outer + rim_loops:
        loop_targets += _loop_targets(grid, r, r[0], settings)
    rem = den & ~stroke_union(grid, den, loop_targets, r_cov)

    Q = F >= spiral.wall_level(res) + LANE_INSET * spacing
    if not rims:   # inset only from the outer walls; lanes reach up to the obstacles
        Fo = spiral.distance_field(grid, reach, polygon, r_cov, 'outer')
        Q = (Fo >= spiral.wall_level(res) + LANE_INSET * spacing) & (F >= spiral.wall_level(res))
    regions = []
    L = np.zeros(grid.shape, int)
    if Q.any():
        if decomposition_angle is None:   # whole interior as one region
            L, count = label(Q, structure=np.ones((3, 3)))
            L, count = _merge_small(L, max(4, int((MERGE_SPACINGS * spacing / res) ** 2)))
        else:
            L, count = decompose(grid, Q, decomposition_angle, max(4, int((MERGE_SPACINGS * spacing / res) ** 2)))
        if count:
            _, (ny, nx) = distance_transform_edt(L == 0, return_indices=True)
            owner = L[ny, nx]
            clipped = None
            if ring_targets is not None:   # the spiral minus its outer wall loop, cut per region
                band = Fo < spiral.wall_level(res) + 0.5 * spacing
                clipped = _clip_to_regions(grid, ring_targets, np.where(band, 0, owner), res)
            for i in range(1, count + 1):
                if progress:
                    progress(f'Region {i}/{count}: comparing lanes and spiral...')
                Qi = L == i
                extra = () if decomposition_angle is None else (decomposition_angle, decomposition_angle + 90)
                if clipped is None:
                    reg = plan_region(grid, Qi, rem & (owner == i), reach, coverable, settings,
                                      extra_angles=extra + tuple(wall_angles),
                                      max_shift=0.0 if rims else lane_shift(settings))
                else:       # lanes parallel to the walls or along the region's long axis
                    xy_i = grid.world(np.argwhere(Qi))
                    axis = []
                    if len(xy_i) > 2:
                        _, vec = np.linalg.eigh(np.cov((xy_i - xy_i.mean(axis=0)).T))
                        axis = [math.degrees(math.atan2(vec[1, -1], vec[0, -1]))]
                    reg = plan_region(grid, Qi, rem & (owner == i), reach, coverable, settings,
                                      max_shift=lane_shift(settings), spiral_targets=clipped.get(i, []),
                                      angles=list(extra) + list(wall_angles) + axis)
                regions.append(reg)
    nodes += [('region', r) for r in regions]

    # greedy ordering with the best entry variant of each node
    while nodes:
        best = None
        for k, (kind, obj) in enumerate(nodes):
            if kind == 'loop':
                j = int(np.argmin(np.linalg.norm(obj - xy, axis=1)))
                cand = [(np.linalg.norm(obj[j] - xy), None)]
            else:
                cand = obj.entry_options(grid, xy, settings)
            for d, v in cand:
                if best is None or d < best[0]:
                    best = (d, k, v)
        if best is None:
            break
        _, k, v = best
        kind, obj = nodes.pop(k)
        if kind == 'loop':
            ts = _loop_targets(grid, obj, xy, settings)
        else:
            ts = v if v is not None else obj.build(grid, xy, settings)
        targets += ts
        if ts:
            xy = np.asarray(ts[-1].end)
    info = {'regions': len(regions),
            'region_patterns': [(r.kind, r.angle) for r in regions],
            'decomposition_angle': decomposition_angle,
            'cells': L}                   # region id per grid cell (0 = wall loop band / outside)
    return targets, info


# ------------------------------------------------------------------ hybrid


def _clip_to_regions(grid, targets, owner, res):
    """Cut every target where it crosses from one region into the next.
    Returns {region id: [targets]} (pieces keep their kind)."""
    out = {}
    for t in targets:
        pts = np.asarray(points_of(t), float)
        kind = getattr(t, 'kind', 'spiral')
        if len(pts) < 2 or np.allclose(pts[0], pts[-1]) and len(pts) == 2:
            rc = grid.cell(pts[0])
            lab = int(owner[rc]) if grid.valid(rc) else 0
            out.setdefault(lab, []).append(t)
            continue
        dense = [pts[0]]
        for a, b in zip(pts, pts[1:]):
            m = max(1, int(math.ceil(np.linalg.norm(b - a) / (res / 2))))
            dense.extend(a + (b - a) * (np.arange(1, m + 1)[:, None] / m))
        dense = np.asarray(dense)
        rc = _cells_of(grid, dense)
        lab = np.where(rc[:, 0] >= 0, owner[np.clip(rc[:, 0], 0, None), np.clip(rc[:, 1], 0, None)], 0)
        cuts = np.flatnonzero(np.diff(lab) != 0) + 1
        for run in np.split(np.arange(len(dense)), cuts):
            if len(run) < 2:
                continue
            part = spiral._simplify(dense[run], spiral.SIMPLIFY * res)
            if np.linalg.norm(part[-1] - part[0]) < 1e-9 and len(part) <= 2:
                continue
            out.setdefault(int(lab[run[0]]), []).append(poly_target(part.tolist(), kind))
    return out


def plan_hybrid(grid, reach, den, coverable, polygon, start, settings, angle, ring_targets, progress=None,
                wall_angles=()):
    """Regional choice between spiral rings and straight lanes.

    After the outer wall loop, the rest of the room is split into regions. Every
    region keeps whichever is cheaper on the cost function: the part of the
    spiral (ring_targets, including its loops around obstacles) that lies in it,
    or straight lanes that run up to the obstacles (the outer lanes may move a
    little inward from a bumpy edge to stay one straight line). So a strip where
    lanes are better gets lanes even if rings win in the rest of the room."""
    return plan_regions(grid, reach, den, coverable, polygon, start, settings, angle, progress,
                        wall_angles, rims=False, ring_targets=ring_targets)


# --------------------------------------------------------------- display
def boustrophedon_cells(grid, reach, polygon, settings, angle, merge_spacings=MERGE_SPACINGS, mask=None):
    """Boustrophedon cell decomposition of the whole safe area (robot-centre
    positions), for display. Returns (labels, count): labels[r, c] = cell id, 0 outside.
    merge_spacings=0 shows the raw decomposition without merging small cells."""
    res = grid.resolution
    r_cov = float(settings['geometry']['coverage_disk_radius_m'])
    F = spiral.distance_field(grid, reach, polygon, r_cov, 'around')
    Q = F >= spiral.wall_level(res)
    if mask is not None:
        Q &= mask
    if not Q.any():
        return np.zeros(grid.shape, int), 0
    spacing = spiral.path_spacing(settings)
    min_cells = max(1, int((merge_spacings * spacing / res) ** 2)) if merge_spacings > 0 else 1
    return decompose(grid, Q, angle, min_cells)


# ------------------------------------------------- boustrophedon candidate
def _cell_offsets(grid, L, i, u, n, spacing, n0, need, collision=0.0):
    """Lane offsets for boustrophedon cell i: the room-wide lane grid n0 + k*spacing,
    plus a lane on each cell edge that borders a wall/obstacle (not another cell)
    when the nearest grid lane is more than `need` away from it."""
    res = grid.resolution
    xy = grid.world(np.argwhere(L == i))
    pu, pn = xy @ u, xy @ n
    lo, hi = pn.min(), pn.max()
    offs = list(n0 + spacing * np.arange(math.ceil((lo - n0) / spacing - 1e-9),
                                         math.floor((hi - n0) / spacing + 1e-9) + 1))
    for edge, step in ((lo, -1.0), (hi, 1.0)):
        rows = pu[np.abs(pn - edge) <= res]
        pts = rows[:, None] * u + (edge + step * res) * n
        rc = _cells_of(grid, pts)
        ok = rc[:, 0] >= 0
        other = np.zeros(len(rc), bool)
        other[ok] = L[rc[ok, 0], rc[ok, 1]] > 0
        if other.mean() < 0.5 and (not offs or min(abs(o - edge) for o in offs) > need):
            # along a bumpy wall a lane right on the edge is chopped up by every bump:
            # move it inward (at most half a spacing) to where it is longest in one piece
            Qi = L == i
            sm = np.arange(pu.min(), pu.max() + res / 4, res / 2)
            best = None
            for d in np.arange(0.0, 0.5 * spacing + 1e-9, res / 2):
                c = edge - step * d
                pieces = _lane_at(grid, Qi, u, n, sm, c, collision)
                Lp = sum(math.dist(a, b) for a, b in pieces)
                score = Lp - 0.3 * spacing * len(pieces) - d     # long, few pieces, close to the edge
                if best is None or score > best[0] + 1e-9:
                    best = (score, c)
            offs.append(best[1])
    return sorted(offs)


def plan_boustrophedon(grid, reach, den, coverable, polygon, start, settings, angle, progress=None,
                       wall_loop=False):
    """Classic boustrophedon coverage: split everything the robot can reach into
    boustrophedon cells (no wall loop, no obstacle loops), sweep every cell with
    one continuous zigzag of straight lanes on a room-wide lane grid at `angle`,
    and visit the cells in a greedy order. The route optimiser then orders the
    cells and adds detours for leftovers."""
    res = grid.resolution
    spacing = spiral.path_spacing(settings)
    collision = settings.collision
    need = max(0.0, float(settings['geometry']['coverage_disk_radius_m']) - collision) + 0.5 * res
    targets, xy = [], np.asarray(start, float)
    mask = None
    if wall_loop:        # outer walls first (a loop handles bumpy walls well), cells inside it
        r_cov = float(settings['geometry']['coverage_disk_radius_m'])
        F = spiral.distance_field(grid, reach, polygon, r_cov, 'around')
        Fo = spiral.distance_field(grid, reach, polygon, r_cov, 'outer')
        outer, _ = wall_loops(grid, F, settings)
        while outer:
            k = min(range(len(outer)), key=lambda k: np.min(np.linalg.norm(outer[k] - xy, axis=1)))
            ts = _loop_targets(grid, outer.pop(k), xy, settings)
            targets += ts
            if ts:
                xy = np.asarray(ts[-1].end)
        mask = Fo >= spiral.wall_level(res) + LANE_INSET * spacing
    L, count = boustrophedon_cells(grid, reach, polygon, settings, angle, merge_spacings=1.0, mask=mask)
    u, n = _unit(angle)
    n_all = grid.world(np.argwhere(L > 0)) @ n if count else np.zeros(1)
    n0 = n_all.min()
    regions = []
    for i in range(1, count + 1):
        if progress:
            progress(f'Boustrophedon cell {i}/{count}...')
        Qi = L == i
        lanes = lane_pieces(grid, Qi, angle, spacing, collision,
                            offsets=_cell_offsets(grid, L, i, u, n, spacing, n0, need, collision))
        if lanes:
            regions.append(Region('lanes', angle, lanes, Qi))
        else:
            idx = np.argwhere(Qi)
            p = grid.world(idx[len(idx) // 2]).tolist()
            t = poly_target([p, p], 'lane')
            t.points = [p, list(p)]
            regions.append(Region('point', None, [t], Qi, 0.0))
    nodes = list(regions)
    while nodes:
        best = None
        for k, obj in enumerate(nodes):
            for d, v in obj.entry_options(grid, xy, settings):
                if best is None or d < best[0]:
                    best = (d, k, v)
        if best is None:
            break
        _, k, v = best
        obj = nodes.pop(k)
        ts = v if v is not None else obj.build(grid, xy, settings)
        targets += ts
        if ts:
            xy = np.asarray(ts[-1].end)
    info = {'regions': count, 'region_patterns': [(r.kind, r.angle) for r in regions],
            'decomposition_angle': angle, 'cells': L}
    return targets, info
