"""Inward spiral coverage (contour-parallel offset loops).

The first loop follows the walls at the closest safe distance (robot radius).
Every next loop lies one path spacing further inward, where

    path spacing = 2 x coverage radius x (1 - overlap)

Loops are the level sets of a distance field, so they keep the shape of the
room. When the room splits into separate parts deeper inside (an L-shape, a
table in the middle) each part gets its own inward spiral, visited depth-first:
one part is finished down to its centre before the robot moves to the next.

Obstacle modes
  'around'  loops follow walls AND free-standing obstacles (full coverage)
  'outer'   loops only follow the outer shape of the room; where a loop would
            hit an obstacle it is cut, and the robot drives around it (A*)
"""
import math

import contourpy
import numpy as np
from scipy.ndimage import (binary_fill_holes, distance_transform_edt, find_objects, label,
                           maximum, maximum_position)

from .planning import poly_target, split_path

MAX_OVERLAP = 0.5
WALL_LEVEL = 0.49     # x cell: wall loop on the centres of the outermost safe cells (robot on its safety limit)
SIMPLIFY = 0.2        # x cell: how far simplified paths may deviate from the exact contour


def wall_level(res):
    return WALL_LEVEL * res + 1e-6


def path_spacing(settings):
    r = float(settings['geometry']['coverage_disk_radius_m'])
    overlap = min(max(float(settings['spiral']['overlap_pct']) / 100.0, 0.0), MAX_OVERLAP)
    return 2.0 * r * (1.0 - overlap)


def distance_field(grid, reach, polygon, coverage_radius, mode):
    """F = distance (m) inward from the boundary the loops should follow.

    F = 0 lies at the robot's safety limit along walls/obstacles, and one
    coverage radius inside the edge of a drawn area (so the brush reaches it).
    """
    res = grid.resolution
    base = reach if mode == 'around' else binary_fill_holes(reach)
    F = distance_transform_edt(np.pad(base, 1))[1:-1, 1:-1] * res - res / 2
    if polygon:
        inside = grid.polygon_mask(polygon)
        Fp = distance_transform_edt(np.pad(inside, 1))[1:-1, 1:-1] * res - res / 2 - coverage_radius
        F = np.minimum(F, Fp)
    return F


# --------------------------------------------------------------------- loops
def _lookup(arr, pts, fill):
    r = np.rint(pts[:, 1]).astype(int)
    c = np.rint(pts[:, 0]).astype(int)
    ok = (r >= 0) & (c >= 0) & (r < arr.shape[0]) & (c < arr.shape[1])
    out = np.full(len(pts), fill, dtype=arr.dtype)
    out[ok] = arr[r[ok], c[ok]]
    return out


def _analyse(line, F, lab, wall_side):
    """Orient a closed contour (index coords x=col, y=row) and find its component."""
    p = line[:-1] if np.allclose(line[0], line[-1]) else line
    t = np.roll(p, -1, axis=0) - np.roll(p, 1, axis=0)
    norm = np.linalg.norm(t, axis=1, keepdims=True)
    norm[norm == 0] = 1
    n = np.stack([-t[:, 1], t[:, 0]], axis=1) / norm * 0.75   # left normal, 0.75 cell
    left = _lookup(F, p + n, -np.inf)
    right = _lookup(F, p - n, -np.inf)
    interior_left = np.sum(left > right) >= np.sum(right > left)
    inner = p + n if interior_left else p - n
    labels = _lookup(lab, inner, 0)
    labels = labels[labels > 0]
    comp = int(np.bincount(labels).argmax()) if len(labels) else 0
    # wall on the robot's right  <=>  free interior on its left
    if interior_left != (wall_side == 'right'):
        p = p[::-1]
    return p, comp


def _contours(F, level):
    Fp = np.pad(F, 1, constant_values=-1e9)
    gen = contourpy.contour_generator(z=Fp, line_type='Separate')
    return [ln - 1.0 for ln in gen.lines(level) if len(ln) >= 4]


def _to_world(grid, idx, offset=(0, 0)):
    rc = np.stack([idx[:, 1] + offset[0], idx[:, 0] + offset[1]], axis=1)
    return grid.world(rc)


class Node:
    def __init__(self, level, value):
        self.level, self.value = level, value
        self.loops, self.points, self.children = [], [], []
        self.parent = None
        self.max_value = value
        self.max_xy = None

    def all_points(self):
        pts = [lp for lp in self.loops] + [np.asarray(self.points)] if self.points else list(self.loops)
        return np.vstack(pts) if pts else np.zeros((0, 2))


def _add_loops(node, lines, grid, F, lab, wall_side, coverage_radius, offset=(0, 0)):
    for ln in lines:
        p, _ = _analyse(ln, F, lab, wall_side)
        w = _to_world(grid, p, offset)
        c = w.mean(axis=0)
        if np.max(np.linalg.norm(w - c, axis=1)) <= 0.8 * coverage_radius:
            node.points.append(c.tolist())  # tiny loop: one goal in the middle covers it
        else:
            node.loops.append(w)


def build_tree(grid, F, settings):
    res = grid.resolution
    r_cov = float(settings['geometry']['coverage_disk_radius_m'])
    spacing = path_spacing(settings)
    side = settings['spiral']['wall_side']
    v0 = wall_level(res)  # centres of the outermost safe cells (robot radius from walls)
    roots, all_nodes = [], []
    prev_lab, prev_nodes = None, None
    k = 0
    while True:
        v = v0 + k * spacing
        mask = F >= v
        lab, n = label(mask, structure=np.ones((3, 3))) if mask.any() else (None, 0)
        nodes = {}
        if n:
            idx = list(range(1, n + 1))
            maxv = maximum(F, lab, idx)
            maxp = maximum_position(F, lab, idx)
            for c in idx:
                node = Node(k, v)
                node.max_value = float(maxv[c - 1])
                node.max_xy = grid.world(maxp[c - 1]).tolist()
                nodes[c] = node
                if prev_lab is None:
                    roots.append(node)
                else:
                    parent = prev_nodes[int(prev_lab[maxp[c - 1]])]
                    node.parent = parent
                    parent.children.append(node)
            by_comp = {}
            for ln in _contours(F, v):
                _, comp = _analyse(ln, F, lab, side)
                by_comp.setdefault(comp, []).append(ln)
            for c, node in nodes.items():
                _add_loops(node, by_comp.get(c, []), grid, F, lab, side, r_cov)
                if not node.loops and not node.points:
                    node.points.append(node.max_xy)
                all_nodes.append(node)
        # components of the previous level that ended here: fill their centre
        if prev_nodes:
            slices = find_objects(prev_lab)
            for c, node in prev_nodes.items():
                if node.children or node.max_value - node.value <= 0.9 * r_cov:
                    continue
                w = min(node.max_value - 0.25 * res, max(node.value + 0.5 * spacing, node.max_value - 0.9 * r_cov))
                sl = slices[c - 1]
                sub_lab = prev_lab[sl]
                Fc = np.where(sub_lab == c, F[sl], -1e9)
                extra = Node(node.level + 1, w)
                extra.parent = node
                offset = (sl[0].start, sl[1].start)
                _add_loops(extra, _contours(Fc, w), grid, Fc, (sub_lab == c).astype(int), side, r_cov, offset)
                if not extra.loops and not extra.points:
                    extra.points.append(node.max_xy)
                node.children.append(extra)
                all_nodes.append(extra)
        if not n:
            break
        prev_lab, prev_nodes = lab, nodes
        k += 1
    return roots, all_nodes


# --------------------------------------------------------------- traversal
def _simplify(pts, tol):
    """Douglas-Peucker on an (N,2) polyline."""
    n = len(pts)
    if n < 3:
        return pts
    keep = np.zeros(n, bool)
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        a, b = pts[i], pts[j]
        ab = b - a
        L = np.hypot(*ab)
        seg = pts[i + 1:j] - a
        d = np.abs(ab[0] * seg[:, 1] - ab[1] * seg[:, 0]) / L if L > 1e-12 else np.linalg.norm(seg, axis=1)
        m = int(np.argmax(d))
        if d[m] > tol:
            keep[i + 1 + m] = True
            stack += [(i, i + 1 + m), (i + 1 + m, j)]
    return pts[keep]


def _safe_parts(grid, pts, radius):
    parts, cur = [], [pts[0]]
    for q in pts[1:]:
        if grid.segment_safe(cur[-1], q, radius):
            cur.append(q)
        else:
            if len(cur) > 1:
                parts.append(cur)
            cur = [q]
    if len(cur) > 1:
        parts.append(cur)
    return parts


def _loop_route(ring, xy, advance):
    """Closed route around `ring`, starting a little ahead of the point nearest xy."""
    ring = np.asarray(ring)
    j = int(np.argmin(np.linalg.norm(ring - xy, axis=1)))
    seg = np.linalg.norm(np.diff(np.vstack([ring, ring[:1]]), axis=0), axis=1)
    L = seg.sum()
    adv = min(advance, 0.25 * L)
    walked = 0.0
    while walked < adv:
        walked += seg[j]
        j = (j + 1) % len(ring)
    return np.vstack([ring[j:], ring[:j], ring[j:j + 1]])


def order_targets(grid, roots, start, settings):
    spacing = path_spacing(settings)
    collision = settings.collision
    max_seg = settings['planning']['max_segment_length_m']
    tol = SIMPLIFY * grid.resolution
    targets = []
    state = {'xy': np.asarray(start, float), 'loops': 0}

    def emit_route(route, kind):
        simple = _simplify(route, tol)
        for part in _safe_parts(grid, [p.tolist() for p in simple], collision):
            for t in split_path(part, max_seg, kind):
                targets.append(t)
                state['xy'] = np.asarray(t.end)

    def visit(node):
        kind = 'wall' if node.level == 0 else 'spiral'
        loops = list(node.loops)
        points = list(node.points)
        while loops:
            i = min(range(len(loops)), key=lambda i: np.min(np.linalg.norm(loops[i] - state['xy'], axis=1)))
            ring = loops.pop(i)
            emit_route(_loop_route(ring, state['xy'], spacing), kind)
            state['loops'] += 1
        while points:
            i = min(range(len(points)), key=lambda i: math.dist(points[i], state['xy']))
            p = points.pop(i)
            rc = grid.cell(p)
            if grid.valid(rc) and grid.clearance[rc] >= collision:
                t = poly_target([p, p], 'spiral')
                t.points = [list(p), list(p)]
                targets.append(t)
                state['xy'] = np.asarray(p)
        children = list(node.children)
        while children:
            i = min(range(len(children)),
                    key=lambda i: np.min(np.linalg.norm(children[i].all_points() - state['xy'], axis=1))
                    if len(children[i].all_points()) else math.inf)
            visit(children.pop(i))

    remaining = list(roots)
    while remaining:
        i = min(range(len(remaining)),
                key=lambda i: np.min(np.linalg.norm(remaining[i].all_points() - state['xy'], axis=1))
                if len(remaining[i].all_points()) else math.inf)
        visit(remaining.pop(i))
    return targets, state['loops']


def spiral_targets(grid, reach, polygon, start, settings):
    r_cov = float(settings['geometry']['coverage_disk_radius_m'])
    F = distance_field(grid, reach, polygon, r_cov, settings['spiral']['obstacle_mode'])
    roots, nodes = build_tree(grid, F, settings)
    targets, loops = order_targets(grid, roots, start, settings)
    levels = 1 + max((n.level for n in nodes), default=-1)
    return targets, {'loops': loops, 'rings': levels}


# ------------------------------------------------------------ centre lines
def skeletonize(mask):
    """Zhang-Suen thinning of a boolean image to 1-pixel-wide centre lines
    (numpy only, so scikit-image is not needed)."""
    img = np.pad(np.asarray(mask, bool), 1).astype(np.uint8)
    if not img.any():
        return img[1:-1, 1:-1].astype(bool)
    rows, cols = np.nonzero(img)
    r0, r1, c0, c1 = max(rows.min() - 1, 0), rows.max() + 2, max(cols.min() - 1, 0), cols.max() + 2
    sub = img[r0:r1, c0:c1].copy()
    changed = True
    while changed:
        changed = False
        for step in (0, 1):
            P = np.pad(sub, 1)
            p2, p3, p4 = P[:-2, 1:-1], P[:-2, 2:], P[1:-1, 2:]
            p5, p6, p7 = P[2:, 2:], P[2:, 1:-1], P[2:, :-2]
            p8, p9 = P[1:-1, :-2], P[:-2, :-2]
            nb = [p2, p3, p4, p5, p6, p7, p8, p9]
            B = sum(n.astype(int) for n in nb)
            seq = nb + [p2]
            A = sum(((seq[i] == 0) & (seq[i + 1] == 1)).astype(int) for i in range(8))
            if step == 0:
                c = (p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0)
            else:
                c = (p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0)
            remove = (sub == 1) & (B >= 2) & (B <= 6) & (A == 1) & c
            if remove.any():
                sub[remove] = 0
                changed = True
    img[r0:r1, c0:c1] = sub
    return img[1:-1, 1:-1].astype(bool)


def _trace_skeleton(skel):
    """Split a skeleton image into pixel polylines (list of (N,2) row/col arrays)."""
    pix = set(map(tuple, np.argwhere(skel)))
    nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    def around(p):
        return [(p[0] + a, p[1] + b) for a, b in nbrs if (p[0] + a, p[1] + b) in pix]

    lines, seen = [], set()
    while pix - seen:
        todo = pix - seen
        ends = [p for p in todo if len([q for q in around(p) if q not in seen]) <= 1]
        cur = min(ends) if ends else min(todo)
        line = [cur]
        seen.add(cur)
        while True:
            nxt = [q for q in around(cur) if q not in seen]
            if not nxt:
                break
            straight = [q for q in nxt if q[0] == cur[0] or q[1] == cur[1]]
            cur = (straight or nxt)[0]
            seen.add(cur)
            line.append(cur)
        lines.append(np.array(line, float))
    return lines


def centre_line_fill(grid, targets, region, reach, settings):
    """Paths along the thin strips a spiral leaves between opposite loops.

    Where two loops meet from opposite sides (the middle of a corridor, the
    ridge between a table and a wall) their distance can be up to one path
    spacing, which leaves a narrow uncovered strip. This drives along the
    centre line of every such strip.
    """
    from scipy.ndimage import binary_dilation
    from .planning import stroke_union

    res = grid.resolution
    r_cov = float(settings['geometry']['coverage_disk_radius_m'])
    n = int(math.ceil(r_cov / res))
    yy, xx = np.mgrid[-n:n + 1, -n:n + 1]
    disk = np.hypot(xx, yy) * res <= r_cov
    covered = stroke_union(grid, region, targets, r_cov)
    # Along walls the wall loop already runs at the safety limit; a 1-cell strip left
    # there is grid rounding, and chasing it would mean driving the wall again.
    # So only there, allow one cell of tolerance. Interior ridges are checked exactly.
    near_wall = grid.clearance < settings.collision + r_cov
    covered |= near_wall & stroke_union(grid, region & near_wall, targets, r_cov + res)
    residual = region & ~covered & binary_dilation(reach, disk)
    lab, count = label(residual, structure=np.ones((3, 3)))
    if not count:
        return []
    sizes = np.bincount(lab.ravel())
    keep = sizes >= max(2, int(round(r_cov / res)))  # ignore specks shorter than one coverage radius
    keep[0] = False
    residual = keep[lab]
    if not residual.any():
        return []
    positions = reach & binary_dilation(residual, disk)
    out = []
    max_seg = settings['planning']['max_segment_length_m']
    for line in _trace_skeleton(skeletonize(positions)):
        pts = grid.world(line)
        if len(pts) < 3 or planning_length(pts) < 2 * res:
            p = pts.mean(axis=0).tolist()
            if grid.segment_safe(p, p, settings.collision):
                t = poly_target([p, p], 'fill')
                t.points = [p, list(p)]
                out.append(t)
            continue
        simple = _simplify(pts, SIMPLIFY * res)
        for part in _safe_parts(grid, [q.tolist() for q in simple], settings.collision):
            out.extend(split_path(part, max_seg, 'fill'))
    return out


def planning_length(pts):
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())


def insert_cheapest(targets, extra, start):
    """Insert each extra segment where it adds the least straight-line detour."""
    seq = list(targets)
    for f in extra:
        ends = np.array([start] + [t.end for t in seq], float)          # position before slot i
        nexts = np.array([t.start for t in seq] + [ends[-1]], float)     # position after slot i
        base = np.linalg.norm(nexts - ends, axis=1)
        base[-1] = 0.0
        best = None
        for rev in (False, True):
            a, b = (np.asarray(f.end), np.asarray(f.start)) if rev else (np.asarray(f.start), np.asarray(f.end))
            back = np.linalg.norm(nexts - b, axis=1)
            back[-1] = 0.0
            cost = np.linalg.norm(ends - a, axis=1) + back - base
            i = int(np.argmin(cost))
            if best is None or cost[i] < best[0]:
                best = (cost[i], i, rev)
        _, i, rev = best
        if rev:
            f = poly_target(list(reversed(f.points)), 'fill')
        seq.insert(i, f)
    return seq


# ------------------------------------------------- outer mode: obstacle rims
def obstacle_rims(grid, reach, polygon, settings):
    """One loop around every free-standing obstacle, at robot-radius distance.

    Used in 'outer' mode, where the spiral itself keeps the room shape and
    does not circle obstacles, so their edges would otherwise be missed.
    """
    res = grid.resolution
    r_cov = float(settings['geometry']['coverage_disk_radius_m'])
    F = distance_field(grid, reach, polygon, r_cov, 'around')
    lab, _ = label(F >= wall_level(res), structure=np.ones((3, 3)))
    rims = []
    for ln in _contours(F, wall_level(res)):
        p = ln[:-1] if np.allclose(ln[0], ln[-1]) else ln
        area = 0.5 * np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1])
        oriented, _ = _analyse(ln, F, lab, 'right')   # right => free space on the left
        o_area = 0.5 * np.sum(oriented[:, 0] * np.roll(oriented[:, 1], -1) - np.roll(oriented[:, 0], -1) * oriented[:, 1])
        if o_area < 0:  # free space on the left while turning clockwise => loop around an obstacle
            q, _ = _analyse(ln, F, lab, settings['spiral']['wall_side'])
            rims.append(_to_world(grid, q))
    return rims


def insert_loops(grid, targets, loops, start, settings, kind='wall'):
    """Insert closed loops into the target sequence where they cost the least detour."""
    max_seg = settings['planning']['max_segment_length_m']
    seq = list(targets)
    for ring in loops:
        ends = np.array([start] + [t.end for t in seq], float)
        nexts = np.array([t.start for t in seq] + [ends[-1]], float)
        base = np.linalg.norm(nexts - ends, axis=1)
        base[-1] = 0.0
        d = np.linalg.norm(ends[:, None, :] - ring[None, :, :], axis=2)   # slots x ring points
        j = d.argmin(axis=1)
        back = np.linalg.norm(nexts - ring[j], axis=1)
        back[-1] = 0.0
        cost = d[np.arange(len(ends)), j] + back - base
        i = int(np.argmin(cost))
        route = _loop_route(ring, ends[i], 0.0)
        new = []
        for part in _safe_parts(grid, [q.tolist() for q in _simplify(route, SIMPLIFY * grid.resolution)], settings.collision):
            new.extend(split_path(part, max_seg, kind))
        seq[i:i] = new
    return seq
