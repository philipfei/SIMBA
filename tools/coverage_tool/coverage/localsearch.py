"""Cost-driven local search over the complete route.

The route is a sequence of pieces (a lane, a loop, a spiral ring, a centre
line, a single goal). The optimiser repeatedly applies whichever move lowers
the *total* cost from cost.py:

  drop      remove a piece (saves its driving; costs missed cells it alone covers)
  flip      drive a piece in the other direction
  move      take a piece out and insert it elsewhere (either direction)
  2-opt     reverse a whole stretch of the route
  rotate    enter a closed loop at the point that is cheapest to reach and leave
  straighten  replace a wiggly piece (a loop along a bumpy wall) by fewer,
            longer straight lines that are still safe to drive
  detour    add a goal that covers still-missed cells, where the missed-cell
            penalty it removes outweighs the extra driving and turning

Every move is scored with exact cost differences: transition time (straight
connector or A* around obstacles), stop-and-turn costs at every junction,
hairpin penalties, double coverage and missed reachable cells. Nothing is
forced: with a very high missed-cell weight covering everything is simply the
cheapest option; with a low weight tiny specks may be left if a detour costs
more than they are worth.
"""
import math
import time

import numpy as np

from . import cost as costmod
from .planning import poly_target, split_path, stroke_union, visible_many

EPS = 1e-6


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _first_heading(pts):
    for p, q in zip(pts, pts[1:]):
        if math.dist(p, q) > EPS:
            return math.atan2(q[1] - p[1], q[0] - p[0])
    return None


def _last_heading(pts):
    for p, q in zip(pts[-2::-1], pts[::-1]):
        if math.dist(p, q) > EPS:
            return math.atan2(q[1] - p[1], q[0] - p[0])
    return None


class Piece:
    def __init__(self, pts, kind):
        self.pts = [list(map(float, p)) for p in pts]
        self.kind = kind
        self.point = len(self.pts) < 2 or all(math.dist(self.pts[0], q) < EPS for q in self.pts)
        self.h0 = None if self.point else _first_heading(self.pts)
        self.h1 = None if self.point else _last_heading(self.pts)
        self.h = None     # single goal: the heading the robot has while passing it (set by orient())

    def state(self, rev):
        """(start xy, start heading, end xy, end heading) when driven forwards/backwards."""
        a, b = self.pts[0], self.pts[-1]
        if self.point:    # the turn at a single goal = turn onto h + turn from h to the way out
            return a, self.h, a, self.h
        if not rev:
            return a, self.h0, b, self.h1
        f = lambda h: None if h is None else _wrap(h + math.pi)
        return b, f(self.h1), a, f(self.h0)

    def targets(self, rev, max_seg):
        pts = self.pts[::-1] if rev else self.pts
        if self.point:
            t = poly_target([pts[0], pts[0]], self.kind)
            t.points = [list(pts[0]), list(pts[0])]
            t.kind = self.kind
            return [t]
        return split_path(pts, max_seg, self.kind)


def pieces_from_targets(targets):
    """Join consecutive, continuous targets of the same kind into pieces."""
    pieces, cur, kind = [], None, None
    for t in targets:
        pts = [list(map(float, p)) for p in (getattr(t, 'points', None) or [t.start, t.end])]
        k = getattr(t, 'kind', 'spiral')
        is_point = len(pts) < 2 or math.dist(pts[0], pts[-1]) < EPS and len(pts) <= 2
        if cur and not is_point and k == kind and math.dist(cur[-1], pts[0]) < EPS:
            cur.extend(pts[1:])
            continue
        if cur:
            pieces.append(Piece(cur, kind))
        cur, kind = pts, k
        if is_point:
            pieces.append(Piece(pts[:1], k))
            cur = None
    if cur:
        pieces.append(Piece(cur, kind))
    return pieces


class RouteOptimizer:
    def __init__(self, grid, pieces, start, reach, den, coverable, settings, neighbours=8):
        self.g, self.reach, self.den = grid, reach, den
        self.start = list(map(float, start))
        self.s = settings
        c = settings['cost']
        self.v = float(settings['motion']['linear_m_s'])
        self.w = float(settings['motion']['angular_rad_s'])
        self.smooth = math.radians(float(c['smooth_turn_deg']))
        self.hairpin = math.radians(float(c['reverse_angle_deg']))
        self.stop = float(c['stop_penalty_s'])
        self.revpen = float(c['reverse_penalty_s'])
        self.bend = float(c.get('bend_s_per_rad', 0.0))
        at = costmod.area_time(settings)
        self.cell = grid.resolution ** 2
        self.k_over = float(c['overlap_weight']) * at           # s per m2 double covered
        self.k_miss = float(c['missed_weight']) * at * self.cell  # s per missed cell
        # cells right against a wall/obstacle (the bottom of a recess between lidar bumps)
        # may have their own, lower weight: that is what lets a line stay straight
        edge = (grid.clearance < float(c.get('edge_band_m', 0.0))).ravel()
        k_edge = float(c.get('edge_missed_weight', c['missed_weight'])) * at * self.cell
        self.wmiss = np.where(edge, k_edge, self.k_miss)
        self.r = float(settings['geometry']['coverage_disk_radius_m'])
        self.collision = settings.collision
        self.K = neighbours
        self.cov_ok = coverable.ravel().copy()
        self.count = np.zeros(grid.cells.size, np.int32)
        self._conn = {}
        self.pieces = []
        self.seq = []   # list of [piece index, reversed]
        for p in pieces:
            self._register(p)
            self.seq.append([len(self.pieces) - 1, False])

    def _miss(self, idx):
        """Missed-area cost (s) of the given cells if none of them were covered."""
        idx = np.asarray(idx, dtype=np.int64)
        return float(self.wmiss[idx][self.cov_ok[idx]].sum()) if len(idx) else 0.0

    # ------------------------------------------------------------ pieces
    def _register(self, p):
        self._measure(p)
        self.pieces.append(p)
        self.count[p.cov] += 1
        return len(self.pieces) - 1

    def _measure(self, p):
        """Coverage cells, swept area and internal time of a piece."""
        p.cov = costmod.piece_cells(self.g, self.den, p.pts, self.r)
        p.swept = len(p.cov) * self.cell       # floor only: brush over walls/obstacles is free
        terms = costmod.route_terms(p.pts, self.s) if not p.point else None
        p.internal = (terms['drive_s'] + terms['turn_s'] + terms['reverse_s']) if terms else \
            0.0                                  # turning at a single goal is charged at its junctions
        return p

    # ------------------------------------------------------- transitions
    def _turn(self, h1, h2):
        """Heading change at a junction between pieces: treated as a real corner."""
        if h1 is None or h2 is None:
            return 0.0
        d = abs(_wrap(h2 - h1))
        if d <= self.smooth:
            return d * self.bend
        cost = d / self.w + self.stop
        if d >= self.hairpin:
            cost += self.revpen
        return cost

    def _connector(self, a, b):
        key = (round(a[0], 4), round(a[1], 4), round(b[0], 4), round(b[1], 4))
        if key in self._conn:
            return self._conn[key]
        g = self.g
        if g.segment_safe(a, b, self.collision):
            path = [a, b]
        else:
            path = g.astar(a, b, self.reach, self.collision)
            if path:
                compact, i = [path[0]], 0
                while i < len(path) - 1:
                    j = len(path) - 1
                    while j > i + 1 and not g.segment_safe(path[i], path[j], self.collision):
                        j -= 1
                    compact.append(path[j])
                    i = j
                path = compact
        if not path:
            res = None
        else:
            terms = costmod.route_terms(path, self.s)
            res = (terms['drive_s'] + terms['turn_s'] + terms['reverse_s'],
                   _first_heading(path), _last_heading(path), path)
        self._conn[key] = res
        return res

    def T(self, frm, to):
        """Seconds from the end of state `frm` (None = robot start) to the start of state `to`."""
        if to is None:
            return 0.0
        if frm is None:
            xy, h = self.start, None
        else:
            _, _, xy, h = self.pieces[frm[0]].state(frm[1])
        sxy, sh, _, _ = self.pieces[to[0]].state(to[1])
        if math.dist(xy, sxy) < EPS:
            return self._turn(h, sh)
        c = self._connector(xy, sxy)
        if c is None:
            return 1e7
        t, ho, hi, _ = c
        return t + self._turn(h, ho) + self._turn(hi, sh)

    def _slot_cost(self, a, st, b):
        """T(a, st) + T(st, b). A single goal gets its best heading for this slot
        (the way in or the way out), so its turn is charged correctly."""
        p = self.pieces[st[0]]
        if not p.point:
            return self.T(a, st) + self.T(st, b)
        xy = p.pts[0]
        cands = []
        fa = self.start if a is None else self.pieces[a[0]].state(a[1])[2]
        hb = self._heading_between(fa, xy)
        if hb:
            cands.append(hb[1])
        elif a is not None:
            cands.append(self.pieces[a[0]].state(a[1])[3])
        if b is not None:
            hb = self._heading_between(xy, self.pieces[b[0]].state(b[1])[0])
            if hb:
                cands.append(hb[0])
        old, best = p.h, None
        for h in cands or [None]:
            p.h = h
            c = self.T(a, st) + self.T(st, b)
            if best is None or c < best[0]:
                best = (c, h)
        p.h = old
        return best[0]

    def _at(self, k):
        return tuple(self.seq[k]) if 0 <= k < len(self.seq) else None

    # --------------------------------------------------------------- cost
    def total(self):
        time_s = sum(self.pieces[i].internal for i, _ in self.seq)
        prev = None
        for st in self.seq:
            time_s += self.T(prev, tuple(st))
            prev = tuple(st)
        covered = self.count > 0
        swept = sum(self.pieces[i].swept for i, _ in self.seq)
        overlap = max(0.0, swept - covered.sum() * self.cell)
        open_ = self.cov_ok & ~covered
        return time_s + self.k_over * overlap + float(self.wmiss[open_].sum()), int(open_.sum())

    # -------------------------------------------------------------- moves
    def _drop_delta(self, k):
        i, rv = self.seq[k]
        p = self.pieces[i]
        prev, cur, nxt = self._at(k - 1), (i, rv), self._at(k + 1)
        dt = -p.internal - self.T(prev, cur) - self.T(cur, nxt) + self.T(prev, nxt)
        uniq = p.cov[self.count[p.cov] == 1]
        return dt + self.k_over * (-p.swept + len(uniq) * self.cell) + self._miss(uniq)

    def _drop(self, k):
        i, _ = self.seq.pop(k)
        self.count[self.pieces[i].cov] -= 1

    def _neighbour_positions(self, k):
        """Positions whose end lies near the start or end of the piece at k."""
        if len(self.seq) < 3:
            return []
        ends = np.array([self.pieces[i].state(rv)[2] for i, rv in self.seq])
        i, rv = self.seq[k]
        a, _, b, _ = self.pieces[i].state(rv)
        d = np.minimum(np.linalg.norm(ends - a, axis=1), np.linalg.norm(ends - b, axis=1))
        d[k] = np.inf
        return list(np.argsort(d)[:self.K])

    def improve_once(self, deadline):
        """One pass over the route; returns number of improving moves applied."""
        applied = 0
        k = 0
        while k < len(self.seq):
            if time.monotonic() > deadline:
                break
            i, rv = self.seq[k]
            prev, cur, nxt = self._at(k - 1), (i, rv), self._at(k + 1)
            # drop
            if self._drop_delta(k) < -1e-6:
                self._drop(k)
                applied += 1
                continue
            # flip
            if not self.pieces[i].point:
                flip = (i, not rv)
                d = self.T(prev, flip) + self.T(flip, nxt) - self.T(prev, cur) - self.T(cur, nxt)
                if d < -1e-6:
                    self.seq[k][1] = not rv
                    applied += 1
                    continue
            best = None
            # move elsewhere
            removal = self.T(prev, nxt) - self.T(prev, cur) - self.T(cur, nxt)
            for j in self._neighbour_positions(k):
                if j in (k - 1, k):
                    continue
                a, b = self._at(j), self._at(j + 1)
                for o in ((False, True) if not self.pieces[i].point else (False,)):
                    st = (i, o)
                    d = removal + self._slot_cost(a, st, b) - self.T(a, b)
                    if d < -1e-6 and (best is None or d < best[0]):
                        best = (d, 'move', j, o)
            # 2-opt: reverse the stretch between k and j
            for j in self._neighbour_positions(k):
                lo, hi = min(k, j), max(k, j)
                if hi - lo < 1:
                    continue
                A, B = self._at(lo), self._at(hi)
                p0, n0 = self._at(lo - 1), self._at(hi + 1)
                Br, Ar = (B[0], not B[1]), (A[0], not A[1])
                d = self.T(p0, Br) + self.T(Ar, n0) - self.T(p0, A) - self.T(B, n0)
                if d < -1e-6 and (best is None or d < best[0]):
                    best = (d, '2opt', lo, hi)
            if best:
                if best[1] == 'move':
                    _, _, j, o = best
                    item = self.seq.pop(k)
                    item[1] = o
                    self.seq.insert(j + 1 if j < k else j, item)
                else:
                    _, _, lo, hi = best
                    block = [[i2, not r2] for i2, r2 in reversed(self.seq[lo:hi + 1])]
                    self.seq[lo:hi + 1] = block
                applied += 1
                continue
            k += 1
        return applied

    # --------------------------------------------------------------- trim
    @staticmethod
    def _cut(pts, d, from_end):
        """Polyline with length d removed from one end (None if nothing would remain)."""
        pts = [list(p) for p in (pts[::-1] if from_end else pts)]
        while len(pts) >= 2:
            seg = math.dist(pts[0], pts[1])
            if seg > d + EPS:
                f = d / seg
                pts[0] = [pts[0][0] + f * (pts[1][0] - pts[0][0]), pts[0][1] + f * (pts[1][1] - pts[0][1])]
                break
            d -= seg
            pts.pop(0)
        if len(pts) < 2 or all(math.dist(pts[0], q) < EPS for q in pts):
            return None
        return pts[::-1] if from_end else pts

    def _replace_delta(self, k, q):
        """Exact cost change of driving piece q instead of the piece at position k
        (q is registered in self.pieces; returns (delta, index of q))."""
        i, rv = self.seq[k]
        p = self.pieces[i]
        self.count[p.cov] -= 1
        lost = p.cov[self.count[p.cov] == 0]
        self.count[p.cov] += 1
        lost = np.setdiff1d(lost, q.cov, assume_unique=False)
        gainq = np.setdiff1d(q.cov, p.cov)
        gained = gainq[self.count[gainq] == 0]
        prev, nxt = self._at(k - 1), self._at(k + 1)
        self.pieces.append(q)
        qi = len(self.pieces) - 1
        dt = (q.internal - p.internal + self.T(prev, (qi, rv)) + self.T((qi, rv), nxt)
              - self.T(prev, (i, rv)) - self.T((i, rv), nxt))
        dcov = len(gained) - len(lost)
        d = (dt + self.k_over * (q.swept - p.swept - dcov * self.cell)
             + (self._miss(lost) - self._miss(gained)))
        return d, qi

    def _commit(self, k, qi):
        i = self.seq[k][0]
        self.count[self.pieces[i].cov] -= 1
        self.count[self.pieces[qi].cov] += 1
        self.seq[k][0] = qi

    def trim(self, deadline, step):
        """Shorten open pieces from either end while that lowers the cost."""
        applied = 0
        for k in range(len(self.seq)):
            if time.monotonic() > deadline:
                break
            for from_end in (False, True):
                for _ in range(20):
                    i, rv = self.seq[k]
                    p = self.pieces[i]
                    if p.point:                  # closed loops may be opened: their seam double-covers
                        break
                    cut = self._cut(p.pts, step, from_end)
                    if cut is None:
                        break
                    d, qi = self._replace_delta(k, self._measure(Piece(cut, p.kind)))
                    if d < -1e-6:
                        self._commit(k, qi)
                        applied += 1
                    else:
                        self.pieces.pop()
                        break
        return applied

    # -------------------------------------------------------------- orient
    def _heading_between(self, a, b):
        """Direction the robot drives in when leaving a towards b (via the connector)."""
        if math.dist(a, b) < EPS:
            return None
        c = self._connector(a, b)
        return None if c is None else (c[1], c[2])

    def orient(self, deadline):
        """Give every single goal the heading that makes passing it cheapest: the
        way the robot arrives, or the way it leaves. Then the turn at the goal
        (none when it drives straight through, a full U-turn when it has to go
        back the way it came) is part of every cost comparison."""
        applied = 0
        for k in range(len(self.seq)):
            if time.monotonic() > deadline:
                break
            i, _ = self.seq[k]
            p = self.pieces[i]
            if not p.point:
                continue
            prev, nxt = self._at(k - 1), self._at(k + 1)
            xy = p.pts[0]
            cands = set()
            if prev is not None or self.start is not None:
                a = self.start if prev is None else self.pieces[prev[0]].state(prev[1])[2]
                hb = self._heading_between(a, xy)
                if hb:
                    cands.add(round(hb[1], 6))       # arrival heading
                elif prev is not None and self.pieces[prev[0]].state(prev[1])[3] is not None:
                    cands.add(round(self.pieces[prev[0]].state(prev[1])[3], 6))
            if nxt is not None:
                hb = self._heading_between(xy, self.pieces[nxt[0]].state(nxt[1])[0])
                if hb:
                    cands.add(round(hb[0], 6))       # departure heading
            base = self.T(prev, (i, False)) + self.T((i, False), nxt)
            best = None
            for h in cands:
                q = Piece([xy], p.kind)
                q.cov, q.swept, q.internal, q.h = p.cov, p.swept, p.internal, h
                self.pieces.append(q)
                qi = len(self.pieces) - 1
                c = self.T(prev, (qi, False)) + self.T((qi, False), nxt)
                self.pieces.pop()
                if c < base - 1e-6 and (best is None or c < best[0]):
                    best = (c, q)
            if best:
                self.pieces.append(best[1])
                self.seq[k] = [len(self.pieces) - 1, False]   # same cells: coverage counts unchanged
                applied += 1
        return applied

    # -------------------------------------------------------------- rotate
    def rotate(self, deadline, candidates=8):
        """Closed loops (wall loop, loops around obstacles, rings) can be entered
        anywhere. Start each loop at the point that makes getting there and
        leaving it cheapest, instead of where it happened to be generated."""
        applied = 0
        for k in range(len(self.seq)):
            if time.monotonic() > deadline:
                break
            i, rv = self.seq[k]
            p = self.pieces[i]
            if p.point or len(p.pts) < 4 or math.dist(p.pts[0], p.pts[-1]) > EPS:
                continue
            prev, nxt = self._at(k - 1), self._at(k + 1)
            a = self.start if prev is None else self.pieces[prev[0]].state(prev[1])[2]
            b = None if nxt is None else self.pieces[nxt[0]].state(nxt[1])[0]
            ring = np.asarray(p.pts[:-1], float)
            d = np.linalg.norm(ring - a, axis=1) + (np.linalg.norm(ring - b, axis=1) if b is not None else 0)
            base = p.internal + self.T(prev, (i, rv)) + self.T((i, rv), nxt)
            best = None
            for j in np.argsort(d)[:candidates]:
                if j == 0:
                    continue
                pts = np.vstack([ring[j:], ring[:j], ring[j:j + 1]]).tolist()
                q = Piece(pts, p.kind)
                q.cov, q.swept = p.cov, p.swept          # same loop, same coverage
                terms = costmod.route_terms(q.pts, self.s)
                q.internal = terms['drive_s'] + terms['turn_s'] + terms['reverse_s']
                self.pieces.append(q)
                qi = len(self.pieces) - 1
                for o in (False, True):
                    c = q.internal + self.T(prev, (qi, o)) + self.T((qi, o), nxt)
                    if c < base - 1e-6 and (best is None or c < best[0]):
                        best = (c, q, o)
                self.pieces.pop()
            if best:
                _, q, o = best
                self.pieces.append(q)
                self.seq[k] = [len(self.pieces) - 1, o]    # coverage counts unchanged (same cells)
                applied += 1
        return applied

    # ---------------------------------------------------------- straighten
    def _safe_dp(self, pts, tol):
        """Douglas-Peucker that only accepts a straight chord if the robot can drive it
        safely: fewer, longer straight lines, never closer to walls than allowed."""
        pts = np.asarray(pts, float)
        n = len(pts)
        keep = np.zeros(n, bool)
        keep[0] = keep[-1] = True
        stack = [(0, n - 1)]
        while stack:
            i, j = stack.pop()
            if j <= i + 1:
                continue
            a, b = pts[i], pts[j]
            ab = b - a
            L = math.hypot(*ab)
            seg = pts[i + 1:j] - a
            dev = np.abs(ab[0] * seg[:, 1] - ab[1] * seg[:, 0]) / L if L > 1e-12 else np.linalg.norm(seg, axis=1)
            m = int(np.argmax(dev))
            if dev[m] <= tol and L > 1e-12 and self.g.segment_safe(a, b, self.collision):
                continue
            keep[i + 1 + m] = True
            stack += [(i, i + 1 + m), (i + 1 + m, j)]
        return pts[keep].tolist()

    def straighten(self, deadline):
        """Replace a piece by a straighter version of itself (fewer, longer straight
        lines) whenever the cost function says that is cheaper overall. The straight
        line may leave the exact contour: brush over walls is free, while any floor it
        no longer reaches is charged as missed area."""
        applied = 0
        res = self.g.resolution
        tols = sorted({res, 2 * res, 0.5 * self.r, self.r, 2 * self.r, 4 * self.r})
        for k in range(len(self.seq)):
            if time.monotonic() > deadline:
                break
            i, rv = self.seq[k]
            p = self.pieces[i]
            if p.point or len(p.pts) < 3 or getattr(p, 'straight', False):
                continue
            best, tried = None, set()
            for tol in tols[::-1]:              # straightest first
                q = self._safe_dp(p.pts, tol)
                key = tuple(map(tuple, q))
                if len(q) >= len(p.pts) or key in tried:
                    continue
                tried.add(key)
                d, qi = self._replace_delta(k, self._measure(Piece(q, p.kind)))
                self.pieces.pop()
                if d < -1e-6 and (best is None or d < best[0]):
                    best = (d, q)
            if best:
                q = self._measure(Piece(best[1], p.kind))
                q.straight = True
                self.pieces.append(q)
                self._commit(k, len(self.pieces) - 1)
                applied += 1
            else:
                p.straight = True               # nothing better: do not try this piece again
        return applied

    # ---------------------------------------------------- block moves
    def block_moves(self, deadline):
        """Move runs of 2-3 consecutive pieces elsewhere (optionally reversed)."""
        applied = 0
        for L in (2, 3):
            k = 0
            while k + L <= len(self.seq):
                if time.monotonic() > deadline:
                    return applied
                first, last = tuple(self.seq[k]), tuple(self.seq[k + L - 1])
                prev, nxt = self._at(k - 1), self._at(k + L)
                removal = self.T(prev, nxt) - self.T(prev, first) - self.T(last, nxt)
                best = None
                for j in self._neighbour_positions(k):
                    if k - 1 <= j <= k + L - 1:
                        continue
                    a, b = self._at(j), self._at(j + 1)
                    for rev in (False, True):
                        f, l = ((last[0], not last[1]), (first[0], not first[1])) if rev else (first, last)
                        d = removal + self.T(a, f) + self.T(l, b) - self.T(a, b)
                        if d < -1e-6 and (best is None or d < best[0]):
                            best = (d, j, rev)
                if best:
                    _, j, rev = best
                    block = self.seq[k:k + L]
                    if rev:
                        block = [[i2, not r2] for i2, r2 in reversed(block)]
                    del self.seq[k:k + L]
                    pos = j + 1 if j < k else j + 1 - L
                    self.seq[pos:pos] = block
                    applied += 1
                    continue
                k += 1
        return applied

    def local_optimum_done(self, deadline, step):
        """Make sure the route is at a local optimum (e.g. after the last detours)."""
        self.local_optimum(deadline + 2.0, step)        # small grace period: never stop mid-repair
        return time.monotonic() < deadline

    def local_optimum(self, deadline, step):
        while time.monotonic() < deadline:
            n = (self.orient(deadline) + self.straighten(deadline) + self.improve_once(deadline)
                 + self.block_moves(deadline) + self.rotate(deadline) + self.orient(deadline)
                 + self.trim(deadline, step))
            if n == 0:
                break

    def perturb(self, rng):
        """Random 2-opt reversal plus a random relocation (keeps coverage unchanged)."""
        n = len(self.seq)
        if n < 4:
            return
        i, j = sorted(rng.choice(n, 2, replace=False))
        self.seq[i:j + 1] = [[a, not r] for a, r in reversed(self.seq[i:j + 1])]
        k = int(rng.integers(n))
        item = self.seq.pop(k)
        self.seq.insert(int(rng.integers(n)), item)

    # ------------------------------------------------------------- detours
    def detours(self, deadline, max_new=500):
        """Add a goal (a single pose, or a line along a missed strip) for missed
        cells when that lowers the total cost."""
        g, r = self.g, self.r
        reach_xy = g.world(np.argwhere(self.reach))
        added, declined = 0, set()
        while added < max_new and time.monotonic() < deadline:
            missed = np.flatnonzero(self.cov_ok & (self.count == 0))
            missed = [m for m in missed if m not in declined]
            if not missed:
                break
            m = missed[0]
            rc = divmod(int(m), g.shape[1])
            p = g.world(rc)
            cand = reach_xy[np.linalg.norm(reach_xy - p, axis=1) <= r + 1e-9]
            open_idx = np.array(missed)
            open_xy = g.world(np.stack(np.divmod(open_idx, g.shape[1]), axis=1))
            best = None
            for q in cand:
                near = np.flatnonzero(np.linalg.norm(open_xy - q, axis=1) <= r + 1e-9)
                if not len(near):
                    continue
                vis = near[visible_many(g, np.tile(q, (len(near), 1)), open_xy[near])]
                if m in set(open_idx[vis]) and (best is None or len(vis) > best[1]):
                    best = (q, len(vis))
            if best is None:        # no safe pose can see this cell: not reachable
                self.cov_ok[m] = False
                continue
            q = best[0].tolist()
            options = [Piece([q], 'gap')] + self._strip_lines(m, open_idx)
            choice = None
            ends = np.array([self.start] + [self.pieces[i].state(rv)[2] for i, rv in self.seq])
            for piece in options:
                idx = self._register(piece)
                self.count[piece.cov] -= 1           # evaluate before committing
                new = piece.cov[self.count[piece.cov] == 0]
                if not self.cov_ok[new].any():
                    continue
                gain = self._miss(new)
                over = self.k_over * (piece.swept - len(new) * self.cell)
                anchor = np.asarray(piece.pts[0])
                order = np.argsort(np.linalg.norm(ends - anchor, axis=1))[:self.K]
                for pos in order:       # insert after position pos-1 (pos=0 -> right after start)
                    a, b = self._at(pos - 1) if pos > 0 else None, self._at(pos)
                    for o in ((False,) if piece.point else (False, True)):
                        st = (idx, o)
                        d = self._slot_cost(a, st, b) - self.T(a, b)
                        delta = piece.internal + d + over - gain
                        if choice is None or delta < choice[0]:
                            choice = (delta, idx, pos, o, new)
            if choice is not None and choice[0] < 0:
                _, idx, pos, o, _ = choice
                self.count[self.pieces[idx].cov] += 1
                self.seq.insert(int(pos), [idx, o])
                added += 1
            else:
                declined.update(int(x) for x in (choice[4] if choice else [m]))
        return added

    def _strip_lines(self, m, open_idx, max_lines=3):
        """Line detours for the strip of missed cells around cell m: centre lines of
        the robot positions that can reach it (a row of point detours along a strip
        means a stop at every point; one line along it is usually far cheaper)."""
        from scipy.ndimage import binary_dilation, label
        from .spiral import SIMPLIFY, _safe_parts, _simplify, _trace_skeleton, skeletonize
        g, r = self.g, self.r
        H, W = g.shape
        rc = np.stack(np.divmod(open_idx, W), axis=1)
        r0, c0 = divmod(int(m), W)
        win = int(math.ceil(1.5 / g.resolution))
        near = (np.abs(rc[:, 0] - r0) <= win) & (np.abs(rc[:, 1] - c0) <= win)
        if near.sum() < 3:
            return []
        pad = int(math.ceil(r / g.resolution)) + 2
        R0, C0 = max(0, r0 - win - pad), max(0, c0 - win - pad)
        R1, C1 = min(H, r0 + win + pad + 1), min(W, c0 + win + pad + 1)
        miss = np.zeros((R1 - R0, C1 - C0), bool)
        sel = rc[near]
        miss[sel[:, 0] - R0, sel[:, 1] - C0] = True
        lab, _ = label(miss, structure=np.ones((3, 3)))
        comp = lab == lab[r0 - R0, c0 - C0]
        if comp.sum() < 3:
            return []
        n = int(math.ceil(r / g.resolution))
        yy, xx = np.mgrid[-n:n + 1, -n:n + 1]
        disk = np.hypot(xx, yy) * g.resolution <= r
        pos = self.reach[R0:R1, C0:C1] & binary_dilation(comp, disk)
        out = []
        for line in _trace_skeleton(skeletonize(pos)):
            if len(line) < 3:
                continue
            pts = g.world(line + [R0, C0])
            for part in _safe_parts(g, [q.tolist() for q in _simplify(pts, SIMPLIFY * g.resolution)],
                                    self.collision):
                if len(part) > 1 and sum(math.dist(a, b) for a, b in zip(part, part[1:])) > g.resolution:
                    out.append(Piece(part, 'fill'))
        out.sort(key=lambda p: -sum(math.dist(a, b) for a, b in zip(p.pts, p.pts[1:])))
        return out[:max_lines]

    # ---------------------------------------------------------------- run
    def run(self, budget_s, seed=0):
        deadline = time.monotonic() + budget_s
        step = 0.5 * float(self.s['geometry']['coverage_disk_radius_m'])
        for _ in range(20):                      # detours + local moves until stable
            n = self.detours(deadline)
            before = self.total()[0]
            self.local_optimum(deadline, step)
            if n == 0 and self.total()[0] >= before - 1e-6 or time.monotonic() > deadline:
                break
        # use the remaining time: perturb, re-optimise, keep only improvements
        if not self.local_optimum_done(deadline, step):
            return self
        rng = np.random.default_rng(seed)
        best_cost = self.total()[0]
        while time.monotonic() < deadline:
            saved = ([list(x) for x in self.seq], self.count.copy(), len(self.pieces), self.cov_ok.copy())
            self.perturb(rng)
            self.local_optimum(deadline, step)
            if self.detours(deadline):
                self.local_optimum(deadline, step)     # new goals get ordered too
            c = self.total()[0]
            if c < best_cost - 1e-6:
                best_cost = c
            else:
                self.seq, self.count, self.cov_ok = saved[0], saved[1], saved[3]
                del self.pieces[saved[2]:]
        return self

    def targets(self, max_seg):
        out = []
        for i, rv in self.seq:
            out.extend(self.pieces[i].targets(rv, max_seg))
        return out
