"""Occupancy grid built from a ROS map (.pgm + .yaml).

Provides exactly the interface that planning.py / planner.py expect:
cells, free, resolution, origin, world(), cell(), local(), valid(),
segment_safe(), astar(), disk(), polygon_mask().

Conventions (same as nav_msgs/OccupancyGrid):
  cells[row, col], row 0 is the bottom of the map (at the origin),
  values 0 = free, 100 = occupied, -1 = unknown.
  rc tuples are (row, col); world points are (x, y) in metres in the map frame.
"""
import heapq
import math
import os

import numpy as np
import yaml
from PIL import Image
from matplotlib.path import Path
from scipy.ndimage import distance_transform_edt, label


class Grid:
    def __init__(self, cells, resolution, origin=(0.0, 0.0, 0.0)):
        self.cells = np.asarray(cells, dtype=np.int16)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]), float(origin[2] if len(origin) > 2 else 0.0))
        self.free = self.cells == 0
        h, w = self.cells.shape
        self.shape = (h, w)
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        self._R = np.array([[c, -s], [s, c]])
        # Clearance: metric distance from a free cell centre to the nearest
        # non-free cell (occupied, unknown or outside the map), measured to that
        # cell's edge. Everything that is not known-free counts as an obstacle.
        padded = np.pad(self.free, 1, constant_values=False)
        edt = distance_transform_edt(padded)[1:-1, 1:-1]
        self.clearance = np.where(self.free, edt * self.resolution - self.resolution / 2, -1.0)
        self._clear_list = self.clearance.tolist()  # fast scalar lookups
        self._astar_cache = {}

    # ------------------------------------------------------------------ frames
    def world(self, rc):
        rc = np.asarray(rc, dtype=float)
        local = np.stack([(rc[..., 1] + 0.5) * self.resolution, (rc[..., 0] + 0.5) * self.resolution], axis=-1)
        return local @ self._R.T + np.array(self.origin[:2])

    def local(self, xy):
        xy = np.asarray(xy, dtype=float)
        return (xy - np.array(self.origin[:2])) @ self._R

    def cell(self, xy):
        lx, ly = self.local(np.asarray(xy, dtype=float).reshape(2))
        return (int(math.floor(ly / self.resolution)), int(math.floor(lx / self.resolution)))

    def valid(self, rc):
        return 0 <= rc[0] < self.shape[0] and 0 <= rc[1] < self.shape[1]

    # ---------------------------------------------------------------- geometry
    def _cell_fast(self, x, y):
        dx, dy = x - self.origin[0], y - self.origin[1]
        c, s = self._R[0, 0], self._R[1, 0]
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        return int(math.floor(ly / self.resolution)), int(math.floor(lx / self.resolution))

    def segment_safe(self, a, b, radius):
        """True if every grid cell the segment passes through has clearance >= radius.

        Samples every quarter cell, so the check is exact up to grid resolution.
        """
        ax, ay = float(a[0]), float(a[1])
        bx, by = float(b[0]), float(b[1])
        d = math.hypot(bx - ax, by - ay)
        n = max(1, int(math.ceil(d / (self.resolution / 4))))
        h, w = self.shape
        clear = self._clear_list
        need = radius - 1e-9
        for i in range(n + 1):
            t = i / n
            r, c = self._cell_fast(ax + (bx - ax) * t, ay + (by - ay) * t)
            if r < 0 or c < 0 or r >= h or c >= w or clear[r][c] < need:
                return False
        return True

    def disk(self, xy, radius):
        rc = np.indices(self.shape).reshape(2, -1).T
        d = np.linalg.norm(self.world(rc) - np.asarray(xy, float), axis=1)
        return (d <= radius + 1e-9).reshape(self.shape)

    def polygon_mask(self, polygon):
        if polygon is None or len(polygon) < 3:
            return np.ones(self.shape, bool)
        rc = np.indices(self.shape).reshape(2, -1).T
        return Path(np.asarray(polygon, float)).contains_points(self.world(rc)).reshape(self.shape)

    def passable(self, radius):
        return self.clearance >= radius - 1e-9

    def reachable_from(self, xy, radius):
        """Connected component of collision-free cells containing (or nearest to) xy."""
        ok = self.passable(radius)
        if not ok.any():
            return ok, None
        rc = self.cell(xy)
        if not (self.valid(rc) and ok[rc]):
            cand = np.argwhere(ok)
            d = np.linalg.norm(self.world(cand) - np.asarray(xy, float), axis=1)
            rc = tuple(int(v) for v in cand[np.argmin(d)])
        # 4-connected, matching A* (which never squeezes diagonally past two blocked
        # cells): every cell counted as reachable can really be driven to.
        labels, _ = label(ok)
        return labels == labels[rc], self.world(rc).tolist()

    # -------------------------------------------------------------------- A*
    def astar(self, a, b, reachable, radius):
        """8-connected A* over reachable cells with enough clearance.

        Returns a list of [x, y] points from a to b, or [] if there is no path.
        """
        key = (id(reachable), round(radius, 6))
        if key not in self._astar_cache:
            if len(self._astar_cache) > 8:
                self._astar_cache.clear()
            self._astar_cache[key] = (reachable & self.passable(radius)).ravel()
        ok = self._astar_cache[key]
        h, w = self.shape
        s, g = self.cell(a), self.cell(b)
        if not (self.valid(s) and self.valid(g)):
            return []
        si, gi = s[0] * w + s[1], g[0] * w + g[1]
        if not ok[gi]:
            return []
        if si == gi:
            return [list(map(float, a)), list(map(float, b))]
        gr, gc = g
        sq2 = math.sqrt(2)
        moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, sq2), (-1, 1, sq2), (1, -1, sq2), (1, 1, sq2)]
        best = {si: 0.0}
        parent = {si: -1}
        heap = [(0.0, 0.0, si)]
        if not ok[si]:
            # The start point lies a hair inside a cell with slightly too little
            # clearance (e.g. the end of a contour). Step out to the nearest
            # valid cells within two cells instead of giving up.
            sr, sc = s
            for dr in range(-2, 3):
                for dc in range(-2, 3):
                    nr, nc = sr + dr, sc + dc
                    if 0 <= nr < h and 0 <= nc < w and ok[nr * w + nc]:
                        ni = nr * w + nc
                        step = math.hypot(dr, dc)
                        best[ni] = step
                        parent[ni] = si
                        ddr, ddc = abs(nr - gr), abs(nc - gc)
                        heapq.heappush(heap, (step + max(ddr, ddc) + (sq2 - 1) * min(ddr, ddc), step, ni))
        closed = set()
        while heap:
            _, cost, cur = heapq.heappop(heap)
            if cur in closed:
                continue
            if cur == gi:
                break
            closed.add(cur)
            if cur == si and not ok[si]:
                continue          # only the valid cells seeded above expand from here
            r, c = divmod(cur, w)
            for dr, dc, step in moves:
                nr, nc = r + dr, c + dc
                if nr < 0 or nc < 0 or nr >= h or nc >= w:
                    continue
                ni = nr * w + nc
                if not ok[ni] or ni in closed:
                    continue
                if dr and dc and not (ok[r * w + nc] and ok[nr * w + c]):
                    continue  # no corner cutting
                nc_cost = cost + step
                if nc_cost < best.get(ni, math.inf):
                    best[ni] = nc_cost
                    parent[ni] = cur
                    ddr, ddc = abs(nr - gr), abs(nc - gc)
                    hcost = max(ddr, ddc) + (sq2 - 1) * min(ddr, ddc)
                    heapq.heappush(heap, (nc_cost + hcost, nc_cost, ni))
        if gi not in parent:
            return []
        chain = []
        cur = parent[gi]
        while cur not in (-1, si):
            chain.append(divmod(cur, w))
            cur = parent[cur]
        chain.reverse()
        mid = self.world(np.array(chain)).tolist() if chain else []
        return [list(map(float, a))] + mid + [list(map(float, b))]


def load_ros_map(yaml_path):
    """Load a map_server / slam_toolbox map (.yaml + image) into a Grid."""
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    image_path = meta['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image_path)
    img = Image.open(image_path)
    has_alpha = img.mode in ('RGBA', 'LA')
    pixels = np.asarray(img.convert('L'), dtype=float)
    alpha = np.asarray(img.getchannel('A')) if has_alpha else None
    mode = meta.get('mode', 'trinary')
    negate = int(meta.get('negate', 0))
    occ_t = float(meta.get('occupied_thresh', 0.65))
    free_t = float(meta.get('free_thresh', 0.196))
    p = pixels / 255.0 if negate else (255.0 - pixels) / 255.0
    if mode == 'raw':
        cells = np.where(pixels > 100, -1, pixels).astype(np.int16)
    else:
        cells = np.full(pixels.shape, -1, np.int16)
        cells[p > occ_t] = 100
        cells[p < free_t] = 0
        if mode == 'scale':
            mid = (p >= free_t) & (p <= occ_t)
            cells[mid] = np.round(99 * (p[mid] - free_t) / (occ_t - free_t)).astype(np.int16)
    if alpha is not None:
        cells[alpha == 0] = -1
    cells = np.flipud(cells)  # image row 0 is the top; grid row 0 is at the origin
    origin = meta.get('origin', [0, 0, 0])
    return Grid(cells, meta['resolution'], origin)
