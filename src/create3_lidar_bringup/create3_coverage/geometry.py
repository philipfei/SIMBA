"""Conservative occupancy-grid geometry; all public positions are in map metres."""
from dataclasses import dataclass
from collections import deque
import hashlib
import heapq
import json
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def validate_polygon(points):
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 2 or len(p) < 3 or not np.isfinite(p).all():
        raise ValueError('A region needs at least three finite map-frame points')
    if np.allclose(p[0], p[-1]):
        p = p[:-1]
    if len(p) < 3 or len(np.unique(p, axis=0)) != len(p):
        raise ValueError('Repeated polygon vertices')
    def orient(a, b, c):
        return float(np.cross(b-a, c-a))
    def intersect(a, b, c, d):
        if np.any(np.maximum(np.minimum(a,b), np.minimum(c,d)) >
                  np.minimum(np.maximum(a,b), np.maximum(c,d)) + 1e-10):
            return False
        return orient(a,b,c)*orient(a,b,d) <= 1e-12 and orient(c,d,a)*orient(c,d,b) <= 1e-12
    for i in range(len(p)):
        if np.linalg.norm(p[i]-p[i-1]) < 1e-8:
            raise ValueError('Zero length polygon edge')
        for j in range(i+1, len(p)):
            if j == i+1 or (i == 0 and j == len(p)-1):
                continue
            if intersect(p[i], p[(i+1)%len(p)], p[j], p[(j+1)%len(p)]):
                raise ValueError('Self-intersecting region')
    if abs(np.sum(p[:,0]*np.roll(p[:,1],-1)-p[:,1]*np.roll(p[:,0],-1))) < 1e-8:
        raise ValueError('Empty polygon')
    return p.tolist()


def inside(points, polygon):
    pts = np.asarray(points); x, y = pts[...,0], pts[...,1]
    result = np.zeros(x.shape, bool)
    for a, b in zip(polygon, polygon[1:]+polygon[:1]):
        if a[1] != b[1]:
            result ^= ((a[1] > y) != (b[1] > y)) & (x < (b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0])
    return result


@dataclass
class Grid:
    cells: np.ndarray  # OccupancyGrid order: row zero is map-local bottom
    resolution: float
    origin: tuple
    identity: str

    @classmethod
    def load(cls, filename):
        filename = Path(filename).resolve()
        meta = yaml.safe_load(filename.read_text())
        if meta.get('mode', 'trinary') != 'trinary':
            raise ValueError('Only explicit trinary maps are supported')
        image = (filename.parent / meta['image']).resolve()
        raw = np.asarray(Image.open(image).convert('L'))
        prob = raw.astype(float)/255 if meta.get('negate', 0) else (255-raw.astype(float))/255
        cells = np.full(raw.shape, -1, np.int8)
        cells[prob < float(meta['free_thresh'])] = 0
        cells[prob > float(meta['occupied_thresh'])] = 100
        cells = np.flipud(cells).copy()
        origin = tuple(float(v) for v in meta['origin'])
        res = float(meta['resolution'])
        if res <= 0 or len(origin) != 3 or not np.isfinite([res, *origin]).all():
            raise ValueError('Invalid map metadata')
        # Canonical geometry identity also agrees with a map_server OccupancyGrid.
        return cls.from_cells(cells, res, origin)

    @classmethod
    def from_cells(cls, cells, resolution, origin):
        a = np.asarray(cells, np.int8)
        header = json.dumps([list(a.shape), round(float(resolution), 8),
                             [round(float(v), 8) for v in origin]], separators=(',', ':'))
        ident = hashlib.sha256(header.encode()+a.tobytes()).hexdigest()
        return cls(a, float(resolution), tuple(origin), ident)

    @property
    def free(self):
        return self.cells == 0

    def local(self, xy):
        v = np.asarray(xy, float) - self.origin[:2]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return v @ np.array([[c, -s], [s, c]])

    def world(self, rc):
        rc = np.asarray(rc)
        xy = (rc[..., ::-1] + .5)*self.resolution
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return xy @ np.array([[c,s],[-s,c]]) + self.origin[:2]

    def cell(self, xy):
        return tuple(np.floor(self.local(xy)[::-1]/self.resolution).astype(int))

    def valid(self, rc):
        return 0 <= rc[0] < self.cells.shape[0] and 0 <= rc[1] < self.cells.shape[1]

    def polygon_mask(self, polygon):
        polygon = validate_polygon(polygon)
        rc = np.indices(self.cells.shape).transpose(1,2,0)
        return inside(self.world(rc), polygon)

    def safe_centres(self, radius):
        if radius <= 0:
            raise ValueError('Collision radius must be positive')
        n = math.ceil(radius/self.resolution + .5)
        padded = np.pad(self.free, n, constant_values=False)
        safe = self.free.copy(); h,w = safe.shape
        for dy in range(-n,n+1):
            for dx in range(-n,n+1):
                d = math.hypot(max(abs(dx)-.5,0), max(abs(dy)-.5,0))*self.resolution
                if d <= radius + 1e-10:
                    safe &= padded[n+dy:n+dy+h,n+dx:n+dx+w]
        return safe

    def segment_safe(self, a, b, radius):
        """Exact capsule vs closed blocked cell rectangles (no diagonal shortcuts)."""
        a,b = self.local(a), self.local(b); r=self.resolution
        lo,hi = np.minimum(a,b)-radius, np.maximum(a,b)+radius
        extent=np.array(self.cells.shape[::-1])*r
        if np.any(lo <= 0) or np.any(hi >= extent):
            return False
        x0,y0=np.floor(lo/r).astype(int); x1,y1=np.floor(hi/r).astype(int)
        blocked=np.argwhere(self.cells[y0:y1+1,x0:x1+1] != 0)
        if not len(blocked):
            return True
        mins=(blocked[:,::-1]+[x0,y0])*r; maxs=mins+r
        v=b-a; vv=float(v@v)
        # Slab test: the centre segment intersects a blocked rectangle.
        tlo=np.zeros(len(mins)); thi=np.ones(len(mins))
        for k in range(2):
            if abs(v[k]) < 1e-14:
                ok=(a[k]>=mins[:,k]) & (a[k]<=maxs[:,k])
                tlo=np.where(ok,tlo,2.)
            else:
                u=(mins[:,k]-a[k])/v[k]; z=(maxs[:,k]-a[k])/v[k]
                tlo=np.maximum(tlo,np.minimum(u,z)); thi=np.minimum(thi,np.maximum(u,z))
        if np.any(tlo <= thi):
            return False
        d2=np.full(len(mins), np.inf)
        for p in (a,b):
            d=np.maximum(np.maximum(mins-p,p-maxs),0)
            d2=np.minimum(d2,np.sum(d*d,axis=1))
        for dx,dy in ((0,0),(0,1),(1,0),(1,1)):
            corners=mins+np.array([dx,dy])*r
            t=np.clip((corners-a)@v/vv,0,1) if vv else np.zeros(len(mins))
            d=corners-(a+t[:,None]*v)
            d2=np.minimum(d2,np.sum(d*d,axis=1))
        return bool(np.all(d2 > radius*radius+1e-12))

    def visible(self, a, b):
        # Tiny positive radius rejects passing through a blocked grid corner.
        return self.segment_safe(a,b,1e-9)

    def reachable(self, start, radius):
        safe=self.safe_centres(radius); root=self.cell(start)
        if not self.valid(root) or not safe[root] or not self.segment_safe(start,self.world(root),radius):
            raise ValueError('Localized start is outside the collision-safe centre region')
        seen=np.zeros_like(safe); seen[root]=True; q=deque([root])
        while q:
            y,x=q.popleft()
            for yy,xx in ((y+1,x),(y-1,x),(y,x+1),(y,x-1)):
                p=(yy,xx)
                if self.valid(p) and safe[p] and not seen[p] and self.segment_safe(self.world((y,x)),self.world(p),radius):
                    seen[p]=True; q.append(p)
        return seen

    def edge_safe(self, a, b, radius):
        cache=getattr(self,'_edge_cache',None)
        if cache is None:self._edge_cache={};cache=self._edge_cache
        key=(radius,*sorted((tuple(a),tuple(b))))
        if key not in cache:cache[key]=self.segment_safe(self.world(a),self.world(b),radius)
        return cache[key]

    def astar(self, start, goal, safe, radius):
        a,b=self.cell(start),self.cell(goal)
        if not self.valid(a) or not self.valid(b) or not safe[a] or not safe[b]:
            return []
        if not self.segment_safe(start,self.world(a),radius) or not self.segment_safe(self.world(b),goal,radius):
            return []
        heap=[(0,0,a)]; prev={a:None}; costs={a:0}
        while heap:
            _,cost,p=heapq.heappop(heap)
            if cost != costs[p]: continue
            if p==b:
                route=[]
                while p is not None: route.append(self.world(p).tolist()); p=prev[p]
                route.reverse()
                return [list(start),*route,list(goal)]
            y,x=p
            for n in ((y+1,x),(y-1,x),(y,x+1),(y,x-1)):
                nc=cost+1
                if self.valid(n) and safe[n] and nc<costs.get(n,math.inf) and self.edge_safe(p,n,radius):
                    costs[n]=nc;prev[n]=p
                    heapq.heappush(heap,(nc+abs(n[0]-b[0])+abs(n[1]-b[1]),nc,n))
        return []

    def disk(self, xy, radius):
        result=np.zeros(self.cells.shape,bool); cy,cx=self.cell(xy)
        n=math.ceil(radius/self.resolution)+1
        for y in range(max(0,cy-n),min(self.cells.shape[0],cy+n+1)):
            for x in range(max(0,cx-n),min(self.cells.shape[1],cx+n+1)):
                p=self.world((y,x))
                if self.free[y,x] and np.linalg.norm(p-xy)<=radius+1e-10 and self.visible(xy,p):
                    result[y,x]=True
        return result

    def coverable(self, reachable, polygon, coverage_radius):
        roi=self.polygon_mask(polygon)&self.free
        result=np.zeros_like(roi)
        # Only inspect still-uncredited target cells and nearby reachable centres.
        for y,x in np.argwhere(roi):
            n=math.ceil(coverage_radius/self.resolution)
            for yy,xx in np.argwhere(reachable[max(0,y-n):y+n+1,max(0,x-n):x+n+1]):
                p=(yy+max(0,y-n),xx+max(0,x-n))
                if math.hypot(p[0]-y,p[1]-x)*self.resolution<=coverage_radius+1e-10 and self.visible(self.world(p),self.world((y,x))):
                    result[y,x]=True;break
        return result
