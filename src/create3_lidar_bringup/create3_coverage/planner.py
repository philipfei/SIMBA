"""Small grid sweep decomposition; no ROS or third-party coverage algorithm."""
from dataclasses import dataclass
import hashlib
import math
import numpy as np


@dataclass
class Target:
    start: list
    end: list
    key: str


def target(a,b):
    endpoints=sorted(tuple(round(float(v),4) for v in p) for p in (a,b))
    return Target(list(a),list(b),hashlib.sha256(repr(endpoints).encode()).hexdigest()[:20])


def decompose(mask):
    """Continue a cell only on a one-to-one interval overlap; split/merge otherwise."""
    active=[]; cells=[]
    for y,row in enumerate(mask):
        xs=np.flatnonzero(row)
        runs=[(int(a[0]),int(a[-1])) for a in np.split(xs,np.flatnonzero(np.diff(xs)>1)+1) if len(a)]
        parents=[[i for i,old in enumerate(active) if max(a,old[-1][1])<=min(b,old[-1][2])] for a,b in runs]
        next_active=[];used=set()
        for (a,b),ps in zip(runs,parents):
            if len(ps)==1 and sum(ps[0] in q for q in parents)==1:
                i=ps[0];c=active[i];c.append((y,a,b));used.add(i)
            else: c=[(y,a,b)]
            next_active.append(c)
        cells.extend(c for i,c in enumerate(active) if i not in used)
        active=next_active
    return cells+active


def sweeps(grid, reachable, denominator, spacing, radius, max_segment=2.):
    """Compatibility entry point: one global metric phase, never per-cell offsets."""
    from .planning import stripes, split_path
    from .settings import load_settings
    if spacing<=0 or max_segment<=0:raise ValueError('Invalid stripe spacing/length')
    ts=stripes(grid,reachable,denominator,math.degrees(grid.origin[2]),0.,spacing,load_settings().collision,radius)
    return [part for t in ts for part in split_path([t.start,t.end],max_segment,'sweep')]


def ordered(grid, candidates, start, reachable, collision_radius):
    remaining=list(candidates);out=[];xy=list(start);connectors=[]
    while remaining:
        i,reverse=min(((i,r) for i in range(len(remaining)) for r in (False,True)),
                      key=lambda v:np.linalg.norm(np.array(remaining[v[0]].end if v[1] else remaining[v[0]].start)-xy))
        t=remaining.pop(i)
        if reverse:t=target(t.end,t.start)
        if not grid.segment_safe(t.start,t.end,collision_radius):
            # An interval whose continuous swept footprint is unsafe is never executed.
            continue
        path=grid.astar(xy,t.start,reachable,collision_radius)
        if not path:continue
        connectors.append(path);out.append(t);xy=t.end
    return out,connectors


def gap_targets(grid, remaining, reachable, coverage_radius, excluded, limit=200):
    """Greedy residual disk gain; retains a fixed task denominator and exclusion set."""
    work=remaining.copy();out=[]
    while work.any() and len(out)<limit:
        p=grid.world(np.argwhere(work)[0]);near=[]
        for rc in np.argwhere(reachable):
            xy=grid.world(rc)
            if np.linalg.norm(xy-p)<=coverage_radius+1e-9:
                t=target(xy,xy)
                if not excluded(t):near.append((xy,t))
        best=None
        for xy,t in near:
            disk=grid.disk(xy,coverage_radius)&work;gain=int(disk.sum())
            if gain and (best is None or gain>best[0]):best=(gain,t,disk)
        if best is None:
            work[grid.cell(p)]=False
        else:
            out.append(best[1]);work &= ~best[2]
    return out
