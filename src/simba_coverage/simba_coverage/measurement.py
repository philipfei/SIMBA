"""Actual-pose virtual coverage; never accepts a planned path as evidence."""
import math
import numpy as np
from .geometry import wrap


class Meter:
    def __init__(self, grid, denominator, radius=.25, max_gap=.3, linear=.12, angular=.40, translation_slack=.05, rotation_slack=5.):
        self.translation_slack=translation_slack;self.rotation_slack=math.radians(rotation_slack)
        self.grid=grid;self.denominator=np.asarray(denominator,bool).copy()
        self.denominator.setflags(write=False)
        self.covered=np.zeros_like(self.denominator)
        self.radius=radius;self.max_gap=max_gap;self.linear=linear;self.angular=angular
        self.previous=None;self.trajectory=[];self.trajectory_edges=[];self.trajectory_segment_start=0

    def sample(self, pose, stamp, trusted=True, active=True):
        if not trusted or not active:
            self.previous=None;return 0
        if not np.isfinite([*pose,stamp]).all():self.previous=None;return 0
        points=[pose[:2]]
        if self.previous is None or stamp-self.previous[1]>self.max_gap:
            self.trajectory_segment_start=len(self.trajectory)
        if self.previous:
            old,t=self.previous;dt=stamp-t
            if dt<=0:
                self.previous=None;return 0
            displacement=math.dist(old[:2],pose[:2]);turn=abs(wrap(pose[2]-old[2]))
            if displacement>self.linear*dt+self.translation_slack or turn>self.angular*dt+self.rotation_slack:
                self.previous=None;return 0
            if dt<=self.max_gap:
                self.trajectory_edges.append((list(old[:2]),list(pose[:2])))
                count=max(1,math.ceil(displacement/(self.grid.resolution/2)))
                points=[np.asarray(old[:2])+(np.asarray(pose[:2])-old[:2])*i/count for i in range(1,count+1)]
        before=int(self.covered.sum())
        for xy in points:self.covered |= self.grid.disk(xy,self.radius)&self.denominator
        self.previous=(list(pose),stamp);self.trajectory.append([stamp,*pose])
        return int(self.covered.sum())-before

    def report(self):
        cell=self.grid.resolution**2;total=int(self.denominator.sum());covered=int(self.covered.sum())
        return {'known_free_m2':int(self.grid.free.sum())*cell,'coverable_m2':total*cell,
                'covered_m2':covered*cell,'remaining_m2':(total-covered)*cell,
                'fraction':covered/total if total else 0.}
