"""Cost function used to compare and choose coverage paths.

Everything is expressed in seconds, so the terms can be added up:

  drive      path length / speed
  bending    every bend costs `bend_s_per_rad` per radian of heading change, so a
             truly straight line is the cheapest way to drive
  turning    see vertex_cost(): a curve only costs extra where it is tighter than
             speed / turn rate; a real corner is a stop + rotation in place +
             `stop_penalty_s`. Bends up to `smooth_turn_deg` never stop.
  reversing  a differential-drive robot never has to drive backwards on these
             paths; the place where a controller would back up instead of
             turning is a hairpin (direction change >= `reverse_angle_deg`).
             Every hairpin costs `reverse_penalty_s` on top of its rotation.
  overlap    FLOOR area swept more than once x `overlap_weight` x the time it
             takes to sweep one square metre. The brush passing over walls or
             obstacles costs nothing, so a straight line may run past a bumpy
             wall instead of following every bump.
  missed     reachable area left uncovered x `missed_weight` x the same time
"""
import math

import numpy as np


def area_time(settings):
    """Seconds needed to sweep one square metre (speed x brush width)."""
    v = float(settings['motion']['linear_m_s'])
    r = float(settings['geometry']['coverage_disk_radius_m'])
    return 1.0 / (v * 2.0 * r)


def vertex_cost(dh, radius, settings):
    """Extra seconds for heading changes dh (rad) at path vertices, vectorised.
    Returns (seconds, is_stop).

    radius = the largest arc that fits the corner (from the neighbouring segment
    lengths; 0 = a real corner). A differential-drive robot drives an arc of
    radius R at speed min(v, w*R), so compared with driving straight an arc
    costs dh * max(0, 1/w - R/v): only arcs tighter than v/w (0.375 m by
    default) slow it down. A real corner (R = 0) is a stop, a rotation in place
    (dh / w) and `stop_penalty_s` to brake and accelerate. A curve therefore
    costs the same however finely it is drawn.
      - bends up to `smooth_turn_deg` never stop; every bend that is not a
        stop also costs `bend_s_per_rad` (preference for straight lines)
      - hairpins (>= `reverse_angle_deg`) always pivot and cost `reverse_penalty_s`"""
    c = settings['cost']
    v = float(settings['motion']['linear_m_s'])
    w = float(settings['motion']['angular_rad_s'])
    smooth = math.radians(float(c['smooth_turn_deg']))
    hairpin = math.radians(float(c['reverse_angle_deg']))
    dh = np.abs(np.asarray(dh, float))
    R = np.where(dh >= hairpin, 0.0, np.asarray(radius, float))
    slow = np.clip(1.0 - R * w / v, 0.0, 1.0)            # 1 = pivot in place, 0 = no slow-down
    stop = (dh > smooth) & (slow > 0.5)                    # a real corner: brake, rotate, accelerate
    out = np.where(stop, 0.0, dh * float(c.get('bend_s_per_rad', 0.0)))   # straightness preference (corners
                                                                             # already pay rotation + stop)
    out = out + dh * np.maximum(0.0, 1.0 / w - R / v)      # slower on arcs tighter than v / w
    out = out + np.where(stop, float(c['stop_penalty_s']) * slow, 0.0)
    return out + np.where(dh >= hairpin, float(c['reverse_penalty_s']), 0.0), stop


CORNER_CUT_M = 0.05   # a rounded corner may pass at most this far from the planned vertex


def corner_radius(seg_in, seg_out, dh):
    """Radius of the arc the robot can drive through a vertex: it must fit in half
    of each neighbouring segment and stay within CORNER_CUT_M of the vertex (so a
    sharp corner stays a corner, while the vertices of a drawn curve are smooth)."""
    dh = np.minimum(np.asarray(dh, float), math.pi - 1e-6)
    t = np.tan(dh / 2)
    L = 0.5 * np.minimum(seg_in, seg_out)
    fit = np.where(t > 1e-9, L / np.maximum(t, 1e-9), np.inf)
    cut = 1.0 / np.cos(dh / 2) - 1.0                  # vertex-to-arc distance per metre of radius
    return np.minimum(fit, np.where(cut > 1e-12, CORNER_CUT_M / np.maximum(cut, 1e-12), np.inf))


def route_terms(points, settings):
    """Length and turning terms of one continuous route (list of [x, y])."""
    c = settings['cost']
    v = float(settings['motion']['linear_m_s'])
    hairpin = math.radians(float(c['reverse_angle_deg']))
    bend = float(c.get('bend_s_per_rad', 0.0))
    a = np.asarray(points, float)
    zero = {'length_m': 0.0, 'drive_s': 0.0, 'turn_s': 0.0, 'bend_s': 0.0, 'stops': 0, 'hairpins': 0,
            'reverse_s': 0.0}
    if len(a) < 2:
        return zero
    d = np.diff(a, axis=0)
    seg = np.hypot(d[:, 0], d[:, 1])
    length = float(seg.sum())
    keep = seg > 1e-6
    d, seg = d[keep], seg[keep]
    turn_s, bend_s, stops, hairpins, rev = 0.0, 0.0, 0, 0, 0.0
    if len(d) > 1:
        h = np.arctan2(d[:, 1], d[:, 0])
        dh = np.abs(np.arctan2(np.sin(np.diff(h)), np.cos(np.diff(h))))
        cost, stop = vertex_cost(dh, corner_radius(seg[:-1], seg[1:], dh), settings)
        hairpins = int((dh >= hairpin).sum())
        rev = hairpins * float(c['reverse_penalty_s'])
        stops = int(stop.sum())
        bend_s = float(dh[~stop].sum() * bend)
        turn_s = float(cost.sum()) - rev
    return {'length_m': length, 'drive_s': length / v, 'turn_s': turn_s, 'bend_s': bend_s, 'stops': stops,
            'hairpins': hairpins, 'reverse_s': rev}


def piece_cells(grid, region, pts, radius):
    """Flat indices of the region cells one polyline's brush covers (cropped, so it is fast)."""
    from .planning import poly_target, stroke_union
    pts = np.asarray(pts, float)
    lo = pts.min(axis=0) - radius - grid.resolution
    hi = pts.max(axis=0) + radius + grid.resolution
    rcs = np.array([grid.cell(lo), grid.cell(hi), grid.cell([lo[0], hi[1]]), grid.cell([hi[0], lo[1]])])
    r0, c0 = np.clip(rcs.min(axis=0), 0, None)
    r1, c1 = rcs.max(axis=0) + 1
    crop = np.zeros_like(region)
    crop[r0:r1, c0:c1] = region[r0:r1, c0:c1]
    t = poly_target([list(p) for p in pts] if len(pts) > 1 else [list(pts[0]), list(pts[0])], 'x')
    if len(pts) == 1:
        t.points = [list(pts[0]), list(pts[0])]
    return np.flatnonzero(stroke_union(grid, crop, [t], radius))


def swept_floor(grid, region, targets, radius):
    """Floor area (m2) the brush passes over, counting every target separately,
    so floor covered by two targets counts twice. Walls/obstacles never count."""
    from .planning import points_of
    chains, prev = [], None          # rejoin a line that was split into several Nav2 goals
    for t in targets:
        pts = [list(map(float, p)) for p in points_of(t)]
        if prev is not None and getattr(t, 'kind', None) == getattr(prev, 'kind', None) \
                and math.dist(chains[-1][-1], pts[0]) < 1e-6 and len(pts) > 1 and len(chains[-1]) > 1 \
                and math.dist(pts[0], pts[-1]) > 1e-6:
            chains[-1].extend(pts[1:])
        else:
            chains.append(pts)
        prev = t
    n = sum(len(piece_cells(grid, region, c, radius)) for c in chains)
    return n * grid.resolution ** 2


def evaluate(route_points, targets, covered_m2, coverable_missed_m2, settings, swept_m2, edge_missed_m2=0.0):
    """Total cost (s) of a path plus its breakdown. `swept_m2` = swept_floor(...);
    `edge_missed_m2` = the part of the missed area that lies within `edge_band_m`
    of a wall/obstacle (charged with `edge_missed_weight` instead)."""
    c = settings['cost']
    terms = route_terms(route_points, settings)
    at = area_time(settings)
    overlap_m2 = max(0.0, swept_m2 - covered_m2)
    overlap_s = float(c['overlap_weight']) * overlap_m2 * at
    edge_w = float(c.get('edge_missed_weight', c['missed_weight']))
    missed_s = (float(c['missed_weight']) * (coverable_missed_m2 - edge_missed_m2) + edge_w * edge_missed_m2) * at
    total = terms['drive_s'] + terms['turn_s'] + terms['reverse_s'] + overlap_s + missed_s
    return dict(terms, overlap_m2=overlap_m2, overlap_s=overlap_s, missed_m2=coverable_missed_m2,
                edge_missed_m2=edge_missed_m2,
                missed_s=missed_s, total_s=total, drive_turn_s=terms['drive_s'] + terms['turn_s'])


def edge_mask(grid, settings):
    """Floor cells within `edge_band_m` of a wall or obstacle."""
    return grid.clearance < float(settings['cost'].get('edge_band_m', 0.0))
