"""Export a plan as Nav2 waypoints (YAML or JSON)."""
import datetime
import json
import math

import numpy as np
import yaml

from .planning import points_of


def _yaw(a, b, fallback=0.0):
    dx, dy = b[0] - a[0], b[1] - a[1]
    return math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-9 else fallback


def _pose(x, y, yaw, kind):
    return {'x': round(float(x), 4), 'y': round(float(y), 4), 'yaw': round(yaw, 5),
            'orientation': {'z': round(math.sin(yaw / 2), 6), 'w': round(math.cos(yaw / 2), 6)},
            'kind': kind}


def waypoints(result, max_gap=0.5):
    """Ordered poses that describe the whole driven path, transit included.

    Every vertex of the planned path is a pose (so Nav2 follows the planned
    shape instead of cutting across between far-apart goals), and straight
    stretches get an extra pose every `max_gap` m. Yaw points along the path.
    Send them with NavigateThroughPoses (send_to_nav2.py default): the robot
    drives through them without stopping at each one."""
    poses, last, yaw = [], None, 0.0

    def add(p, y, kind):
        poses.append(_pose(p[0], p[1], y, kind))

    for t, conn in zip(result['targets'], result['connections']):
        kind = getattr(t, 'kind', 'spiral')
        pts = [list(map(float, p)) for p in points_of(t)]
        is_point = len(pts) < 2 or all(math.dist(pts[0], q) < 1e-9 for q in pts)
        path = [(p, 'transit') for p in conn] + [(p, kind) for p in (pts[:1] if is_point else pts)]
        for (p, k), nxt in zip(path, path[1:] + [(None, None)]):
            if last is not None and math.dist(last, p) < 1e-6:
                continue
            if last is not None:              # fill long straight stretches
                n = int(math.ceil(math.dist(last, p) / max_gap))
                for f in [i / n for i in range(1, n)]:
                    add([last[0] + f * (p[0] - last[0]), last[1] + f * (p[1] - last[1])], _yaw(last, p, yaw), k)
                yaw = _yaw(last, p, yaw)
            if nxt[0] is not None and math.dist(p, nxt[0]) > 1e-6:
                yaw = _yaw(p, nxt[0], yaw) if last is None else yaw
            add(p, yaw, k)
            last = p
    return poses


def _plain(v):
    if isinstance(v, (bool, np.bool_)):
        return bool(v)
    if isinstance(v, (int, np.integer)):
        return int(v)
    if isinstance(v, (float, np.floating)):
        return round(float(v), 4)
    return v


def build(result, map_path, settings):
    return {
        'frame_id': 'map',
        'map': map_path,
        'created': datetime.datetime.now().isoformat(timespec='seconds'),
        'start': [round(v, 4) for v in result['start']],
        'area_polygon': [[round(float(x), 4), round(float(y), 4)] for x, y in (result['polygon'] or [])],
        'stats': {k: _plain(v) for k, v in result['stats'].items()},
        'settings': json.loads(json.dumps(settings)),
        'poses': waypoints(result),
        'segments': [{'kind': getattr(t, 'kind', 'spiral'),
                      'points': [[round(float(x), 4), round(float(y), 4)] for x, y in points_of(t)]}
                     for t in result['targets']],
    }


def save(result, path, map_path, settings):
    data = build(result, map_path, settings)
    with open(path, 'w') as f:
        if path.lower().endswith('.json'):
            json.dump(data, f, indent=2)
        else:
            yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None, width=120)
    return len(data['poses'])
