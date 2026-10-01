"""Planner settings. A dict with a `.collision` attribute (robot radius in m),
which is the shape planning.py expects."""
import copy
import json

DEFAULTS = {
    'geometry': {
        'robot_radius_m': 0.23,          # keeps the robot centre this far from walls/obstacles; SIMBA needs >= 0.23
        'coverage_disk_radius_m': 0.4,   # radius of the area the tool/sensor covers
    },
    'spiral': {
        'overlap_pct': 10.0,             # 0-50 %; path spacing = 2 x coverage radius x (1 - overlap)
        'obstacle_mode': 'around',       # 'around' = loops also circle obstacles, 'outer' = follow outer walls only
        'wall_side': 'right',            # side the wall is on while following it ('right' = counter-clockwise)
    },
    'strategy': {
        'mode': 'auto',                  # 'auto' = cheapest of all, 'regions' = mixed lanes/spiral per region, 'spiral'
        'wall_loop_first': True,         # start by following the outer walls
    },
    'cost': {                            # all terms in seconds, see coverage/cost.py
        'smooth_turn_deg': 20.0,         # bends below this are driven without stopping
        'bend_s_per_rad': 1.5,           # ...but still cost this much per radian (straight = cheapest)
        'stop_penalty_s': 1.0,           # extra time per sharp corner (brake, rotate, accelerate)
        'reverse_angle_deg': 150.0,      # direction change counted as a reversal (hairpin)
        'reverse_penalty_s': 10.0,       # extra cost per reversal
        'overlap_weight': 1.0,           # x time to sweep the double-covered area
        'missed_weight': 10000.0,        # x time to sweep the missed reachable area (very heavy: ~167 s per 5 cm cell)
        'edge_band_m': 0.10,             # floor this close to a wall/obstacle counts as an 'edge' cell...
        'edge_missed_weight': 10000.0,   # ...and costs this weight when missed (lower it for straighter lines)
    },
    'planning': {
        'max_segment_length_m': 2.0,     # longest single Nav2 goal segment
    },
    'motion': {
        'linear_m_s': 0.3,
        'angular_rad_s': 0.8,
    },
    'optimizer': {
        'search_time_s': 4.0,            # route optimisation per candidate pattern
        'polish_time_s': 15.0,           # extra optimisation of the cheapest candidate
        'neighbours': 8,                 # nearby pieces tried per move
    },
}


class Settings(dict):
    @property
    def collision(self):
        return float(self['geometry']['robot_radius_m'])


_current = Settings(copy.deepcopy(DEFAULTS))


def default_settings():
    return Settings(copy.deepcopy(DEFAULTS))


def set_settings(s):
    global _current
    _current = s if isinstance(s, Settings) else Settings(s)


def load_settings(path=None):
    if path is None:
        return _current
    with open(path) as f:
        data = json.load(f)
    s = default_settings()
    for k, v in data.items():
        if isinstance(v, dict) and k in s:
            s[k].update({kk: vv for kk, vv in v.items() if kk in s[k]})
    return s


def save_settings(s, path):
    with open(path, 'w') as f:
        json.dump(s, f, indent=2)
