"""Exported coverage_tool plans: load, check against the loaded map, split into targets."""
from pathlib import Path
import hashlib
import json
import math
import yaml
from .planning import split_path


def map_image_sha256(map_yaml):
    map_yaml=Path(map_yaml).resolve()
    return hashlib.sha256((map_yaml.parent/yaml.safe_load(map_yaml.read_text())['image']).read_bytes()).hexdigest()


def load_plan(path,grid,map_yaml,map_frame,collision,max_segment):
    """Return the plan's driven polyline and its targets, or raise ValueError naming the problem."""
    path=Path(path)
    if not path.is_file():raise ValueError('PLAN_FILE_MISSING: '+str(path))
    text=path.read_text()
    data=json.loads(text) if path.suffix=='.json' else yaml.safe_load(text)
    if not isinstance(data,dict) or not isinstance(data.get('poses'),list):raise ValueError('PLAN_FILE_INVALID: no poses list')
    if data.get('frame_id','map')!=map_frame:raise ValueError(f"PLAN_FRAME_MISMATCH: plan uses {data.get('frame_id')}, map uses {map_frame}")
    # Plans exported before the fingerprint existed rely on the clearance check below.
    expected=data.get('map_image_sha256')
    if expected and expected!=map_image_sha256(map_yaml):raise ValueError('PLAN_MAP_MISMATCH: the plan was made on a different map image')
    points=[]
    for p in data['poses']:
        xy=[float(p['x']),float(p['y'])]
        if not all(math.isfinite(v) for v in xy):raise ValueError('PLAN_FILE_INVALID: non-finite pose')
        if not points or math.dist(points[-1],xy)>1e-6:points.append(xy)
    if len(points)<2:raise ValueError('PLAN_FILE_INVALID: fewer than two distinct poses')
    unsafe=[i for i,(a,b) in enumerate(zip(points,points[1:])) if not grid.segment_safe(a,b,collision)]
    if unsafe:
        x,y=points[unsafe[0]]
        raise ValueError(f'PLAN_TOO_CLOSE_TO_OBSTACLES: {len(unsafe)} of {len(points)-1} segments come within '
                         f'{collision:g} m of a wall/unknown cell (first near x={x:.2f}, y={y:.2f}); '
                         're-plan in coverage_tool with a larger robot radius')
    return {'points':points,'targets':split_path(points,max_segment,'plan'),
            'plan_sha256':hashlib.sha256(text.encode()).hexdigest()}
