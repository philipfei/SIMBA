"""One typed configuration and canonical effective-value identity."""
from copy import deepcopy
from decimal import Decimal
from pathlib import Path
import hashlib
import json
import math
import yaml


def resource(name):
    source=Path(__file__).resolve().parents[1]/'config'/name
    if source.is_file():return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('create3_lidar_bringup'))/'config'/name


def canonical_hash(values):
    def normalize(value):
        if isinstance(value,dict):return {k:normalize(v) for k,v in value.items()}
        if isinstance(value,list):return [normalize(v) for v in value]
        # Integral floats and integers have the same effective numeric value.
        # Non-integral floats retain full precision; never round settings values.
        if isinstance(value,float) and math.isfinite(value) and value.is_integer():return int(value)
        return value
    return hashlib.sha256(json.dumps(normalize(values),sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()


def _typed(value, template, path=''):
    if template is None:
        if value is None:return None
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:raise ValueError(path+' must be null or positive numeric')
        return float(value)
    if isinstance(template,dict):
        if not isinstance(value,dict):raise ValueError(path+' must be a mapping')
        if set(value)-set(template):raise ValueError(path+' unknown keys: '+str(set(value)-set(template)))
        return {k:_typed(value.get(k,v),v,path+'.'+k) for k,v in template.items()}
    if isinstance(template,bool):
        if not isinstance(value,bool):raise ValueError(path+' must be boolean')
    elif isinstance(template,(int,float)):
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):raise ValueError(path+' must be finite numeric')
        if isinstance(template,int) and int(value)!=value:raise ValueError(path+' must be integral')
        value=type(template)(value)
        if value==0:value=type(template)(0)
    elif not isinstance(value,type(template)):raise ValueError(path+' has the wrong type')
    return deepcopy(value)


class Settings:
    def __init__(self, values):
        self.values=values
        g=values['geometry'];self.collision=float(Decimal(str(g['body_radius_m']))+Decimal(str(g['planning_padding_m'])))
        p=values['planning'];m=values['motion']
        if g['body_radius_m']<=0 or g['planning_padding_m']<0 or g['coverage_disk_radius_m']<=0:raise ValueError('Invalid radii')
        if not 0<p['stripe_spacing_m']<=2*g['coverage_disk_radius_m']:raise ValueError('Invalid stripe spacing')
        if not 0<=p['area_tie_tolerance_ratio']<=.001:raise ValueError('Area tolerance must be 0..0.001')
        for section in ('motion','localization','health','measurement','deadlines','recovery','visualization'):
            for k,v in values[section].items():
                if isinstance(v,(int,float)) and v<=0:raise ValueError(section+'.'+k+' must be positive')
        for k in ('min_trim_segment_length_m','stationary_target_length_m','max_segment_length_m','angle_search_step_deg','min_wall_length_m','wall_cluster_tolerance_deg','min_wall_support_m'):
            if p[k]<=0:raise ValueError(k+' must be positive')
        if p['max_segment_length_m']<2*p['min_trim_segment_length_m']:raise ValueError('Maximum segment length must be at least twice minimum trimmed length')
        if not 0<p['min_wall_support_ratio']<=1 or not 0<values['completion']['target_coverage_ratio']<=1:raise ValueError('Invalid ratio')
        if not 0<values['localization']['matched_ratio']<=1:raise ValueError('Invalid match ratio')
        if not 0<values['battery']['critical_ratio']<values['battery']['return_ratio']<values['battery']['start_ratio']<=1:raise ValueError('Battery thresholds must be ordered')
        if values['resweep']['max_rounds']<0 or values['resweep']['max_targets']<1 or values['resweep']['minimum_gain_m2']<0:raise ValueError('Invalid resweep limits')
        if p['alignment'] not in ('auto','manual','off'):raise ValueError('Invalid alignment')
        if values['boundary']['direction'] not in ('auto','left','right') or values['boundary']['scope'] not in ('outer_and_obstacles','outer'):raise ValueError('Invalid boundary policy')
        # Derived settings override duplicated Nav2 template defaults before hashing.
        n=values['nav2'];r=values['runtime']
        for name in ('global_costmap','local_costmap'):
            n[name][name]['ros__parameters'].update(robot_radius=self.collision,footprint_padding=0.,robot_base_frame=r['base_frame'])
        n['amcl']['ros__parameters'].update(base_frame_id=r['base_frame'],odom_frame_id=r['odom_frame'],global_frame_id=r['map_frame'])
        n['map_server']['ros__parameters']['frame_id']=r['map_frame']
        n['global_costmap']['global_costmap']['ros__parameters']['global_frame']=r['map_frame']
        n['local_costmap']['local_costmap']['ros__parameters']['global_frame']=r['odom_frame']
        n['controller_server']['ros__parameters']['FollowPath'].update(desired_linear_vel=m['linear_m_s'],rotate_to_heading_angular_vel=m['angular_rad_s'],max_angular_accel=m['angular_accel_rad_s2'])
        self.hash=canonical_hash(values)

    def __getitem__(self,key):return self.values[key]


def load_settings(filename=None, runtime_overrides=None):
    template=yaml.safe_load(resource('coverage_settings.yaml').read_text())
    raw=yaml.safe_load(Path(filename).read_text()) if filename else template
    values=_typed(raw,template)
    if runtime_overrides:values['runtime'].update(runtime_overrides)
    return Settings(values)


def node_settings(node):
    filename=node.declare_parameter('config_file',str(resource('coverage_settings.yaml'))).value
    settings=load_settings(filename)
    overrides={k:node.declare_parameter(k,settings['runtime'][k]).value for k in ('map_yaml','output_dir','dock_file')}
    return load_settings(filename,overrides)


def offline_case(grid, registry=None):
    cases=yaml.safe_load(Path(registry or resource('offline_map_cases.yaml')).read_text())
    if grid.identity not in cases:raise ValueError('No versioned offline start registered for map '+grid.identity)
    case=cases[grid.identity]
    if case.get('frame')!='map' or len(case.get('start_xy_m',[]))!=2:raise ValueError('Invalid offline start fixture')
    start=case['start_xy_m']
    if not all(math.isfinite(x) for x in start):raise ValueError('Non-finite fixture start')
    return deepcopy(case),canonical_hash(case)
