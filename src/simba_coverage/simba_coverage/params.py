"""Load, validate, canonicalize, and render SIMBA configuration."""
from copy import deepcopy
from decimal import Decimal
from pathlib import Path
import hashlib
import json
import math
import os
import tempfile
import yaml

CONFIG_FILES = (
    'robot_params.yaml', 'lidar_params.yaml', 'slam_toolbox.yaml',
    'nav2_params.yaml', 'coverage_params.yaml', 'offline_maps.yaml')


def default_config_dir():
    candidate = Path.cwd() / 'src' / 'simba_bringup' / 'config'
    if candidate.is_dir():
        return candidate
    source = Path(__file__).resolve().parents[2] / 'simba_bringup' / 'config'
    if source.is_dir():
        return source
    raise ValueError('No source-tree config directory found; pass --config-dir explicitly')



def resource(name):
    """Return a source-tree configuration path for tests and tooling."""
    return default_config_dir() / name


def _read(path):
    with Path(path).open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f'{path}: expected a mapping')
    return data


def _block(document, selector, path):
    unknown = set(document) - {selector}
    if unknown:
        raise ValueError(f'{path}: unknown node selectors: {sorted(unknown)}')
    try:
        block = document[selector]
        if set(block) != {'ros__parameters'} or not isinstance(block['ros__parameters'], dict):
            raise KeyError
        return deepcopy(block['ros__parameters'])
    except (KeyError, TypeError):
        raise ValueError(f'{path}: expected {selector}/ros__parameters') from None


def _float_token(value):
    if not math.isfinite(value):
        raise ValueError('Configuration contains NaN or infinity')
    return value.hex()


def canonical_value(value):
    if isinstance(value, dict):
        return {str(k): canonical_value(value[k]) for k in sorted(value, key=str)}
    if isinstance(value, (list, tuple)):
        return [canonical_value(v) for v in value]
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return value
    if isinstance(value, (int, float)):
        return {'__number_hex__': _float_token(float(value))}
    raise TypeError(f'Unsupported canonical value type: {type(value).__name__}')


def canonical_bytes(value):
    return json.dumps(canonical_value(value), sort_keys=True, separators=(',', ':'),
                      ensure_ascii=True, allow_nan=False).encode('utf-8')


def canonical_hash(value):
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def file_hash(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def mask_hash(mask):
    import numpy as np
    packed = np.packbits(np.asarray(mask, dtype=np.uint8).reshape(-1), bitorder='little')
    shape = list(np.asarray(mask).shape)
    return canonical_hash({'shape': shape, 'packed_sha256': hashlib.sha256(packed.tobytes()).hexdigest()})


_REQUIRED = {
    'geometry': {'body_radius_m', 'planning_padding_m', 'coverage_disk_radius_m'},
    'motion': {'linear_m_s', 'angular_rad_s', 'linear_accel_m_s2', 'angular_accel_rad_s2',
               'manual_forward_m_s', 'manual_reverse_m_s', 'manual_angular_rad_s'},
    'runtime': {'map_frame', 'odom_frame', 'base_frame', 'map_yaml', 'output_dir', 'dock_file'},
    'create3': {'expected_safety_override', 'mismatch_policy'},
    'coverage': {'planning', 'boundary', 'completion', 'resweep', 'localization', 'health',
                 'measurement', 'battery', 'deadlines', 'recovery', 'visualization'},
}


def _exact_keys(data, expected, name):
    missing = expected - set(data)
    unknown = set(data) - expected
    if missing or unknown:
        raise ValueError(f'{name}: missing={sorted(missing)} unknown={sorted(unknown)}')


def _positive_tree(section, values):
    for key, value in values.items():
        if isinstance(value, bool) or value is None or isinstance(value, str):
            continue
        if isinstance(value, (int, float)) and (not math.isfinite(value) or value <= 0):
            raise ValueError(f'{section}.{key} must be positive')


class Settings:
    """Validated effective values with collision radius derived exactly once."""
    def __init__(self, values, config_dir):
        self.values = values
        self.config_dir = Path(config_dir).resolve()
        geometry = values['geometry']
        motion = values['motion']
        planning = values['planning']
        self.collision = float(Decimal(str(geometry['body_radius_m'])) +
                               Decimal(str(geometry['planning_padding_m'])))
        if geometry['body_radius_m'] <= 0 or geometry['planning_padding_m'] < 0:
            raise ValueError('Invalid body radius or planning padding')
        if geometry['coverage_disk_radius_m'] <= 0:
            raise ValueError('Invalid coverage disk radius')
        if not 0 < planning['stripe_spacing_m'] <= 2 * geometry['coverage_disk_radius_m']:
            raise ValueError('Invalid stripe spacing')
        if not 0 <= planning['area_tie_tolerance_ratio'] <= 0.001:
            raise ValueError('Area tolerance must be in [0, 0.001]')
        if planning['min_trim_segment_length_m'] <= 0:
            raise ValueError('Minimum trimmed segment length must be positive')
        if not 0 < values['completion']['target_coverage_ratio'] <= 1:
            raise ValueError('Invalid target coverage ratio')
        if values['resweep']['max_rounds'] < 0:
            raise ValueError('Invalid resweep count')
        if values['create3']['mismatch_policy'] != 'warn':
            raise ValueError('Create 3 mismatch policy must be warn')
        if values['create3']['expected_safety_override'] != 'backup_only':
            raise ValueError('Expected Create 3 safety override must be backup_only')
        for section in ('motion', 'localization', 'health', 'measurement', 'deadlines',
                        'recovery', 'visualization'):
            _positive_tree(section, values[section])
        if not (0 < values['battery']['critical_ratio'] < values['battery']['return_ratio'] <
                values['battery']['start_ratio'] <= 1):
            raise ValueError('Battery thresholds must be ordered')
        self.hash = canonical_hash(self.behavior_values())

    def __getitem__(self, key):
        return self.values[key]

    def behavior_values(self):
        return {key: self.values[key] for key in (
            'geometry', 'motion', 'planning', 'boundary', 'completion', 'resweep',
            'localization', 'health', 'measurement', 'battery', 'deadlines',
            'recovery', 'visualization')}

    def robot_approval_subset(self):
        return {
            'geometry': self.values['geometry'],
            'motion': {key: self.values['motion'][key] for key in (
                'linear_m_s', 'angular_rad_s', 'linear_accel_m_s2', 'angular_accel_rad_s2')},
            'frames': {key: self.values['runtime'][key] for key in (
                'map_frame', 'odom_frame', 'base_frame')},
        }

    def render_nav2(self, map_yaml='', directory=None, stamped_supported=True, live_map=False):
        nav2 = deepcopy(self.values['nav2'])
        runtime = self.values['runtime']
        motion = self.values['motion']
        for name in ('global_costmap', 'local_costmap'):
            params = nav2[name][name]['ros__parameters']
            params['robot_radius'] = self.collision
            params['footprint_padding'] = 0.0
            params['robot_base_frame'] = runtime['base_frame']
        nav2['amcl']['ros__parameters'].update(
            base_frame_id=runtime['base_frame'], odom_frame_id=runtime['odom_frame'],
            global_frame_id=runtime['map_frame'])
        nav2['map_server']['ros__parameters'].update(
            frame_id=runtime['map_frame'], yaml_filename=str(Path(map_yaml).resolve()))
        nav2['global_costmap']['global_costmap']['ros__parameters']['global_frame'] = runtime['map_frame']
        nav2['local_costmap']['local_costmap']['ros__parameters']['global_frame'] = runtime['odom_frame']
        navigator = nav2['bt_navigator']['ros__parameters']
        navigator.update(global_frame=runtime['map_frame'], robot_base_frame=runtime['base_frame'])
        tree = self.config_dir / 'navigate_through_poses.xml'
        if not tree.is_file():
            raise ValueError(f'Missing behavior tree: {tree}')
        navigator['default_nav_through_poses_bt_xml'] = str(tree)
        controller = nav2['controller_server']['ros__parameters']
        controller['FollowPath'].update(
            desired_linear_vel=motion['linear_m_s'],
            rotate_to_heading_angular_vel=motion['angular_rad_s'],
            max_angular_accel=motion['angular_accel_rad_s2'])
        if stamped_supported:
            controller['enable_stamped_cmd_vel'] = False
        else:
            controller.pop('enable_stamped_cmd_vel', None)
        if live_map:
            profile = _read(self.config_dir / 'exploration_params.yaml')['navigation']
            nav2.pop('amcl')
            nav2.pop('map_server')
            nav2['planner_server']['ros__parameters']['GridBased'].update(
                tolerance=profile['planner_tolerance_m'], allow_unknown=False,
                use_final_approach_orientation=False)
            controller['goal_checker'].update(
                xy_goal_tolerance=profile['goal_xy_m'], yaw_goal_tolerance=profile['goal_yaw_rad'])
            controller['FollowPath']['use_rotate_to_heading'] = True
            global_params = nav2['global_costmap']['global_costmap']['ros__parameters']
            global_params.update(rolling_window=False, map_topic='/map')
            global_params.pop('width', None)
            global_params.pop('height', None)
            global_params['static_layer'].update(
                map_topic='/map', map_subscribe_transient_local=True, subscribe_to_updates=False)
        target = Path(directory) if directory else Path(tempfile.mkdtemp(prefix='simba-nav2-'))
        target.mkdir(parents=True, exist_ok=True)
        output = target / f'nav2-{self.hash}.yaml'
        output.write_text(yaml.safe_dump(nav2, sort_keys=False), encoding='utf-8')
        return output


def load_settings(config_dir=None, runtime_overrides=None):
    config_dir = Path(config_dir or default_config_dir()).resolve()
    missing = [name for name in CONFIG_FILES if not (config_dir / name).is_file()]
    if missing:
        raise ValueError(f'Missing configuration files in {config_dir}: {missing}')
    robot = _block(_read(config_dir / 'robot_params.yaml'), '/simba_defaults', 'robot_params.yaml')
    coverage = _block(_read(config_dir / 'coverage_params.yaml'), '/coverage_config', 'coverage_params.yaml')
    _exact_keys(robot, {'geometry', 'motion', 'runtime', 'create3'}, 'robot_params.yaml')
    for key in ('geometry', 'motion', 'runtime', 'create3'):
        _exact_keys(robot[key], _REQUIRED[key], f'robot_params.yaml:{key}')
    _exact_keys(coverage, _REQUIRED['coverage'], 'coverage_params.yaml')
    values = deepcopy(robot)
    values.update(coverage)
    values['nav2'] = _read(config_dir / 'nav2_params.yaml')
    if runtime_overrides:
        unknown = set(runtime_overrides) - _REQUIRED['runtime']
        if unknown:
            raise ValueError(f'Unknown runtime overrides: {sorted(unknown)}')
        values['runtime'].update(runtime_overrides)
    return Settings(values, config_dir)


def node_settings(node):
    config_dir = node.declare_parameter('config_dir', '').value
    if not config_dir:
        config_dir = str(default_config_dir())
    base = load_settings(config_dir)
    overrides = {}
    for key in ('map_yaml', 'output_dir', 'dock_file'):
        overrides[key] = node.declare_parameter(key, base['runtime'][key]).value
    return load_settings(config_dir, overrides)


def offline_cases(config_dir):
    document = _read(Path(config_dir) / 'offline_maps.yaml')
    params = _block(document, '/coverage_preview', 'offline_maps.yaml')
    fixtures = params.get('fixtures')
    if not isinstance(fixtures, dict):
        raise ValueError('offline_maps.yaml: fixtures must be a mapping')
    result = {}
    for key, value in fixtures.items():
        if not key.startswith('map_') or not isinstance(value, dict):
            raise ValueError(f'Invalid offline map fixture: {key}')
        identity = value.get('identity')
        if key != 'map_' + str(identity):
            raise ValueError(f'Offline map fixture key mismatch: {key}')
        result[identity] = deepcopy(value)
    return result


def offline_case(grid, config_dir=None, start=None):
    config_dir = Path(config_dir or default_config_dir())
    case = offline_cases(config_dir).get(grid.identity)
    if case is None:
        if start is None:
            raise ValueError('Map is not registered in offline_maps.yaml; --start X Y is required')
        case = {'identity': grid.identity, 'frame': 'map', 'start_xy_m': list(start),
                'region': 'whole_map', 'description': 'Explicit unregistered preview start'}
    elif start is not None:
        case['start_xy_m'] = list(start)
    if case.get('frame') != 'map' or case.get('region') != 'whole_map':
        raise ValueError('Only frame=map and region=whole_map are supported in v1')
    point = case.get('start_xy_m')
    if not isinstance(point, list) or len(point) != 2 or not all(math.isfinite(float(x)) for x in point):
        raise ValueError('Offline start must contain two finite coordinates')
    return case, canonical_hash(case)


def approval_manifest(settings, map_yaml, reachable, coverable, selected, region='whole_map'):
    map_yaml = Path(map_yaml).resolve()
    metadata = _read(map_yaml)
    image = (map_yaml.parent / metadata['image']).resolve()
    stripe = {
        'direction_deg': float(selected['angle_deg']),
        'offset_m': float(selected['offset_m']),
        'spacing_m': float(settings['planning']['stripe_spacing_m']),
    }
    components = {
        'coverage_params': canonical_hash({key: settings.values[key] for key in (
            'planning', 'boundary', 'completion', 'resweep', 'localization', 'health',
            'measurement', 'battery', 'deadlines', 'recovery', 'visualization')}),
        'robot_params': canonical_hash(settings.robot_approval_subset()),
        'map_yaml': file_hash(map_yaml),
        'map_image': file_hash(image),
        'region': canonical_hash({'type': region}),
        'reachable_mask': mask_hash(reachable),
        'coverable_mask': mask_hash(coverable),
        'stripe_family': canonical_hash(stripe),
    }
    manifest = {'schema_version': 1, 'components': components, 'stripe_family': stripe,
                'region': {'type': region}}
    manifest['approval_hash'] = canonical_hash(manifest)
    return manifest
