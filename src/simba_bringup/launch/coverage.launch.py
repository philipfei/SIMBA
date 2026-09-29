"""Fixed-map coverage bringup. Launching never starts robot motion."""
from pathlib import Path
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from simba_coverage.params import load_settings

CONTROLLER_REMAPS = [('cmd_vel', '/cmd_vel_nav'), ('/cmd_vel', '/cmd_vel_nav')]


def _node(package, executable, name, **kwargs):
    return Node(package=package, executable=executable, name=name, output='screen',
                sigterm_timeout='6', sigkill_timeout='2', **kwargs)


def _setup(context):
    share = Path(get_package_share_directory('simba_bringup'))
    config_dir = Path(LaunchConfiguration('config_dir').perform(context)).resolve()
    map_yaml = LaunchConfiguration('map').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)
    approved_hash = LaunchConfiguration('approved_hash').perform(context)
    plan = LaunchConfiguration('plan').perform(context)
    lidar_driver = LaunchConfiguration('lidar_driver').perform(context)
    stamped = LaunchConfiguration('nav2_stamped_supported').perform(context).lower() in ('1', 'true', 'yes')
    if not map_yaml:
        raise RuntimeError('coverage.launch.py requires map:=PATH_TO_MAP_YAML')
    if not output_dir:
        raise RuntimeError('coverage.launch.py requires output_dir:=PATH')
    if plan and not Path(plan).is_file():
        raise RuntimeError('coverage.launch.py plan:= file does not exist: ' + plan)
    settings = load_settings(config_dir, {'map_yaml': map_yaml, 'output_dir': output_dir, 'dock_file': ''})
    nav_dir = Path(tempfile.mkdtemp(prefix='simba-nav2-'))
    nav_file = settings.render_nav2(map_yaml, nav_dir, stamped_supported=stamped)
    shared = {'config_dir': str(config_dir), 'map_yaml': str(Path(map_yaml).resolve()),
              'output_dir': str(Path(output_dir).resolve()), 'dock_file': '',
              'approved_hash': approved_hash}
    nodes = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/lidar.launch.py')),
        launch_arguments={'config_dir': str(config_dir), 'lidar_driver': lidar_driver}.items())]
    nodes += [
        _node('nav2_map_server', 'map_server', 'map_server', parameters=[str(nav_file)]),
        _node('nav2_amcl', 'amcl', 'amcl', parameters=[str(nav_file)]),
        _node('nav2_planner', 'planner_server', 'planner_server', parameters=[str(nav_file)]),
        _node('nav2_controller', 'controller_server', 'controller_server', parameters=[str(nav_file)],
              remappings=CONTROLLER_REMAPS),
        _node('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_localization',
              parameters=[{'autostart': True, 'use_sim_time': False, 'node_names': ['map_server', 'amcl']}]),
        _node('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_navigation',
              parameters=[{'autostart': True, 'use_sim_time': False,
                           'node_names': ['planner_server', 'controller_server']}]),
        _node('simba_coverage', 'coverage_supervisor', 'coverage_supervisor',
              parameters=[{**shared, 'plan_file': str(Path(plan).resolve()) if plan else ''}]),
        _node('simba_coverage', 'coverage_meter', 'coverage_meter', parameters=[shared]),
        _node('simba_bringup', 'velocity_gate', 'velocity_safety_gate',
              parameters=[{**shared, 'mapping_mode': False}]),
        _node('simba_bringup', 'tf_relay', 'simba_tf_relay'),
    ]
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('simba_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('config_dir', default_value=str(share / 'config')),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument('approved_hash', default_value=''),
        DeclareLaunchArgument('plan', default_value='',
                              description='coverage_tool export to drive instead of on-robot planning'),
        DeclareLaunchArgument('lidar_driver', default_value='true'),
        DeclareLaunchArgument('nav2_stamped_supported', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
