"""Fresh SLAM, native Undock, manual exploration and supervised Nav2 return."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from simba_coverage.params import load_settings
from simba_bringup.mode_guard import preflight
from simba_bringup.exploration import load_profile

CONTROLLER_REMAPS = [('cmd_vel', '/cmd_vel_nav'), ('/cmd_vel', '/cmd_vel_nav')]


def _node(package, executable, name, **kwargs):
    return Node(package=package, executable=executable, name=name, output='screen',
                sigterm_timeout='6', sigkill_timeout='2', **kwargs)


def _setup(context):
    share = Path(get_package_share_directory('simba_bringup'))
    config = Path(LaunchConfiguration('config_dir').perform(context)).resolve()
    auto = LaunchConfiguration('auto_undock').perform(context).lower() == 'true'
    settings = load_settings(config)
    load_profile(config)  # Validate before creating hardware or action clients.
    slam = yaml.safe_load((config / 'slam_toolbox.yaml').read_text())['slam_toolbox']['ros__parameters']
    if slam.get('mode') != 'mapping' or slam.get('map_file_name', ''):
        raise RuntimeError('Exploration requires fresh mapping with empty map_file_name')
    directory = Path(tempfile.mkdtemp(prefix='simba-explore-nav2-'))
    nav_file = settings.render_nav2(directory=directory, live_map=True)
    config_args = {'config_dir': str(config)}
    actions = [LogInfo(msg=(
        'AUTOMATIC UNDOCK ENABLED: robot will reverse and rotate 180 degrees. '
        'Clear the rear travel area and turning envelope; PC stop may not be connected.'
        if auto else 'auto_undock=false: stationary diagnostics; relaunch with true to begin.'))]
    actions += [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/lidar.launch.py')),
            launch_arguments={**config_args, 'lidar_driver': LaunchConfiguration('lidar_driver')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('slam_toolbox')) / 'launch/online_async_launch.py')),
            launch_arguments={'autostart': 'true', 'use_lifecycle_manager': 'false',
                'use_sim_time': 'false', 'slam_params_file': str(config / 'slam_toolbox.yaml')}.items()),
        _node('simba_bringup', 'tf_relay', 'simba_tf_relay'),
        _node('nav2_planner', 'planner_server', 'planner_server', parameters=[str(nav_file)]),
        _node('nav2_controller', 'controller_server', 'controller_server',
              parameters=[str(nav_file)], remappings=CONTROLLER_REMAPS),
        _node('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_navigation',
              parameters=[{'autostart': True, 'use_sim_time': False,
                           'node_names': ['planner_server', 'controller_server']}]),
        _node('simba_bringup', 'velocity_gate', 'velocity_safety_gate',
              parameters=[{**config_args, 'mapping_mode': True, 'supervised_mapping': True}]),
        _node('simba_bringup', 'exploration_coordinator', 'exploration_coordinator',
              parameters=[{**config_args, 'auto_undock': auto}]),
    ]
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('simba_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('config_dir', default_value=str(share / 'config')),
        DeclareLaunchArgument('lidar_driver', default_value='true'),
        DeclareLaunchArgument('auto_undock', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=preflight),
        OpaqueFunction(function=_setup),
    ])
