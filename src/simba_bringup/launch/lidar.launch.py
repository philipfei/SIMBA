"""RPLIDAR and the configured base-to-laser static transform."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_prefix, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parameters(config_dir):
    data = yaml.safe_load((Path(config_dir) / 'lidar_params.yaml').read_text())
    return data['/rplidar_node']['ros__parameters'], data['/laser_static_transform']['ros__parameters']


def _setup(context):
    config_dir = LaunchConfiguration('config_dir').perform(context)
    enabled = LaunchConfiguration('lidar_driver').perform(context).lower() in ('1', 'true', 'yes')
    driver, transform = _parameters(config_dir)
    nodes = [Node(
        package='tf2_ros', executable='static_transform_publisher', name='laser_static_transform',
        arguments=['--x', str(transform['x']), '--y', str(transform['y']), '--z', str(transform['z']),
                   '--roll', str(transform['roll']), '--pitch', str(transform['pitch']), '--yaw', str(transform['yaw']),
                   '--frame-id', transform['parent_frame'], '--child-frame-id', transform['child_frame']],
        output='screen', sigterm_timeout='6', sigkill_timeout='2')]
    if enabled:
        try:
            get_package_prefix('rplidar_ros')
        except PackageNotFoundError as error:
            raise RuntimeError('lidar_driver:=true requires the apt package ros-jazzy-rplidar-ros') from error
        nodes.append(TimerAction(period=2.0, actions=[Node(
            package='rplidar_ros', executable='rplidar_composition', name='rplidar_node',
            parameters=[driver], output='screen', sigterm_timeout='6', sigkill_timeout='2')]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_dir'),
        DeclareLaunchArgument('lidar_driver', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
