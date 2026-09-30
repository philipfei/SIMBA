"""PC-only RViz launcher for relayed TF."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('slam', 'coverage', 'exploration'):
        raise RuntimeError('mode must be slam, coverage or exploration')
    config = Path(get_package_share_directory('simba_bringup')) / 'rviz' / f'{mode}.rviz'
    return [Node(package='rviz2', executable='rviz2', name=f'simba_{mode}_rviz',
                 arguments=['-d', str(config), '--ros-args', '-r', '/tf:=/tf_relay',
                            '-r', '/tf_static:=/tf_static_relay'], output='screen',
                 sigterm_timeout='6', sigkill_timeout='2')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('mode', default_value='coverage'),
                              OpaqueFunction(function=_setup)])
