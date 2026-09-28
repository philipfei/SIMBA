"""Manual sensor and safety bringup. It never grants manual ownership automatically."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('simba_bringup'))
    config = LaunchConfiguration('config_dir')
    lidar = LaunchConfiguration('lidar_driver')
    common = {'config_dir': config, 'mapping_mode': True}
    return LaunchDescription([
        DeclareLaunchArgument('config_dir', default_value=str(share / 'config')),
        DeclareLaunchArgument('lidar_driver', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/lidar.launch.py')),
                                 launch_arguments={'config_dir': config, 'lidar_driver': lidar}.items()),
        Node(package='simba_bringup', executable='tf_relay', name='simba_tf_relay', output='screen',
             sigterm_timeout='6', sigkill_timeout='2'),
        Node(package='simba_bringup', executable='velocity_gate', name='velocity_safety_gate',
             parameters=[common], output='screen', sigterm_timeout='6', sigkill_timeout='2'),
    ])
