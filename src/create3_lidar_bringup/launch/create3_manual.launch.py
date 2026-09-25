"""Manual sensor bringup without a mapping or localization publisher."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package = 'create3_lidar_bringup'
    share = get_package_share_directory(package)
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'create3_lidar.launch.py'))),
        Node(package=package, executable='tf_relay.py', name='create3_tf_relay', output='screen'),
        Node(package=package, executable='safe_cmd_vel_relay.py', name='velocity_safety_gate', output='screen', parameters=[{'mapping_mode': True}]),
    ])
