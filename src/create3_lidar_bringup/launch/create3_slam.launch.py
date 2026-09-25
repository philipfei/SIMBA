import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("create3_lidar_bringup")
    slam_share = get_package_share_directory("slam_toolbox")

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "create3_lidar.launch.py")
        )
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "autostart": "true",
            "use_lifecycle_manager": "false",
            "use_sim_time": "false",
            "slam_params_file": os.path.join(
                bringup_share, "config", "slam_toolbox.yaml"
            ),
        }.items(),
    )

    tf_relay = Node(
        package="create3_lidar_bringup",
        executable="tf_relay.py",
        name="create3_tf_relay",
        output="screen",
    )

    safe_cmd_vel_relay = Node(
        package="create3_lidar_bringup",
        executable="safe_cmd_vel_relay.py",
        name="velocity_safety_gate",
        output="screen",
        parameters=[{"mapping_mode": True}],
    )

    return LaunchDescription([lidar, slam, tf_relay, safe_cmd_vel_relay])
