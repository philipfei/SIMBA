from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


SERIAL_PORT = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="laser_static_transform",
                arguments=[
                    "--x", "0",
                    "--y", "0",
                    "--z", "0.0994",
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", "3.141592653589793",
                    "--frame-id", "base_footprint",
                    "--child-frame-id", "laser",
                ],
                output="screen",
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="rplidar_ros",
                        executable="rplidar_composition",
                        name="rplidar_node",
                        parameters=[
                            {
                                "serial_port": SERIAL_PORT,
                                "serial_baudrate": 115200,
                                "frame_id": "laser",
                                "inverted": False,
                                "angle_compensate": True,
                            }
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )
