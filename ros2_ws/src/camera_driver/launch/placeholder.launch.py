from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="camera_driver",
                executable="camera_driver_node",
                name="camera_driver",
                output="screen",
                parameters=[{"enabled": True}],
            )
        ]
    )
