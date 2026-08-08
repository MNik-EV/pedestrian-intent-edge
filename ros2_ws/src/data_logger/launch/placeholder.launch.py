from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="data_logger",
                executable="data_logger_node",
                name="data_logger",
                output="screen",
                parameters=[{"enabled": True}],
            )
        ]
    )
