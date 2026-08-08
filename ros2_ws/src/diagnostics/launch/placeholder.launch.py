from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="diagnostics",
                executable="diagnostics_node",
                name="diagnostics",
                output="screen",
                parameters=[{"enabled": True}],
            )
        ]
    )
