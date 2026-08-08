from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="navigation",
                executable="navigation_node",
                name="navigation",
                output="screen",
                parameters=[{"enabled": True}],
            )
        ]
    )
