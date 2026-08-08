from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="evaluation_tools",
                executable="evaluation_tools_node",
                name="evaluation_tools",
                output="screen",
                parameters=[{"enabled": True}],
            )
        ]
    )
