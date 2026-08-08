from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="simulation",
                executable="simulation_node",
                name="simulation",
                output="screen",
                parameters=[{"enabled": True}],
            )
        ]
    )
