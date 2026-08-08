from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="slam_integration",
                executable="slam_integration_node",
                name="slam_integration",
                output="screen",
                parameters=[{"enabled": True}],
            )
        ]
    )
