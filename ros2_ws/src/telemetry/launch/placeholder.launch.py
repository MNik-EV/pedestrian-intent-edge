from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="telemetry",
                executable="telemetry_node",
                name="telemetry",
                output="screen",
                parameters=[{"enabled": True}],
            )
        ]
    )
