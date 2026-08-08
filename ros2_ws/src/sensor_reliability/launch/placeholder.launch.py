from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="sensor_reliability",
                executable="sensor_reliability_node",
                name="sensor_reliability",
                output="screen",
                parameters=[{"enabled": True}],
            )
        ]
    )
