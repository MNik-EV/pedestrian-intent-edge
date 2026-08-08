from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="sensor_fusion",
                executable="sensor_fusion_node",
                name="sensor_fusion",
                output="screen",
                parameters=[{"enabled": True}],
            )
        ]
    )
