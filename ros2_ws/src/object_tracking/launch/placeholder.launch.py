from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="object_tracking",
                executable="object_tracking_node",
                name="object_tracking",
                output="screen",
                parameters=[{"enabled": True}],
            )
        ]
    )
