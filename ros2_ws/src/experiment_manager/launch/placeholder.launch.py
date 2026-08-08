from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="experiment_manager",
                executable="experiment_manager_node",
                name="experiment_manager",
                output="screen",
                parameters=[{"enabled": True}],
            )
        ]
    )
