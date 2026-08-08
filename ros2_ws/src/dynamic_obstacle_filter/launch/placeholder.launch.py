from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="dynamic_obstacle_filter",
                executable="dynamic_obstacle_filter_node",
                name="dynamic_obstacle_filter",
                output="screen",
                parameters=[{"enabled": True}],
            )
        ]
    )
