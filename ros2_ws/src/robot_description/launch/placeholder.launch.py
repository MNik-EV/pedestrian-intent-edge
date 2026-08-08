from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="robot_description",
                executable="robot_description_node",
                name="robot_description",
                output="screen",
                parameters=[{"enabled": True}],
            )
        ]
    )
