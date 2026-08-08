from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="robot_bringup",
                executable="robot_bringup_node",
                name="robot_bringup",
                output="screen",
                parameters=[{"enabled": True}],
            )
        ]
    )
