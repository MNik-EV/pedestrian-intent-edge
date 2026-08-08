from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="lidar_perception",
                executable="lidar_perception_node",
                name="lidar_perception",
                output="screen",
                parameters=[{"enabled": True}],
            )
        ]
    )
