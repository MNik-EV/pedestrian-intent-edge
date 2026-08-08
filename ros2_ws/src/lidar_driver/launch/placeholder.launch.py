from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="lidar_driver",
                executable="lidar_driver_node",
                name="lidar_driver",
                output="screen",
                parameters=[{"enabled": True}],
            )
        ]
    )
