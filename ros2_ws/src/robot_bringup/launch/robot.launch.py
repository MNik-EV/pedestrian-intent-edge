"""Bring-up launch for AMP robot (ROS2)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    fusion_mode = LaunchConfiguration("fusion_mode")
    return LaunchDescription(
        [
            DeclareLaunchArgument("fusion_mode", default_value="adaptive_fusion"),
            Node(
                package="lidar_driver",
                executable="lidar_driver_node",
                name="lidar_driver",
                output="screen",
            ),
            Node(
                package="camera_driver",
                executable="camera_driver_node",
                name="camera_driver",
                output="screen",
            ),
            Node(
                package="sensor_reliability",
                executable="sensor_reliability_node",
                name="sensor_reliability",
                output="screen",
            ),
            Node(
                package="sensor_fusion",
                executable="sensor_fusion_node",
                name="sensor_fusion",
                parameters=[{"fusion_mode": fusion_mode}],
                output="screen",
            ),
            Node(
                package="navigation",
                executable="navigation_node",
                name="navigation",
                output="screen",
            ),
        ]
    )
