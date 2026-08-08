"""Gazebo / Ignition hooks — use mock stack on Windows; full sim on Ubuntu."""

# Placeholder world description for future Gazebo Harmonic integration.
world: indoor_simple
robots: 1
pedestrians: 2
sensors: [lidar, camera]
note: "Primary PC validation uses amp_core.mocks; Gazebo launch is ROS2/Ubuntu-only."
