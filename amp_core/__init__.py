"""Adaptive Multimodal Perception (AMP) core library.

ROS-agnostic algorithms for low-cost indoor robot research.

Implemented/tested architecture: all perception, fusion, and the dashboard
run on a laptop (Windows/Linux). The LD19 LiDAR is wired directly to the
laptop over USB-serial; the camera (IMX219-120 CSI) is captured by a
Raspberry Pi Zero 2W acting only as a network camera relay (see edge/ and
amp_core/vision/network_camera.py) — the Pi Zero 2W does no AI or fusion
itself. A full onboard-compute robot (Raspberry Pi 5, ROS2, SLAM,
navigation — see ros2_ws/, deployment/, simulation/) is documented future
work, not part of the implemented/tested thesis system.
"""

from __future__ import annotations

__version__ = "0.1.0"
__title__ = "Adaptive Multimodal Perception and Sensor Fusion"

VERSION = __version__
SCHEMA_VERSION = 1
