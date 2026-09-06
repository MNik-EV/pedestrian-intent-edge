"""Shared demo hardware defaults.

Primary camera path: Raspberry Pi Zero 2W + IMX219-120 CSI camera, streamed
over Wi-Fi as MJPEG/HTTP (see edge/camera_streamer.py and
amp_core/vision/network_camera.py). A directly-attached USB camera (index 1
on this dev machine, a PS3 Eye) remains available as a bench-debug fallback
when the Pi isn't powered on — set camera.mode: usb in demo_hardware.yaml.
"""

from __future__ import annotations

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
_HW = ROOT / "config" / "demo_hardware.yaml"

DEFAULT_CAMERA_MODE = "network"
DEFAULT_CAMERA_URL = "http://raspberrypi.local:8000/stream.mjpg"
DEFAULT_RECONNECT_TIMEOUT_S = 3.0
# Bench-debug fallback only (camera.mode: usb): confirmed on this dev PC,
# OpenCV index 1 = USB PS3 Eye.
DEFAULT_CAMERA_INDEX = 1
DEFAULT_LIDAR_PORT = "COM17"
DEFAULT_LIDAR_BAUD = 230400
DEFAULT_INTRINSICS_PATH = "calibration/imx219_intrinsics.yaml"
DEFAULT_EXTRINSICS_PATH = "calibration/imx219_ld19_extrinsics.yaml"


def load_demo_hardware() -> dict:
    if not _HW.is_file():
        return {
            "camera": {
                "mode": DEFAULT_CAMERA_MODE,
                "index": DEFAULT_CAMERA_INDEX,
                "network": {
                    "url": DEFAULT_CAMERA_URL,
                    "reconnect_timeout_s": DEFAULT_RECONNECT_TIMEOUT_S,
                },
            },
            "lidar": {"port": DEFAULT_LIDAR_PORT, "baudrate": DEFAULT_LIDAR_BAUD},
        }
    with _HW.open(encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data


def default_camera_mode() -> str:
    data = load_demo_hardware()
    return str(data.get("camera", {}).get("mode", DEFAULT_CAMERA_MODE))


def default_camera_url() -> str:
    data = load_demo_hardware()
    return str(data.get("camera", {}).get("network", {}).get("url", DEFAULT_CAMERA_URL))


def default_camera_index() -> int:
    data = load_demo_hardware()
    try:
        return int(data.get("camera", {}).get("index", DEFAULT_CAMERA_INDEX))
    except (TypeError, ValueError):
        return DEFAULT_CAMERA_INDEX


def default_lidar_port() -> str:
    data = load_demo_hardware()
    return str(data.get("lidar", {}).get("port", DEFAULT_LIDAR_PORT))
