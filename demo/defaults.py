"""Shared demo hardware defaults — USB PS3 Eye, not the laptop webcam."""

from __future__ import annotations

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
_HW = ROOT / "config" / "demo_hardware.yaml"

# Confirmed on this PC: OpenCV index 1 = USB PS3 Eye.
DEFAULT_CAMERA_INDEX = 1
DEFAULT_LIDAR_PORT = "COM17"
DEFAULT_LIDAR_BAUD = 230400


def load_demo_hardware() -> dict:
    if not _HW.is_file():
        return {
            "camera": {"index": DEFAULT_CAMERA_INDEX},
            "lidar": {"port": DEFAULT_LIDAR_PORT, "baudrate": DEFAULT_LIDAR_BAUD},
        }
    with _HW.open(encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data


def default_camera_index() -> int:
    data = load_demo_hardware()
    try:
        return int(data.get("camera", {}).get("index", DEFAULT_CAMERA_INDEX))
    except (TypeError, ValueError):
        return DEFAULT_CAMERA_INDEX


def default_lidar_port() -> str:
    data = load_demo_hardware()
    return str(data.get("lidar", {}).get("port", DEFAULT_LIDAR_PORT))
