#!/usr/bin/env python3
"""Discover available hardware interfaces and write hardware_report.json."""

from __future__ import annotations

import json
import os
import platform
import socket
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from amp_core.hardware.discovery import discover_hardware


def _cpu_info() -> dict:
    info = {
        "processor": platform.processor(),
        "machine": platform.machine(),
        "cores_logical": os.cpu_count(),
    }
    try:
        import psutil  # type: ignore

        info["cpu_percent"] = psutil.cpu_percent(interval=0.2)
        info["ram_total_gb"] = round(psutil.virtual_memory().total / (1024**3), 2)
        info["ram_percent"] = psutil.virtual_memory().percent
    except Exception as exc:  # noqa: BLE001
        info["psutil_error"] = str(exc)
    return info


def _temp_c() -> float | None:
    try:
        zone = Path("/sys/class/thermal/thermal_zone0/temp")
        if zone.exists():
            return int(zone.read_text().strip()) / 1000.0
    except Exception:
        return None
    return None


def _network() -> list[dict]:
    out = []
    hostname = socket.gethostname()
    try:
        out.append({"hostname": hostname, "ip": socket.gethostbyname(hostname)})
    except Exception as exc:  # noqa: BLE001
        out.append({"hostname": hostname, "error": str(exc)})
    return out


def main() -> int:
    inv = discover_hardware()
    report = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "platform": {
            "system": platform.system(),
            "release": platform.release(),
            "version": platform.version(),
            "python": sys.version,
        },
        "cpu_ram": _cpu_info(),
        "temperature_c": _temp_c(),
        "hardware_inventory": inv.to_dict(),
        "serial_devices": inv.serial_ports,
        "video_devices": [
            f"index={c.index} {c.width}x{c.height} backend={c.backend} ({c.note})"
            for c in inv.cameras
            if c.ok
        ],
        "lidar_connected": inv.has_lidar,
        "camera_connected": inv.has_camera,
        "primary_camera_index": inv.primary_camera_index,
        "network": _network(),
        "recommendations": inv.recommendations,
        "notes": [
            "This local probe does not discover the Pi MJPEG camera; configure its URL in config/demo_hardware.yaml.",
            "LiDAR points are NOT faked when no serial LiDAR is found.",
            "Object distance without LiDAR uses monocular size geometry (approximate).",
        ],
    }
    out = Path("hardware_report.json")
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"\nWrote {out.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
