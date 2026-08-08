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
        # Linux thermal zones (Pi)
        zone = Path("/sys/class/thermal/thermal_zone0/temp")
        if zone.exists():
            return int(zone.read_text().strip()) / 1000.0
    except Exception:
        return None
    return None


def _list_serial_candidates() -> list[str]:
    candidates = []
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/serial/by-id/*"):
        import glob

        candidates.extend(glob.glob(pattern))
    # Windows COM ports
    if platform.system() == "Windows":
        try:
            import serial.tools.list_ports  # type: ignore

            candidates.extend([p.device for p in serial.tools.list_ports.comports()])
        except Exception:
            pass
    return sorted(set(candidates))


def _list_video() -> list[str]:
    import glob

    vids = glob.glob("/dev/video*")
    if platform.system() == "Windows":
        vids.append("DirectShow: available (use OpenCV index enumeration on deploy)")
    return vids


def _network() -> list[dict]:
    out = []
    hostname = socket.gethostname()
    try:
        out.append({"hostname": hostname, "ip": socket.gethostbyname(hostname)})
    except Exception as exc:  # noqa: BLE001
        out.append({"hostname": hostname, "error": str(exc)})
    return out


def main() -> int:
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
        "serial_devices": _list_serial_candidates(),
        "video_devices": _list_video(),
        "network": _network(),
        "notes": [
            "LD19 typically appears as a USB serial device.",
            "PS3 Eye typically appears as a UVC /dev/video* device on Linux.",
            "This PC report may not include robot peripherals until connected.",
        ],
    }
    out = Path("hardware_report.json")
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"\nWrote {out.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
