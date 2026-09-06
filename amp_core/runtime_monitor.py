"""System monitor service — CPU/RAM/temp for diagnostics & dashboard."""

from __future__ import annotations

import json
import logging
import time
from pathlib import Path

logging.basicConfig(level=logging.INFO)
log = logging.getLogger("amp.monitor")


def read_stats() -> dict:
    stats = {
        "cpu_percent": None,
        "ram_percent": None,
        "temperature_c": None,
        "disk_percent": None,
    }
    try:
        import psutil

        stats["cpu_percent"] = psutil.cpu_percent(interval=0.2)
        stats["ram_percent"] = psutil.virtual_memory().percent
        stats["disk_percent"] = psutil.disk_usage("/").percent
    except Exception as exc:  # noqa: BLE001
        stats["psutil_error"] = str(exc)
    zone = Path("/sys/class/thermal/thermal_zone0/temp")
    if zone.exists():
        try:
            stats["temperature_c"] = int(zone.read_text().strip()) / 1000.0
        except Exception:
            pass
    return stats


def main() -> None:
    out = Path("logs/system_stats.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    log.info("Monitor writing %s", out)
    while True:
        out.write_text(json.dumps(read_stats()), encoding="utf-8")
        time.sleep(1.0)


if __name__ == "__main__":
    main()
