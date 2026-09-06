#!/usr/bin/env python3
"""Run PC stack with laptop webcam + YOLOv8 detection."""

from __future__ import annotations

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

os.environ.setdefault("AMP_AUTH_DISABLED", "0")
os.environ.setdefault("AMP_CONTROL_TOKEN", "dev-token-change-me")
os.environ.setdefault("AMP_FUSION_MODE", "adaptive_fusion")
os.environ.setdefault("AMP_DETECTOR", "auto")
os.environ.setdefault("AMP_FORCE_MOCK", "0")


def main() -> None:
    import uvicorn
    from amp_core.hardware.discovery import discover_hardware

    inv = discover_hardware()
    print("=== Hardware discovery ===")
    for line in inv.recommendations:
        print(" -", line)
    print(
        f"Camera: {'LIVE idx=' + str(inv.primary_camera_index) if inv.has_camera else 'NOT FOUND'}"
    )
    print("LiDAR:", "FOUND" if inv.has_lidar else "NOT CONNECTED")
    print()
    print("Detector: YOLOv8n (first run downloads ~6MB weights)")
    print("AMP stack: http://127.0.0.1:8000")
    print("Token:", os.environ["AMP_CONTROL_TOKEN"])
    uvicorn.run(
        "web_dashboard.backend.app:app",
        host="127.0.0.1",
        port=8000,
        reload=False,
    )


if __name__ == "__main__":
    main()
