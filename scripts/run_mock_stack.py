#!/usr/bin/env python3
"""Run the controlled mock stack for software rehearsal and ablation."""

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
os.environ.setdefault("AMP_FORCE_MOCK", "1")


def main() -> None:
    import uvicorn
    from amp_core.hardware.discovery import discover_hardware

    inv = discover_hardware()
    print("=== Hardware discovery ===")
    for line in inv.recommendations:
        print(" -", line)
    print("Sensor source: MOCK (physical inventory below is informational only)")
    print(
        f"Detected local camera: {inv.primary_camera_index if inv.has_camera else 'none'}"
    )
    print("Detected serial candidate:", "yes" if inv.has_lidar else "no")
    print()
    print("Detector: stub (deterministic mock input)")
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
