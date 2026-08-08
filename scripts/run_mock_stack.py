#!/usr/bin/env python3
"""Run the PC mock stack + web dashboard (no ROS2 required)."""

from __future__ import annotations

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

os.environ.setdefault("AMP_AUTH_DISABLED", "0")
os.environ.setdefault("AMP_CONTROL_TOKEN", "dev-token-change-me")
os.environ.setdefault("AMP_FUSION_MODE", "adaptive_fusion")


def main() -> None:
    import uvicorn

    print("AMP mock stack starting on http://127.0.0.1:8000")
    print("Control token:", os.environ["AMP_CONTROL_TOKEN"])
    uvicorn.run(
        "web_dashboard.backend.app:app",
        host="127.0.0.1",
        port=8000,
        reload=False,
    )


if __name__ == "__main__":
    main()
