"""Structured JSONL + experiment folder logger for the live demo."""

from __future__ import annotations

import json
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


class DemoLogger:
    def __init__(self, root: str | Path = "experiments", prefix: str = "DEMO") -> None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        self.exp_id = f"{prefix}_{stamp}"
        self.dir = Path(root) / self.exp_id
        self.dir.mkdir(parents=True, exist_ok=True)
        self.jsonl = self.dir / "telemetry.jsonl"
        self.meta = {
            "experiment_id": self.exp_id,
            "started_at": datetime.now(timezone.utc).isoformat(),
            "mode": "live_hardware_demo",
            "notes": [
                "2D LD19 LiDAR only",
                "Practical field extrinsics (not formal optimization) unless calibrated",
                "No SLAM / navigation in this demo",
            ],
        }
        (self.dir / "metadata.json").write_text(json.dumps(self.meta, indent=2), encoding="utf-8")
        (self.dir / "README.md").write_text(
            f"# {self.exp_id}\n\nLive hardware demo log (camera + LD19 + detections).\n",
            encoding="utf-8",
        )
        self._n = 0

    def log(self, record: dict[str, Any]) -> None:
        record = dict(record)
        record["wall_ns"] = time.time_ns()
        record["seq"] = self._n
        self._n += 1
        with self.jsonl.open("a", encoding="utf-8") as f:
            f.write(json.dumps(record, default=str) + "\n")

    def close(self) -> None:
        self.meta["ended_at"] = datetime.now(timezone.utc).isoformat()
        self.meta["num_records"] = self._n
        (self.dir / "metadata.json").write_text(json.dumps(self.meta, indent=2), encoding="utf-8")
