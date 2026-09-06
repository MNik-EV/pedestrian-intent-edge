"""Structured JSONL + experiment folder logger for the live demo."""

from __future__ import annotations

import json
import hashlib
import importlib.metadata
import platform
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from amp_core import __version__


REPO_ROOT = Path(__file__).resolve().parents[1]


def _git_value(*args: str) -> str:
    try:
        return subprocess.check_output(
            ["git", *args], cwd=REPO_ROOT, stderr=subprocess.DEVNULL, text=True
        ).strip()
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def _sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _package_version(name: str) -> str | None:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return None


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
            "git_commit": _git_value("rev-parse", "HEAD"),
            "git_dirty": bool(_git_value("status", "--porcelain").strip()),
            "software": {
                "amp_version": __version__,
                "python": sys.version.split()[0],
                "opencv": _package_version("opencv-python"),
                "ultralytics": _package_version("ultralytics"),
                "pyserial": _package_version("pyserial"),
            },
            "host": {
                "platform": platform.platform(),
                "machine": platform.machine(),
                "processor": platform.processor(),
            },
            "artifacts": self._artifact_manifest(),
            "notes": [
                "2D LD19 LiDAR only",
                "Practical target-based extrinsics (not formal multi-pose metrology)",
                "No SLAM / navigation in this demo",
            ],
        }
        (self.dir / "metadata.json").write_text(
            json.dumps(self.meta, indent=2), encoding="utf-8"
        )
        (self.dir / "README.md").write_text(
            f"# {self.exp_id}\n\nLive hardware demo log (camera + LD19 + detections).\n",
            encoding="utf-8",
        )
        self._n = 0

    @staticmethod
    def _artifact_manifest() -> list[dict[str, Any]]:
        paths = [
            REPO_ROOT / "config" / "demo_hardware.yaml",
            REPO_ROOT / "config" / "detection.yaml",
            REPO_ROOT / "calibration" / "imx219_intrinsics.yaml",
            REPO_ROOT / "calibration" / "imx219_ld19_extrinsics.yaml",
            REPO_ROOT / "yolov8n.pt",
        ]
        return [
            {
                "path": path.relative_to(REPO_ROOT).as_posix(),
                "exists": path.is_file(),
                "sha256": _sha256(path),
            }
            for path in paths
        ]

    def update_context(self, **context: Any) -> None:
        """Persist runtime facts learned after sensors and detector start."""
        self.meta.setdefault("runtime", {}).update(context)
        self._write_metadata()

    def _write_metadata(self) -> None:
        (self.dir / "metadata.json").write_text(
            json.dumps(self.meta, indent=2, ensure_ascii=False), encoding="utf-8"
        )

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
        self._write_metadata()
