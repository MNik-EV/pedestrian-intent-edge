"""Structured experiment logging (JSONL / CSV / SQLite)."""

from __future__ import annotations

import csv
import json
import sqlite3
import hashlib
import os
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable


@dataclass
class ExperimentMetadata:
    experiment_id: str
    start_time: str
    end_time: str | None
    configuration: dict[str, Any]
    git_commit: str
    hardware: dict[str, Any]
    software_version: str
    ros2_version: str
    model_version: str
    config_hash: str
    schema_version: int = 1

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def config_hash(cfg: dict[str, Any]) -> str:
    blob = json.dumps(cfg, sort_keys=True, default=str).encode("utf-8")
    return hashlib.sha256(blob).hexdigest()[:16]


def new_experiment_id(prefix: str = "EXP") -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y_%m%d_%H%M%S")
    return f"{prefix}_{stamp}"


class ExperimentLogger:
    """Creates experiments/<id>/ with metadata, metrics, events, and bags dir."""

    def __init__(self, root: str | Path, meta: ExperimentMetadata) -> None:
        self.root = Path(root)
        self.dir = self.root / meta.experiment_id
        self.dir.mkdir(parents=True, exist_ok=True)
        (self.dir / "rosbag").mkdir(exist_ok=True)
        (self.dir / "screenshots").mkdir(exist_ok=True)
        (self.dir / "plots").mkdir(exist_ok=True)
        self.meta = meta
        self._events = self.dir / "events.jsonl"
        self._metrics = self.dir / "metrics.csv"
        self._db = self.dir / "telemetry.sqlite"
        self._write_metadata()
        self._init_metrics()
        self._init_db()
        (self.dir / "README.md").write_text(
            f"# {meta.experiment_id}\n\nResearch experiment recording.\n",
            encoding="utf-8",
        )

    def _write_metadata(self) -> None:
        path = self.dir / "metadata.yaml"
        # YAML-ish without requiring PyYAML for bootstrap
        lines = ["# Experiment metadata", f"experiment_id: {self.meta.experiment_id}"]
        for k, v in self.meta.to_dict().items():
            if k == "experiment_id":
                continue
            if isinstance(v, (dict, list)):
                lines.append(f"{k}: {json.dumps(v)}")
            else:
                lines.append(f"{k}: {v}")
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        (self.dir / "metadata.json").write_text(
            json.dumps(self.meta.to_dict(), indent=2), encoding="utf-8"
        )

    def _init_metrics(self) -> None:
        if not self._metrics.exists():
            with self._metrics.open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        "wall_ns",
                        "metric",
                        "value",
                        "method",
                        "scenario",
                    ]
                )

    def _init_db(self) -> None:
        conn = sqlite3.connect(self._db)
        try:
            conn.execute(
                """
                CREATE TABLE IF NOT EXISTS samples (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    wall_ns INTEGER NOT NULL,
                    topic TEXT NOT NULL,
                    payload TEXT NOT NULL
                )
                """
            )
            conn.commit()
        finally:
            conn.close()

    def log_event(self, event_type: str, payload: dict[str, Any]) -> None:
        rec = {
            "type": event_type,
            "wall_ns": payload.get("wall_ns"),
            "payload": payload,
        }
        with self._events.open("a", encoding="utf-8") as f:
            f.write(json.dumps(rec, default=str) + "\n")

    def log_metric(self, metric: str, value: float, method: str = "", scenario: str = "") -> None:
        import time

        with self._metrics.open("a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([time.time_ns(), metric, value, method, scenario])

    def log_sample(self, topic: str, payload: dict[str, Any]) -> None:
        import time

        conn = sqlite3.connect(self._db)
        try:
            conn.execute(
                "INSERT INTO samples(wall_ns, topic, payload) VALUES (?, ?, ?)",
                (time.time_ns(), topic, json.dumps(payload, default=str)),
            )
            conn.commit()
        finally:
            conn.close()

    def close(self, end_time: str | None = None) -> None:
        self.meta.end_time = end_time or datetime.now(timezone.utc).isoformat()
        self._write_metadata()


class JsonlWriter:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, record: dict[str, Any]) -> None:
        with self.path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(record, default=str) + "\n")
