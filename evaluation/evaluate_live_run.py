#!/usr/bin/env python3
"""Summarize measured live-demo telemetry rates, latency, and range coverage."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from evaluation.ranging_metrics import percentile  # noqa: E402


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    records = []
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"Invalid JSON on line {line_number}") from exc
        if not isinstance(value, dict):
            raise ValueError(f"Expected an object on JSONL line {line_number}")
        records.append(value)
    return records


def _numbers(records: list[dict[str, Any]], key: str) -> list[float]:
    output = []
    for record in records:
        value = record.get(key)
        if isinstance(value, (int, float)) and math.isfinite(value):
            output.append(float(value))
    return output


def _distribution(values: list[float]) -> dict[str, int | float | None]:
    if not values:
        return {"n": 0, "median": None, "p05": None, "p95": None, "mean": None}
    return {
        "n": len(values),
        "median": statistics.median(values),
        "p05": percentile(values, 0.05),
        "p95": percentile(values, 0.95),
        "mean": statistics.fmean(values),
    }


def summarize_live_run(records: list[dict[str, Any]]) -> dict[str, Any]:
    if not records:
        raise ValueError("Telemetry file has no records")
    timestamps = _numbers(records, "wall_ns")
    duration_s = (
        max(0.0, (timestamps[-1] - timestamps[0]) / 1e9)
        if len(timestamps) >= 2
        else 0.0
    )
    objects = [obj for record in records for obj in (record.get("objects") or [])]
    ranged = [obj for obj in objects if obj.get("distance_m") is not None]
    lidar_ranged = [
        obj
        for obj in ranged
        if int(obj.get("lidar_points") or 0) >= 2
        and (not obj.get("fusion") or "lidar" in str(obj.get("fusion")))
    ]
    camera_states = [record.get("camera_connected") for record in records]
    known_camera_states = [state for state in camera_states if isinstance(state, bool)]
    return {
        "records": len(records),
        "duration_s": duration_s,
        "camera_fps": _distribution(_numbers(records, "camera_fps")),
        "lidar_scan_hz": _distribution(_numbers(records, "lidar_hz")),
        "detector_latency_ms": _distribution(_numbers(records, "detect_ms")),
        "camera_frame_age_s": _distribution(_numbers(records, "camera_age_s")),
        "camera_connected_rate": (
            sum(bool(state) for state in known_camera_states) / len(known_camera_states)
            if known_camera_states
            else None
        ),
        "detections": len(objects),
        "detections_with_any_range": len(ranged),
        "detections_with_lidar_range": len(lidar_ranged),
        "any_range_coverage": len(ranged) / len(objects) if objects else None,
        "lidar_range_coverage": len(lidar_ranged) / len(objects) if objects else None,
    }


def _fmt(value: Any, digits: int = 3) -> str:
    return "NOT YET MEASURED" if value is None else f"{value:.{digits}f}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", required=True, type=Path)
    args = parser.parse_args()
    telemetry = args.experiment / "telemetry.jsonl"
    if not telemetry.is_file():
        print(f"Missing {telemetry}; results remain NOT YET MEASURED.", file=sys.stderr)
        return 2
    summary = summarize_live_run(load_jsonl(telemetry))
    (args.experiment / "runtime_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    report = f"""# Runtime summary

| Quantity | Value |
|---|---:|
| Records | {summary["records"]} |
| Duration [s] | {_fmt(summary["duration_s"])} |
| Camera FPS, median | {_fmt(summary["camera_fps"]["median"])} |
| LiDAR scan rate, median [Hz] | {_fmt(summary["lidar_scan_hz"]["median"])} |
| Detector latency, median [ms] | {_fmt(summary["detector_latency_ms"]["median"])} |
| Detector latency, P95 [ms] | {_fmt(summary["detector_latency_ms"]["p95"])} |
| Camera connected rate | {_fmt(summary["camera_connected_rate"])} |
| Any-range coverage | {_fmt(summary["any_range_coverage"])} |
| LiDAR-range coverage | {_fmt(summary["lidar_range_coverage"])} |
"""
    (args.experiment / "runtime_summary.md").write_text(report, encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
