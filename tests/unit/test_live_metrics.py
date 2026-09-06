"""Tests for live-run performance summaries."""

from __future__ import annotations

import pytest

from evaluation.evaluate_live_run import summarize_live_run


def test_live_summary_counts_lidar_range_coverage() -> None:
    records = [
        {
            "wall_ns": 1_000_000_000,
            "camera_fps": 10.0,
            "lidar_hz": 8.0,
            "detect_ms": 40.0,
            "camera_age_s": 0.02,
            "camera_connected": True,
            "objects": [
                {
                    "distance_m": 2.0,
                    "lidar_points": 5,
                    "fusion": "bearing_cluster_lidar",
                },
                {"distance_m": 1.5, "lidar_points": 0, "fusion": "monocular"},
            ],
        },
        {
            "wall_ns": 3_000_000_000,
            "camera_fps": 12.0,
            "lidar_hz": 9.0,
            "detect_ms": 60.0,
            "camera_age_s": 0.04,
            "camera_connected": False,
            "objects": [],
        },
    ]
    summary = summarize_live_run(records)
    assert summary["duration_s"] == pytest.approx(2.0)
    assert summary["detector_latency_ms"]["median"] == pytest.approx(50.0)
    assert summary["camera_connected_rate"] == pytest.approx(0.5)
    assert summary["any_range_coverage"] == pytest.approx(1.0)
    assert summary["lidar_range_coverage"] == pytest.approx(0.5)
