"""Tests for physical ranging metrics and strict CSV validation."""

from __future__ import annotations

import csv

import pytest

from evaluation.ranging_metrics import (
    RangingSample,
    load_ranging_csv,
    summarize_ranging,
)


def test_ranging_summary_has_expected_errors() -> None:
    rows = [
        RangingSample("1", 1.0, 1.1),
        RangingSample("2", 2.0, 1.8),
        RangingSample("3", 3.0, 3.3),
    ]
    summary = summarize_ranging(rows)
    assert summary.n == 3
    assert summary.mae_m == pytest.approx(0.2)
    assert summary.rmse_m == pytest.approx((0.14 / 3) ** 0.5)
    assert summary.bias_m == pytest.approx(0.2 / 3)
    assert summary.mae_ci95_low_m <= summary.mae_m <= summary.mae_ci95_high_m


def test_csv_loader_rejects_non_physical_measurement(tmp_path) -> None:
    path = tmp_path / "measurements.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sample_id", "ground_truth_m", "estimated_m"])
        writer.writerow(["bad", 0, 1.0])
    with pytest.raises(ValueError, match="Non-physical"):
        load_ranging_csv(path)
