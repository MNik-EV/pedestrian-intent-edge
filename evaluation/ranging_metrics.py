"""Metrics for physical camera–LiDAR object-ranging experiments."""

from __future__ import annotations

import csv
import math
import random
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable


REQUIRED_COLUMNS = {"sample_id", "ground_truth_m", "estimated_m"}


@dataclass(frozen=True)
class RangingSample:
    sample_id: str
    ground_truth_m: float
    estimated_m: float
    scene: str = "unspecified"
    object_class: str = "unspecified"
    fusion_method: str = "unspecified"
    bearing_deg: float | None = None
    fusion_confidence: float | None = None
    lidar_points: int | None = None

    @property
    def error_m(self) -> float:
        return self.estimated_m - self.ground_truth_m


@dataclass(frozen=True)
class RangingSummary:
    n: int
    mae_m: float
    rmse_m: float
    bias_m: float
    median_abs_error_m: float
    p95_abs_error_m: float
    relative_mae_percent: float
    mae_ci95_low_m: float
    mae_ci95_high_m: float

    def to_dict(self) -> dict[str, int | float]:
        return asdict(self)


def _optional_float(value: str | None) -> float | None:
    if value is None or not value.strip():
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _optional_int(value: str | None) -> int | None:
    number = _optional_float(value)
    return None if number is None else int(number)


def load_ranging_csv(path: Path) -> list[RangingSample]:
    """Load measured samples and reject malformed or non-physical values."""
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        columns = set(reader.fieldnames or [])
        missing = REQUIRED_COLUMNS - columns
        if missing:
            raise ValueError(f"Missing required CSV columns: {sorted(missing)}")
        samples: list[RangingSample] = []
        for line_number, row in enumerate(reader, start=2):
            try:
                ground_truth = float(row["ground_truth_m"])
                estimated = float(row["estimated_m"])
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"Invalid numeric value on CSV line {line_number}"
                ) from exc
            if not (math.isfinite(ground_truth) and math.isfinite(estimated)):
                raise ValueError(f"Non-finite measurement on CSV line {line_number}")
            if ground_truth <= 0 or estimated < 0:
                raise ValueError(f"Non-physical measurement on CSV line {line_number}")
            samples.append(
                RangingSample(
                    sample_id=(row.get("sample_id") or str(line_number - 1)).strip(),
                    ground_truth_m=ground_truth,
                    estimated_m=estimated,
                    scene=(row.get("scene") or "unspecified").strip(),
                    object_class=(row.get("object_class") or "unspecified").strip(),
                    fusion_method=(row.get("fusion_method") or "unspecified").strip(),
                    bearing_deg=_optional_float(row.get("bearing_deg")),
                    fusion_confidence=_optional_float(row.get("fusion_confidence")),
                    lidar_points=_optional_int(row.get("lidar_points")),
                )
            )
    return samples


def percentile(values: list[float], q: float) -> float:
    """Linearly interpolated percentile, where q is in [0, 1]."""
    if not values:
        return float("nan")
    ordered = sorted(values)
    position = (len(ordered) - 1) * max(0.0, min(1.0, q))
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _bootstrap_mae_ci(
    absolute_errors: list[float], *, repeats: int = 2000, seed: int = 20260906
) -> tuple[float, float]:
    if len(absolute_errors) < 2:
        value = absolute_errors[0] if absolute_errors else float("nan")
        return value, value
    rng = random.Random(seed)
    n = len(absolute_errors)
    estimates = [
        sum(absolute_errors[rng.randrange(n)] for _ in range(n)) / n
        for _ in range(repeats)
    ]
    return percentile(estimates, 0.025), percentile(estimates, 0.975)


def summarize_ranging(samples: Iterable[RangingSample]) -> RangingSummary:
    rows = list(samples)
    if not rows:
        raise ValueError("No measured ranging samples were supplied")
    errors = [row.error_m for row in rows]
    absolute = [abs(error) for error in errors]
    mae = statistics.fmean(absolute)
    low, high = _bootstrap_mae_ci(absolute)
    return RangingSummary(
        n=len(rows),
        mae_m=mae,
        rmse_m=math.sqrt(statistics.fmean(error * error for error in errors)),
        bias_m=statistics.fmean(errors),
        median_abs_error_m=statistics.median(absolute),
        p95_abs_error_m=percentile(absolute, 0.95),
        relative_mae_percent=100.0
        * statistics.fmean(abs(row.error_m) / row.ground_truth_m for row in rows),
        mae_ci95_low_m=low,
        mae_ci95_high_m=high,
    )


def distance_bin(distance_m: float) -> str:
    if distance_m < 1.0:
        return "<1 m"
    if distance_m < 2.0:
        return "1–2 m"
    if distance_m < 3.0:
        return "2–3 m"
    return "≥3 m"


def grouped_summaries(
    samples: list[RangingSample],
) -> dict[str, dict[str, RangingSummary]]:
    """Summarize predeclared groups; no post-hoc best-subset selection."""
    dimensions = {
        "scene": lambda row: row.scene,
        "object_class": lambda row: row.object_class,
        "fusion_method": lambda row: row.fusion_method,
        "distance_bin": lambda row: distance_bin(row.ground_truth_m),
    }
    output: dict[str, dict[str, RangingSummary]] = {}
    for dimension, key_function in dimensions.items():
        groups: dict[str, list[RangingSample]] = {}
        for row in samples:
            groups.setdefault(key_function(row), []).append(row)
        output[dimension] = {
            name: summarize_ranging(group) for name, group in sorted(groups.items())
        }
    return output
