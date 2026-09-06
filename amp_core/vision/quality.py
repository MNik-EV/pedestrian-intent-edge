"""Vision quality metrics and lightweight feature tracking helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class VisionQualityFeatures:
    feature_count: float
    feature_spread: float
    brightness: float
    sharpness: float
    optical_flow_consistency: float
    mean_reprojection_error: float
    motion_blur_score: float

    def to_dict(self) -> dict[str, float]:
        return {
            "feature_count": self.feature_count,
            "feature_spread": self.feature_spread,
            "brightness": self.brightness,
            "sharpness": self.sharpness,
            "optical_flow_consistency": self.optical_flow_consistency,
            "mean_reprojection_error": self.mean_reprojection_error,
            "motion_blur_score": self.motion_blur_score,
        }


def estimate_brightness_gray(pixels: bytes, sample_stride: int = 16) -> float:
    """Mean brightness in [0, 1] from mono8 bytes."""
    if not pixels:
        return 0.0
    total = 0
    count = 0
    for i in range(0, len(pixels), sample_stride):
        total += pixels[i]
        count += 1
    return (total / max(1, count)) / 255.0


def estimate_sharpness_laplacian_proxy(pixels: bytes, width: int, height: int) -> float:
    """Cheap gradient-magnitude sharpness proxy normalized to ~[0, 1]."""
    if width < 3 or height < 3 or not pixels:
        return 0.0
    acc = 0.0
    count = 0
    stride = max(1, width // 40)
    for y in range(1, height - 1, stride):
        row = y * width
        for x in range(1, width - 1, stride):
            i = row + x
            if i + width >= len(pixels) or i - width < 0:
                continue
            gx = int(pixels[i + 1]) - int(pixels[i - 1])
            gy = int(pixels[i + width]) - int(pixels[i - width])
            acc += abs(gx) + abs(gy)
            count += 1
    if count == 0:
        return 0.0
    # Normalize: typical gradients 0..80 -> map to 0..1
    return max(0.0, min(1.0, (acc / count) / 80.0))


def feature_spatial_spread(
    points: list[tuple[float, float]], width: int, height: int
) -> float:
    """Normalized spatial distribution of features in [0, 1]."""
    if len(points) < 2 or width <= 0 or height <= 0:
        return 0.0
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    sx = (max(xs) - min(xs)) / width
    sy = (max(ys) - min(ys)) / height
    return max(0.0, min(1.0, 0.5 * (sx + sy)))


def compute_vision_features(
    gray: bytes,
    width: int,
    height: int,
    feature_points: list[tuple[float, float]] | None = None,
    flow_consistency: float = 0.8,
    reprojection_error: float = 0.5,
) -> VisionQualityFeatures:
    feature_points = feature_points or []
    brightness = estimate_brightness_gray(gray)
    sharpness = estimate_sharpness_laplacian_proxy(gray, width, height)
    # Penalize extremes of brightness (too dark / washed out)
    brightness_quality = 1.0 - abs(brightness - 0.45) / 0.55
    brightness_quality = max(0.0, min(1.0, brightness_quality))
    blur = max(0.0, 1.0 - sharpness)
    count_norm = min(1.0, len(feature_points) / 200.0)
    spread = feature_spatial_spread(feature_points, width, height)
    return VisionQualityFeatures(
        feature_count=count_norm,
        feature_spread=spread,
        brightness=brightness_quality,
        sharpness=sharpness,
        optical_flow_consistency=max(0.0, min(1.0, flow_consistency)),
        mean_reprojection_error=max(0.0, min(5.0, reprojection_error)),
        motion_blur_score=blur,
    )


def synthetic_feature_grid(
    width: int, height: int, step: int = 40
) -> list[tuple[float, float]]:
    """Deterministic feature grid for mock / degraded-environment tests."""
    pts: list[tuple[float, float]] = []
    for y in range(step, height - step, step):
        for x in range(step, width - step, step):
            pts.append((float(x), float(y)))
    return pts


def optical_flow_consistency_score(
    prev_pts: list[tuple[float, float]],
    next_pts: list[tuple[float, float]],
    max_disp: float = 30.0,
) -> float:
    """Fraction of correspondences with plausible displacement."""
    n = min(len(prev_pts), len(next_pts))
    if n == 0:
        return 0.0
    ok = 0
    for i in range(n):
        dx = next_pts[i][0] - prev_pts[i][0]
        dy = next_pts[i][1] - prev_pts[i][1]
        if math.hypot(dx, dy) <= max_disp:
            ok += 1
    return ok / n
