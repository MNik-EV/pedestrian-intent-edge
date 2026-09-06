"""Tests for the precision-weighted cross-modal distance fusion contribution.

Covers three pieces, each checkable without any real hardware:
  1. The pure variance/inverse-variance-weighting math (closed-form checks).
  2. End-to-end behaviour through estimate_detection_distance(): a tight
     LiDAR cluster should dominate a much less certain monocular prior, and a
     genuine disagreement should be flagged as "conflict" rather than
     silently averaged away.
  3. The rolling CrossModalMonitor and the symmetric reliability discount it
     feeds into amp_core.reliability.estimator.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from amp_core.calibration.cross_modal_monitor import CrossModalMonitor
from amp_core.calibration.distance_fusion import (
    FusedDistance,
    _fuse_inverse_variance,
    _lidar_cluster_variance,
    _mono_depth_variance,
    estimate_detection_distance,
    prepare_lidar_camera_points,
)
from amp_core.calibration.transforms import CameraIntrinsics, ExtrinsicTransform
from amp_core.common.types import BoundingBox
from amp_core.reliability.estimator import ReliabilityConfig, _cross_modal_penalty


# ---------------------------------------------------------------------------
# 1. Pure math
# ---------------------------------------------------------------------------


def test_lidar_cluster_variance_floors_single_point() -> None:
    v = _lidar_cluster_variance(np.array([1.23]))
    assert v == pytest.approx(0.03**2)


def test_lidar_cluster_variance_uses_sample_spread_when_larger_than_floor() -> None:
    ranges = np.array([1.0, 1.2, 1.4, 1.6])  # std well above the 3cm floor
    v = _lidar_cluster_variance(ranges)
    assert v > 0.03**2
    assert v == pytest.approx(float(np.var(ranges, ddof=1)))


def test_mono_depth_variance_person_is_tighter_than_generic_class() -> None:
    z = 2.0
    v_person = _mono_depth_variance(z, "person")
    v_chair = _mono_depth_variance(z, "chair")
    assert v_person < v_chair  # anthropometric prior is tighter than the generic one


def test_fuse_inverse_variance_matches_closed_form() -> None:
    z1, v1 = 2.0, 0.01
    z2, v2 = 2.5, 0.04
    z, v = _fuse_inverse_variance(z1, v1, z2, v2)
    w1, w2 = 1 / v1, 1 / v2
    assert z == pytest.approx((z1 * w1 + z2 * w2) / (w1 + w2))
    assert v == pytest.approx(1 / (w1 + w2))
    # Fused variance must be smaller than either input (more information).
    assert v < min(v1, v2)


def test_fuse_inverse_variance_favors_the_more_certain_source() -> None:
    # Source 1 is far tighter (100x lower variance) than source 2.
    z, _ = _fuse_inverse_variance(2.0, 0.0001, 3.0, 0.09)
    assert abs(z - 2.0) < abs(z - 3.0)


# ---------------------------------------------------------------------------
# 2. End-to-end through estimate_detection_distance()
# ---------------------------------------------------------------------------

K = CameraIntrinsics(545.0, 548.0, 320.0, 240.0, 640, 480)
EXT = ExtrinsicTransform.lidar_to_camera_optical(0.0, -0.08, -0.03)


def _person_bbox_at_depth(depth_m: float) -> BoundingBox:
    h = K.fy * 1.70 / depth_m
    return BoundingBox(x1=280, y1=240 - h / 2, x2=360, y2=240 + h / 2)


def _lidar_points_at(depth_m: float, spread_m: float, n: int) -> tuple[list[float], list[float]]:
    ranges, angles = [], []
    for i in range(n):
        a = math.radians(-4 + 8 * i / max(1, n - 1))
        ranges.append(depth_m + (spread_m if i % 2 == 0 else -spread_m))
        angles.append(a)
    return ranges, angles


def test_tight_lidar_cluster_dominates_a_looser_monocular_prior() -> None:
    """A dense, tight LiDAR cluster should pull the fused estimate close to
    itself even when the (wide-uncertainty) monocular prior disagrees a bit,
    because inverse-variance weighting favors the more certain source."""
    true_depth = 2.0
    bbox = _person_bbox_at_depth(2.3)  # mono prior implies ~2.3 m (weaker cue)
    ranges, angles = _lidar_points_at(true_depth, spread_m=0.01, n=10)
    points = prepare_lidar_camera_points(ranges, angles, EXT, K)

    result = estimate_detection_distance(bbox, "person", points, K)
    assert result.distance_m is not None
    # Should land much closer to the tight LiDAR cluster than to the mono prior.
    assert abs(result.distance_m - true_depth) < abs(result.distance_m - 2.3)


def test_large_disagreement_is_flagged_as_conflict_not_hidden() -> None:
    """When LiDAR and the monocular prior disagree far beyond their combined
    uncertainty, the result must say so via consistency_flag/z_score rather
    than quietly blending the two."""
    bbox = _person_bbox_at_depth(3.8)  # mono prior ~3.8 m
    ranges, angles = _lidar_points_at(1.0, spread_m=0.01, n=10)  # LiDAR says ~1.0 m
    points = prepare_lidar_camera_points(ranges, angles, EXT, K)

    result = estimate_detection_distance(bbox, "person", points, K)
    assert result.z_score is not None
    assert result.z_score > 2.0
    assert result.consistency_flag == "conflict"


def test_agreement_is_flagged_consistent_with_reasonable_zscore() -> None:
    depth = 2.0
    bbox = _person_bbox_at_depth(depth)
    ranges, angles = _lidar_points_at(depth, spread_m=0.02, n=10)
    points = prepare_lidar_camera_points(ranges, angles, EXT, K)

    result = estimate_detection_distance(bbox, "person", points, K)
    assert result.consistency_flag == "consistent"
    assert result.z_score is not None and result.z_score <= 2.0


def test_no_lidar_case_is_single_source_with_no_zscore() -> None:
    out = FusedDistance(None, 0.0, 0, "no_lidar_in_bearing", None, 0.0)
    assert out.consistency_flag == "single_source"
    assert out.z_score is None


# ---------------------------------------------------------------------------
# 3. CrossModalMonitor + reliability discount
# ---------------------------------------------------------------------------


def _fused(z_score: float | None, flag: str) -> FusedDistance:
    return FusedDistance(2.0, 0.0, 5, "x", 2.1, 0.8, variance_m2=0.01, z_score=z_score, consistency_flag=flag)


def test_cross_modal_monitor_ema_and_conflict_counting() -> None:
    mon = CrossModalMonitor(alpha=0.5)
    mon.update([_fused(0.5, "consistent"), _fused(0.5, "consistent")])
    mon.update([_fused(3.0, "conflict")])
    feats = mon.features()
    assert feats["n_samples"] == 3
    assert feats["n_conflicts"] == 1
    # EMA should sit strictly between the first-frame mean and the new sample.
    assert 0.5 < feats["mean_abs_zscore"] < 3.0


def test_cross_modal_monitor_ignores_frames_without_zscores() -> None:
    mon = CrossModalMonitor()
    mon.update([FusedDistance(None, 0.0, 0, "no_lidar_in_bearing", None, 0.0)])
    assert mon.features()["n_samples"] == 0


def test_cross_modal_penalty_requires_minimum_samples() -> None:
    cfg = ReliabilityConfig()
    assert _cross_modal_penalty({"mean_abs_zscore": 10.0, "n_samples": 1}, cfg) == 1.0


def test_cross_modal_penalty_discounts_when_disagreement_persists() -> None:
    cfg = ReliabilityConfig()
    penalty = _cross_modal_penalty({"mean_abs_zscore": 4.0, "n_samples": 20}, cfg)
    assert penalty < 1.0
    assert penalty >= cfg.cross_modal_penalty_floor


def test_cross_modal_penalty_is_symmetric_and_floored() -> None:
    cfg = ReliabilityConfig()
    penalty = _cross_modal_penalty({"mean_abs_zscore": 100.0, "n_samples": 50}, cfg)
    assert penalty == cfg.cross_modal_penalty_floor
