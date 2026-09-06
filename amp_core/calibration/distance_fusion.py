"""Robust camera–LiDAR distance fusion for detections (persons to ~10 m).

Why naive "median of LiDAR points inside bbox" fails:
  • Tall person boxes swallow nearer clutter projected into the rectangle.
  • Extrinsic error shifts the overlay so the wrong ranges land in the box.
  • Median of a mixed near/far set is systematically biased short.

This module instead:
  1) Computes the camera horizontal bearing span of the bbox (intrinsics).
  2) Keeps only LiDAR returns whose camera-frame bearing falls in that span.
  3) Clusters depths and picks the cluster consistent with a monocular size prior
     (for persons: ~1.7 m body height → depth from bbox height).
  4) Reports a robust front-surface range (20th percentile of the chosen cluster).
  5) When both a LiDAR cluster and a monocular prior are available, combines
     them by inverse-variance (precision) weighting instead of a fixed blend
     ratio, and reports the resulting variance and a disagreement z-score —
     see _fuse_inverse_variance() and _lidar_cluster_variance()/
     _mono_depth_variance() below. This is the estimator that minimizes
     combined variance under independence, given each source's own
     uncertainty, rather than an arbitrary fixed split. A persistently large
     z-score (see amp_core/calibration/cross_modal_monitor.py) is a
     self-supervised signal that the two modalities disagree beyond what
     their stated uncertainties predict — useful even without ground truth.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from amp_core.calibration.transforms import CameraIntrinsics, ExtrinsicTransform
from amp_core.common.types import BoundingBox


PERSON_HEIGHT_M = 1.70
# LD19 is rated to ~12 m; 10 m keeps clear separation between e.g. two people
# standing 5 m and 7 m away in the same frame, with margin below the sensor's
# own noise floor at maximum range.
MAX_RANGE_M = 10.0
MIN_RANGE_M = 0.35

# LD19 single-shot range noise floor (order-of-magnitude from the datasheet,
# an engineering assumption, not a per-unit calibrated value).
LIDAR_RANGE_STD_M = 0.03
# Approximate adult standing-height population std-dev used only to turn the
# monocular height prior into a depth variance; this is a stated engineering
# assumption, not a per-subject measurement.
PERSON_HEIGHT_STD_M = 0.07
# Non-person classes use a deliberately wide fractional depth std, since the
# nominal-width cue has no comparable anthropometric backing.
GENERIC_CLASS_DEPTH_FRAC_STD = 0.25
# Disagreement beyond this many combined-sigma is flagged as a cross-modal
# conflict rather than silently averaged away.
CONFLICT_Z_SCORE = 2.0


@dataclass
class FusedDistance:
    distance_m: float | None
    bearing_deg: float
    n_points: int
    method: str
    mono_prior_m: float | None
    confidence: float  # 0..1
    variance_m2: float | None = None
    z_score: float | None = None
    consistency_flag: str = "single_source"  # single_source | consistent | conflict


def _lidar_cluster_variance(ranges_in_cluster: np.ndarray) -> float:
    """Variance of the chosen LiDAR cluster's range, floored at sensor noise."""
    floor = LIDAR_RANGE_STD_M**2
    if ranges_in_cluster.size < 2:
        return floor
    return max(floor, float(np.var(ranges_in_cluster, ddof=1)))


def _mono_depth_variance(z: float, class_name: str) -> float:
    """Propagate the monocular size-prior uncertainty into a depth variance.

    depth = f * H_ref / h_px, so a fractional error in the assumed real-world
    size H_ref maps to the same fractional error in depth (to first order).
    """
    frac = (
        PERSON_HEIGHT_STD_M / PERSON_HEIGHT_M
        if class_name == "person"
        else GENERIC_CLASS_DEPTH_FRAC_STD
    )
    return float((z * frac) ** 2)


def _fuse_inverse_variance(
    z1: float, v1: float, z2: float, v2: float
) -> tuple[float, float]:
    """Precision-weighted combination: the minimum-variance linear unbiased
    estimator of two independent measurements of the same quantity."""
    w1 = 1.0 / max(v1, 1e-6)
    w2 = 1.0 / max(v2, 1e-6)
    z = (z1 * w1 + z2 * w2) / (w1 + w2)
    v = 1.0 / (w1 + w2)
    return float(z), float(v)


@dataclass
class _LidarCam:
    range_m: float
    angle_rad: float
    bearing_c: float  # atan2(X, Z) in camera
    depth_c: float  # Z in camera (forward)
    u: float | None
    v: float | None
    used: bool = False


def _mono_depth_person(bbox: BoundingBox, K: CameraIntrinsics) -> float | None:
    h = max(1.0, bbox.y2 - bbox.y1)
    if h < 40 or h > K.height * 0.98:
        return None
    z = K.fy * PERSON_HEIGHT_M / h
    if z < MIN_RANGE_M or z > MAX_RANGE_M + 1.0:
        return None
    return float(z)


def _bbox_bearings(
    bbox: BoundingBox, K: CameraIntrinsics, margin_px: float = 8.0
) -> tuple[float, float, float]:
    """Return (theta_left, theta_right, theta_center) in camera frame radians."""
    x1 = max(0.0, bbox.x1 - margin_px)
    x2 = min(float(K.width - 1), bbox.x2 + margin_px)
    t1 = math.atan2(x1 - K.cx, K.fx)
    t2 = math.atan2(x2 - K.cx, K.fx)
    tc = math.atan2(bbox.cx - K.cx, K.fx)
    return min(t1, t2), max(t1, t2), tc


def prepare_lidar_camera_points(
    ranges: Sequence[float],
    angles: Sequence[float],
    ext: ExtrinsicTransform,
    K: CameraIntrinsics,
) -> list[_LidarCam]:
    out: list[_LidarCam] = []
    for r, a in zip(ranges, angles):
        if r != r or r < MIN_RANGE_M or r > MAX_RANGE_M:
            continue
        lx = r * math.cos(a)
        ly = r * math.sin(a)
        X, Y, Z = ext.transform_point(lx, ly, 0.0)
        if Z <= 0.08:
            continue
        bearing = math.atan2(X, Z)
        u = K.fx * (X / Z) + K.cx
        v = K.fy * (Y / Z) + K.cy
        out.append(
            _LidarCam(
                range_m=float(r),
                angle_rad=float(a),
                bearing_c=float(bearing),
                depth_c=float(Z),
                u=float(u),
                v=float(v),
            )
        )
    return out


def _cluster_depths(depths: np.ndarray, gap_m: float = 0.28) -> list[np.ndarray]:
    if depths.size == 0:
        return []
    order = np.argsort(depths)
    d = depths[order]
    clusters: list[list[int]] = [[int(order[0])]]
    for i in range(1, len(d)):
        if d[i] - d[i - 1] > gap_m:
            clusters.append([int(order[i])])
        else:
            clusters[-1].append(int(order[i]))
    return [np.array(c, dtype=np.int64) for c in clusters if len(c) >= 2]


def _robust_front_range(ranges: np.ndarray) -> float:
    if ranges.size == 1:
        return float(ranges[0])
    return float(np.percentile(ranges, 20))


def estimate_detection_distance(
    bbox: BoundingBox,
    class_name: str,
    points: list[_LidarCam],
    K: CameraIntrinsics,
    *,
    reserve_points: bool = True,
) -> FusedDistance:
    t_left, t_right, t_c = _bbox_bearings(bbox, K)
    span = max(0.02, t_right - t_left)
    pad = max(0.015, 0.15 * span)
    t_left -= pad
    t_right += pad

    mono = _mono_depth_person(bbox, K) if class_name == "person" else None
    if mono is None and class_name != "person":
        bw = max(1.0, bbox.x2 - bbox.x1)
        if 25 < bw < K.width * 0.8:
            z_w = K.fx * 0.45 / bw
            if MIN_RANGE_M <= z_w <= MAX_RANGE_M:
                mono = float(z_w)

    y1, y2 = bbox.y1, bbox.y2
    bh = max(1.0, y2 - y1)
    v_lo = y1 + 0.25 * bh
    v_hi = y1 + 0.85 * bh

    cand_idx: list[int] = []
    for i, p in enumerate(points):
        if p.used:
            continue
        if not (t_left <= p.bearing_c <= t_right):
            continue
        if p.depth_c < MIN_RANGE_M or p.depth_c > MAX_RANGE_M:
            continue
        if p.u is not None and not (bbox.x1 - 25 <= p.u <= bbox.x2 + 25):
            continue
        if p.v is not None and not (v_lo - 40 <= p.v <= v_hi + 40):
            if mono is None or abs(p.depth_c - mono) > 1.2:
                if class_name == "person":
                    continue
        cand_idx.append(i)

    bearing_deg = math.degrees(t_c)
    if len(cand_idx) < 3:
        cand_idx = [
            i
            for i, p in enumerate(points)
            if (not p.used)
            and t_left <= p.bearing_c <= t_right
            and MIN_RANGE_M <= p.depth_c <= MAX_RANGE_M
        ]
    if len(cand_idx) < 2:
        return FusedDistance(None, bearing_deg, 0, "no_lidar_in_bearing", mono, 0.0)

    depths = np.array([points[i].depth_c for i in cand_idx], dtype=np.float64)
    ranges = np.array([points[i].range_m for i in cand_idx], dtype=np.float64)
    clusters = _cluster_depths(depths, gap_m=0.30)
    if not clusters:
        clusters = [np.arange(len(cand_idx))]

    def cluster_score(ci: np.ndarray) -> tuple[float, float]:
        d = depths[ci]
        r = ranges[ci]
        med = float(np.median(d))
        n = float(len(ci))
        prior_term = 0.0
        if mono is not None:
            prior_term = abs(med - mono)
            if med < mono * 0.55:
                prior_term += (mono - med) * 2.5
        score = prior_term * 2.0 - 0.05 * n + 0.02 * med
        return score, _robust_front_range(r)

    best = min(clusters, key=lambda ci: cluster_score(ci)[0])
    _, dist = cluster_score(best)
    n = int(len(best))

    var_lidar = _lidar_cluster_variance(ranges[best])
    conf = min(1.0, n / 12.0)
    method = "bearing_cluster_lidar"
    variance_out = var_lidar
    z_score: float | None = None
    consistency_flag = "single_source"

    if mono is not None:
        var_mono = _mono_depth_variance(mono, class_name)
        disagreement = abs(dist - mono)
        z_score = disagreement / math.sqrt(var_lidar + var_mono)
        fused_dist, fused_var = _fuse_inverse_variance(dist, var_lidar, mono, var_mono)
        dist = fused_dist
        variance_out = fused_var
        if z_score <= CONFLICT_Z_SCORE:
            consistency_flag = "consistent"
            conf = min(1.0, conf + 0.25)
            method = "bearing_cluster_lidar+mono_ivw"
        else:
            # Still fuse (precision weighting already discounts whichever
            # source has higher variance) but flag the disagreement rather
            # than silently averaging it away.
            consistency_flag = "conflict"
            conf *= 0.6
            method = "bearing_cluster_lidar+mono_ivw_conflict"

    if reserve_points:
        for j in best:
            points[cand_idx[int(j)]].used = True

    dist = float(max(MIN_RANGE_M, min(MAX_RANGE_M, dist)))
    return FusedDistance(
        dist,
        bearing_deg,
        n,
        method,
        mono,
        float(conf),
        variance_m2=float(variance_out),
        z_score=z_score,
        consistency_flag=consistency_flag,
    )


def fuse_detections_distances(
    detections: Sequence[tuple[BoundingBox, str, float]],
    ranges: Sequence[float],
    angles: Sequence[float],
    ext: ExtrinsicTransform,
    K: CameraIntrinsics,
) -> list[FusedDistance]:
    """Associate each detection to a unique LiDAR depth cluster.

    detections: (bbox, class_name, confidence) — high confidence first.
    """
    points = prepare_lidar_camera_points(ranges, angles, ext, K)
    order = sorted(range(len(detections)), key=lambda i: detections[i][2], reverse=True)
    results: list[FusedDistance | None] = [None] * len(detections)
    for i in order:
        bbox, cls, _ = detections[i]
        results[i] = estimate_detection_distance(
            bbox, cls, points, K, reserve_points=True
        )
    return [r for r in results if r is not None]
