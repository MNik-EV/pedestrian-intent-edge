"""Robust camera–LiDAR distance fusion for detections (persons to ~4 m).

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
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from amp_core.calibration.transforms import CameraIntrinsics, ExtrinsicTransform
from amp_core.common.types import BoundingBox


PERSON_HEIGHT_M = 1.70
MAX_RANGE_M = 4.5
MIN_RANGE_M = 0.35


@dataclass
class FusedDistance:
    distance_m: float | None
    bearing_deg: float
    n_points: int
    method: str
    mono_prior_m: float | None
    confidence: float  # 0..1


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

    conf = min(1.0, n / 12.0)
    method = "bearing_cluster_lidar"
    if mono is not None:
        err = abs(float(np.median(depths[best])) - mono)
        if err < 0.45:
            conf = min(1.0, conf + 0.25)
            method = "bearing_cluster_lidar+mono"
        elif err > 1.2 and class_name == "person":
            dist = 0.65 * dist + 0.35 * mono
            conf *= 0.6
            method = "bearing_lidar_mono_blend"

    if reserve_points:
        for j in best:
            points[cand_idx[int(j)]].used = True

    dist = float(max(MIN_RANGE_M, min(MAX_RANGE_M, dist)))
    return FusedDistance(dist, bearing_deg, n, method, mono, float(conf))


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
