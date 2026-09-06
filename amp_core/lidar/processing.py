"""LiDAR scan filtering, sector distances, and obstacle extraction."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from amp_core.common.types import LidarScan, SectorDistances


@dataclass
class LidarFilterConfig:
    min_range: float = 0.12
    max_range: float = 12.0
    angle_min_deg: float = -180.0
    angle_max_deg: float = 180.0
    median_window: int = 3


SECTOR_DEFS: dict[str, tuple[float, float]] = {
    # name: (center_deg, half_width_deg)
    "front": (0.0, 22.5),
    "front_left": (45.0, 22.5),
    "front_right": (-45.0, 22.5),
    "left": (90.0, 22.5),
    "right": (-90.0, 22.5),
    "rear_left": (135.0, 22.5),
    "rear": (180.0, 22.5),
    "rear_right": (-135.0, 22.5),
}


def normalize_angle_deg(angle: float) -> float:
    a = (angle + 180.0) % 360.0 - 180.0
    return a


def filter_scan(scan: LidarScan, cfg: LidarFilterConfig) -> LidarScan:
    """Reject invalid ranges and out-of-FOV angles."""
    a_min = math.radians(cfg.angle_min_deg)
    a_max = math.radians(cfg.angle_max_deg)
    ranges: list[float] = []
    angles: list[float] = []
    for r, a in zip(scan.ranges, scan.angles):
        if a < a_min or a > a_max:
            continue
        if r != r or r < cfg.min_range or r > cfg.max_range:
            continue
        ranges.append(float(r))
        angles.append(float(a))
    return LidarScan(
        ranges=ranges,
        angles=angles,
        timestamp=scan.timestamp,
        frame_id=scan.frame_id,
        min_range=cfg.min_range,
        max_range=cfg.max_range,
        scan_time_s=scan.scan_time_s,
    )


def sector_distances(scan: LidarScan, default: float = 12.0) -> SectorDistances:
    """Compute minimum range in each of eight sectors."""
    buckets: dict[str, float] = {k: default for k in SECTOR_DEFS}
    for r, a in zip(scan.ranges, scan.angles):
        if r != r:
            continue
        adeg = normalize_angle_deg(math.degrees(a))
        for name, (center, half) in SECTOR_DEFS.items():
            # Handle rear wrap around ±180
            diff = abs(normalize_angle_deg(adeg - center))
            if diff <= half:
                buckets[name] = min(buckets[name], r)
    return SectorDistances(
        front=buckets["front"],
        front_left=buckets["front_left"],
        front_right=buckets["front_right"],
        left=buckets["left"],
        right=buckets["right"],
        rear_left=buckets["rear_left"],
        rear=buckets["rear"],
        rear_right=buckets["rear_right"],
        timestamp=scan.timestamp,
    )


@dataclass
class Obstacle:
    x: float
    y: float
    range_m: float
    bearing_deg: float


def extract_obstacles(
    scan: LidarScan, cluster_gap_m: float = 0.25, min_points: int = 3
) -> list[Obstacle]:
    """Simple polar clustering for obstacle extraction."""
    if not scan.ranges:
        return []
    pairs = sorted(zip(scan.angles, scan.ranges), key=lambda t: t[0])
    clusters: list[list[tuple[float, float]]] = []
    current: list[tuple[float, float]] = [pairs[0]]
    for i in range(1, len(pairs)):
        a0, r0 = current[-1]
        a1, r1 = pairs[i]
        # chord approximation
        gap = abs(r1 - r0) + abs(a1 - a0) * 0.5 * (r0 + r1)
        if gap > cluster_gap_m:
            if len(current) >= min_points:
                clusters.append(current)
            current = [(a1, r1)]
        else:
            current.append((a1, r1))
    if len(current) >= min_points:
        clusters.append(current)

    obstacles: list[Obstacle] = []
    for cluster in clusters:
        rs = [r for _, r in cluster]
        a_mean = sum(a for a, _ in cluster) / len(cluster)
        r_min = min(rs)
        obstacles.append(
            Obstacle(
                x=r_min * math.cos(a_mean),
                y=r_min * math.sin(a_mean),
                range_m=r_min,
                bearing_deg=math.degrees(a_mean),
            )
        )
    obstacles.sort(key=lambda o: o.range_m)
    return obstacles


def scan_quality_features(
    scan: LidarScan, expected_points: int = 360
) -> dict[str, float]:
    """Measurable features for LiDAR reliability estimation."""
    n = len(scan.ranges)
    if n == 0:
        return {
            "valid_ratio": 0.0,
            "point_density": 0.0,
            "range_jump_rate": 1.0,
            "mean_range": 0.0,
            "temporal_consistency": 0.0,
        }
    valid = [r for r in scan.ranges if r == r and scan.min_range <= r <= scan.max_range]
    valid_ratio = len(valid) / max(1, expected_points)
    jumps = 0
    for i in range(1, len(valid)):
        if abs(valid[i] - valid[i - 1]) > 1.0:
            jumps += 1
    jump_rate = jumps / max(1, len(valid) - 1)
    mean_range = sum(valid) / max(1, len(valid))
    density = n / max(1, expected_points)
    return {
        "valid_ratio": float(min(1.0, valid_ratio)),
        "point_density": float(min(1.0, density)),
        "range_jump_rate": float(min(1.0, jump_rate)),
        "mean_range": float(mean_range),
        "temporal_consistency": float(max(0.0, 1.0 - jump_rate)),
    }


def closest_obstacle(obstacles: Sequence[Obstacle]) -> Obstacle | None:
    return obstacles[0] if obstacles else None
