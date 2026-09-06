"""Dynamic obstacle filtering for SLAM / occupancy."""

from __future__ import annotations

from dataclasses import dataclass

from amp_core.common.types import LidarScan, TrackedObject
from amp_core.lidar.processing import Obstacle


@dataclass
class DynamicFilterConfig:
    enabled: bool = True
    association_bearing_deg: float = 12.0
    association_range_m: float = 0.6
    only_dynamic_tracks: bool = True


@dataclass
class DynamicFilterResult:
    dynamic_obstacles: list[Obstacle]
    static_scan: LidarScan
    removed_indices: list[int]

    def to_dict(self) -> dict:
        return {
            "dynamic_count": len(self.dynamic_obstacles),
            "static_points": len(self.static_scan.ranges),
            "removed_indices_count": len(self.removed_indices),
        }


def _angle_diff_deg(a: float, b: float) -> float:
    d = (a - b + 180.0) % 360.0 - 180.0
    return abs(d)


class DynamicObstacleFilter:
    """Remove LiDAR returns associated with dynamic tracked objects."""

    def __init__(self, cfg: DynamicFilterConfig | None = None) -> None:
        self.cfg = cfg or DynamicFilterConfig()

    def filter(
        self,
        scan: LidarScan,
        tracks: list[TrackedObject],
    ) -> DynamicFilterResult:
        if not self.cfg.enabled:
            return DynamicFilterResult([], scan, [])

        dynamic_tracks = [
            t
            for t in tracks
            if (t.is_dynamic or not self.cfg.only_dynamic_tracks)
            and t.distance_m is not None
            and t.bearing_deg is not None
        ]
        if not dynamic_tracks:
            return DynamicFilterResult([], scan, [])

        keep_ranges: list[float] = []
        keep_angles: list[float] = []
        removed: list[int] = []
        dynamic_obs: list[Obstacle] = []

        for i, (r, a) in enumerate(zip(scan.ranges, scan.angles)):
            import math

            bearing = math.degrees(a)
            drop = False
            for t in dynamic_tracks:
                assert t.bearing_deg is not None and t.distance_m is not None
                if (
                    _angle_diff_deg(bearing, t.bearing_deg)
                    <= self.cfg.association_bearing_deg
                    and abs(r - t.distance_m) <= self.cfg.association_range_m
                ):
                    drop = True
                    break
            if drop:
                removed.append(i)
            else:
                keep_ranges.append(r)
                keep_angles.append(a)

        for t in dynamic_tracks:
            import math

            assert t.distance_m is not None and t.bearing_deg is not None
            rad = math.radians(t.bearing_deg)
            dynamic_obs.append(
                Obstacle(
                    x=t.distance_m * math.cos(rad),
                    y=t.distance_m * math.sin(rad),
                    range_m=t.distance_m,
                    bearing_deg=t.bearing_deg,
                )
            )

        static = LidarScan(
            ranges=keep_ranges,
            angles=keep_angles,
            timestamp=scan.timestamp,
            frame_id=scan.frame_id,
            min_range=scan.min_range,
            max_range=scan.max_range,
            scan_time_s=scan.scan_time_s,
        )
        return DynamicFilterResult(dynamic_obs, static, removed)
