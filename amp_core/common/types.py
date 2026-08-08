"""Shared typed data structures used across AMP modules."""

from __future__ import annotations

from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Any, Optional
import time


class FusionMode(str, Enum):
    """Supported research fusion / baseline modes."""

    LIDAR_ONLY = "lidar_only"
    CAMERA_ONLY = "camera_only"
    FIXED_FUSION = "fixed_fusion"
    ADAPTIVE_FUSION = "adaptive_fusion"
    ADAPTIVE_FUSION_DYNFILTER = "adaptive_fusion_dynfilter"


class ReliabilityMode(str, Enum):
    MODE_FIXED = "MODE_FIXED"
    MODE_HEURISTIC_ADAPTIVE = "MODE_HEURISTIC_ADAPTIVE"
    MODE_LEARNED = "MODE_LEARNED"


class HealthState(str, Enum):
    OK = "OK"
    WARNING = "WARNING"
    ERROR = "ERROR"


@dataclass
class Timestamp:
    """Dual-clock timestamp for research logging and sync."""

    wall_ns: int
    mono_ns: int

    @classmethod
    def now(cls) -> "Timestamp":
        return cls(wall_ns=time.time_ns(), mono_ns=time.monotonic_ns())

    def to_dict(self) -> dict[str, int]:
        return {"wall_ns": self.wall_ns, "mono_ns": self.mono_ns}


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float

    def to_dict(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "yaw": self.yaw}


@dataclass
class Twist2D:
    linear: float
    angular: float

    def to_dict(self) -> dict[str, float]:
        return {"linear": self.linear, "angular": self.angular}


@dataclass
class Covariance2D:
    """Diagonal-ish pose covariance (x, y, yaw)."""

    xx: float
    yy: float
    yaw_yaw: float

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass
class LidarScan:
    ranges: list[float]
    angles: list[float]
    timestamp: Timestamp
    frame_id: str = "laser"
    min_range: float = 0.05
    max_range: float = 12.0
    scan_time_s: float = 0.1

    def valid_mask(self) -> list[bool]:
        return [
            self.min_range <= r <= self.max_range and r == r  # not NaN
            for r in self.ranges
        ]


@dataclass
class ImageFrame:
    width: int
    height: int
    channels: int
    data: bytes  # raw bytes; encoding described by encoding
    encoding: str  # e.g. "bgr8", "mono8", "jpeg"
    timestamp: Timestamp
    frame_id: str = "camera"
    fps: float = 0.0


@dataclass
class BoundingBox:
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def cx(self) -> float:
        return 0.5 * (self.x1 + self.x2)

    @property
    def cy(self) -> float:
        return 0.5 * (self.y1 + self.y2)

    @property
    def width(self) -> float:
        return max(0.0, self.x2 - self.x1)

    @property
    def height(self) -> float:
        return max(0.0, self.y2 - self.y1)

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass
class Detection:
    class_name: str
    confidence: float
    bbox: BoundingBox
    timestamp: Timestamp
    inference_latency_ms: float = 0.0
    track_id: Optional[int] = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "class_name": self.class_name,
            "confidence": self.confidence,
            "bbox": self.bbox.to_dict(),
            "timestamp": self.timestamp.to_dict(),
            "inference_latency_ms": self.inference_latency_ms,
            "track_id": self.track_id,
        }


@dataclass
class TrackedObject:
    track_id: int
    class_name: str
    confidence: float
    bbox: BoundingBox
    distance_m: Optional[float]
    bearing_deg: Optional[float]
    velocity_mps: float
    is_dynamic: bool
    age: int
    hits: int
    last_seen: Timestamp
    trajectory: list[tuple[float, float]] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "track_id": self.track_id,
            "class_name": self.class_name,
            "confidence": self.confidence,
            "bbox": self.bbox.to_dict(),
            "distance_m": self.distance_m,
            "bearing_deg": self.bearing_deg,
            "velocity_mps": self.velocity_mps,
            "is_dynamic": self.is_dynamic,
            "age": self.age,
            "hits": self.hits,
            "last_seen": self.last_seen.to_dict(),
            "trajectory": self.trajectory[-50:],
        }


@dataclass
class SensorConfidence:
    lidar: float
    camera: float
    imu: float
    odom: float
    timestamp: Timestamp
    features: dict[str, float] = field(default_factory=dict)

    def clamped(self) -> "SensorConfidence":
        def c(v: float) -> float:
            return max(0.0, min(1.0, float(v)))

        return SensorConfidence(
            lidar=c(self.lidar),
            camera=c(self.camera),
            imu=c(self.imu),
            odom=c(self.odom),
            timestamp=self.timestamp,
            features=dict(self.features),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "lidar": self.lidar,
            "camera": self.camera,
            "imu": self.imu,
            "odom": self.odom,
            "timestamp": self.timestamp.to_dict(),
            "features": self.features,
        }


@dataclass
class SectorDistances:
    """Eight sector minimum ranges in meters."""

    front: float
    front_left: float
    front_right: float
    left: float
    right: float
    rear_left: float
    rear: float
    rear_right: float
    timestamp: Timestamp

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["timestamp"] = self.timestamp.to_dict()
        return d

    def minimum(self) -> float:
        vals = [
            self.front,
            self.front_left,
            self.front_right,
            self.left,
            self.right,
            self.rear_left,
            self.rear,
            self.rear_right,
        ]
        return min(vals)


@dataclass
class FusionState:
    pose: Pose2D
    twist: Twist2D
    covariance: Covariance2D
    sensor_weights: dict[str, float]
    innovation: dict[str, float]
    update_rate_hz: float
    mode: FusionMode
    timestamp: Timestamp

    def to_dict(self) -> dict[str, Any]:
        return {
            "pose": self.pose.to_dict(),
            "twist": self.twist.to_dict(),
            "covariance": self.covariance.to_dict(),
            "sensor_weights": self.sensor_weights,
            "innovation": self.innovation,
            "update_rate_hz": self.update_rate_hz,
            "mode": self.mode.value,
            "timestamp": self.timestamp.to_dict(),
        }


@dataclass
class SystemStats:
    cpu_percent: float
    ram_percent: float
    temperature_c: Optional[float]
    disk_percent: float
    network_latency_ms: Optional[float]
    camera_fps: float
    lidar_fps: float
    detection_fps: float
    slam_hz: float
    fusion_hz: float
    websocket_latency_ms: Optional[float]
    timestamp: Timestamp

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["timestamp"] = self.timestamp.to_dict()
        return d
