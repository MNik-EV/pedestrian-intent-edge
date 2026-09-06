"""Controlled sensor degradation for research failure-injection experiments.

All degradation is synthetic, labeled, and never presented as real-world noise.
"""

from __future__ import annotations

import random
from dataclasses import dataclass

from amp_core.common.types import ImageFrame, LidarScan, Timestamp


@dataclass
class DegradationConfig:
    label: str = "none"
    camera_drop_prob: float = 0.0
    camera_brightness_scale: float = 1.0
    camera_blur_box: int = 1  # 1 = none
    lidar_drop_prob: float = 0.0
    lidar_noise_std: float = 0.0
    lidar_packet_drop_prob: float = 0.0
    timestamp_jitter_ms: float = 0.0
    seed: int = 0


class DegradationEngine:
    """Apply labeled synthetic degradations for adaptive-fusion evaluation."""

    def __init__(self, cfg: DegradationConfig | None = None) -> None:
        self.cfg = cfg or DegradationConfig()
        self.rng = random.Random(self.cfg.seed)

    def maybe_drop_camera_frame(self) -> bool:
        return self.rng.random() < self.cfg.camera_drop_prob

    def degrade_image(self, frame: ImageFrame) -> ImageFrame | None:
        if self.maybe_drop_camera_frame():
            return None
        data = bytearray(frame.data)
        scale = self.cfg.camera_brightness_scale
        if abs(scale - 1.0) > 1e-6:
            for i in range(len(data)):
                data[i] = max(0, min(255, int(data[i] * scale)))
        # Box blur approximation on mono for odd kernels >= 3
        k = self.cfg.camera_blur_box
        if k >= 3 and frame.encoding in {"mono8", "gray"} and frame.channels == 1:
            data = self._box_blur_mono(bytes(data), frame.width, frame.height, k)
        else:
            data = bytes(data)
        ts = frame.timestamp
        if self.cfg.timestamp_jitter_ms > 0:
            jitter = int(
                self.rng.uniform(
                    -self.cfg.timestamp_jitter_ms, self.cfg.timestamp_jitter_ms
                )
                * 1e6
            )
            ts = Timestamp(wall_ns=ts.wall_ns + jitter, mono_ns=ts.mono_ns + jitter)
        return ImageFrame(
            width=frame.width,
            height=frame.height,
            channels=frame.channels,
            data=data if isinstance(data, bytes) else bytes(data),
            encoding=frame.encoding,
            timestamp=ts,
            frame_id=frame.frame_id,
            fps=frame.fps,
        )

    def degrade_scan(self, scan: LidarScan) -> LidarScan | None:
        if self.rng.random() < self.cfg.lidar_packet_drop_prob:
            return None
        ranges: list[float] = []
        angles: list[float] = []
        for r, a in zip(scan.ranges, scan.angles):
            if self.rng.random() < self.cfg.lidar_drop_prob:
                continue
            noise = (
                self.rng.gauss(0.0, self.cfg.lidar_noise_std)
                if self.cfg.lidar_noise_std > 0
                else 0.0
            )
            ranges.append(max(0.0, r + noise))
            angles.append(a)
        ts = scan.timestamp
        if self.cfg.timestamp_jitter_ms > 0:
            jitter = int(
                self.rng.uniform(
                    -self.cfg.timestamp_jitter_ms, self.cfg.timestamp_jitter_ms
                )
                * 1e6
            )
            ts = Timestamp(wall_ns=ts.wall_ns + jitter, mono_ns=ts.mono_ns + jitter)
        return LidarScan(
            ranges=ranges,
            angles=angles,
            timestamp=ts,
            frame_id=scan.frame_id,
            min_range=scan.min_range,
            max_range=scan.max_range,
            scan_time_s=scan.scan_time_s,
        )

    @staticmethod
    def _box_blur_mono(data: bytes, w: int, h: int, k: int) -> bytes:
        r = k // 2
        out = bytearray(len(data))
        for y in range(h):
            for x in range(w):
                acc = 0
                cnt = 0
                for dy in range(-r, r + 1):
                    yy = min(h - 1, max(0, y + dy))
                    for dx in range(-r, r + 1):
                        xx = min(w - 1, max(0, x + dx))
                        acc += data[yy * w + xx]
                        cnt += 1
                out[y * w + x] = acc // max(1, cnt)
        return bytes(out)


# Named scenario presets for the benchmark suite
SCENARIO_DEGRADATIONS: dict[str, DegradationConfig] = {
    "normal": DegradationConfig(label="scenario1_normal"),
    "low_light": DegradationConfig(
        label="scenario2_low_light", camera_brightness_scale=0.35
    ),
    "texture_poor": DegradationConfig(
        label="scenario3_texture_poor", camera_blur_box=5
    ),
    "lidar_degraded": DegradationConfig(
        label="scenario9_lidar_degradation", lidar_drop_prob=0.25, lidar_noise_std=0.05
    ),
    "camera_degraded": DegradationConfig(
        label="scenario10_camera_degradation",
        camera_drop_prob=0.2,
        camera_brightness_scale=0.5,
        camera_blur_box=7,
    ),
    "combined_degraded": DegradationConfig(
        label="scenario11_combined",
        camera_drop_prob=0.15,
        camera_brightness_scale=0.45,
        camera_blur_box=5,
        lidar_drop_prob=0.2,
        lidar_noise_std=0.04,
        timestamp_jitter_ms=5.0,
    ),
}
