"""LD19 serial protocol helper + mock-capable scan publisher logic (ROS-agnostic).

Physical validation on Raspberry Pi 5:
  1. Connect LD19 USB serial
  2. Confirm device via scripts/discover_hardware.py
  3. Set config/lidar.yaml port
  4. Launch lidar_driver on ROS2 Jazzy
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass

from amp_core.common.types import LidarScan, Timestamp


@dataclass
class LD19Config:
    port: str = "/dev/ttyUSB0"
    baudrate: int = 230400
    frame_id: str = "laser"
    min_range: float = 0.12
    max_range: float = 12.0


class LD19Parser:
    """Parse LD19-style packed points into ranges/angles.

    Protocol note: Exact CRC/packet layout can vary by firmware revision.
    This parser accepts a list of (angle_deg, distance_mm, intensity) samples
    produced by a low-level serial reader. Hardware bring-up should verify
    against manufacturer documentation and an oscilloscope/logic capture if needed.
    """

    def __init__(self, cfg: LD19Config | None = None) -> None:
        self.cfg = cfg or LD19Config()

    def from_points(self, points: list[tuple[float, float, int]]) -> LidarScan:
        ranges: list[float] = []
        angles: list[float] = []
        for angle_deg, distance_mm, _intensity in points:
            r = distance_mm / 1000.0
            if r < self.cfg.min_range or r > self.cfg.max_range:
                continue
            a = math.radians(angle_deg)
            # normalize
            a = (a + math.pi) % (2 * math.pi) - math.pi
            ranges.append(r)
            angles.append(a)
        return LidarScan(
            ranges=ranges,
            angles=angles,
            timestamp=Timestamp.now(),
            frame_id=self.cfg.frame_id,
            min_range=self.cfg.min_range,
            max_range=self.cfg.max_range,
        )

    @staticmethod
    def crc8_maxim(data: bytes) -> int:
        """Common CRC8 used by several LDLidar firmwares (verify on hardware)."""
        crc = 0
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x31) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc
