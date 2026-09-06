"""Real LD19 UART reader (230400 baud). No mocks.

Packet: 47 bytes, header 0x54 0x2C, 12 points × (dist_mm u16 + conf u8).
CRC8 uses LDROBOT lookup table from the official development manual.
"""

from __future__ import annotations

import math
import struct
import threading
import time
from dataclasses import dataclass, field

import serial
from serial.tools import list_ports

from amp_core.common.types import LidarScan, Timestamp

PACKET_LEN = 47
POINTS_PER_PKT = 12
BAUD = 230400

# Official LDROBOT CRC8 table (poly reflected style used by LD19 manuals)
CRC_TABLE = [
    0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
    0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07, 0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5,
    0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
    0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA,
    0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90, 0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62,
    0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
    0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D,
    0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26, 0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4,
    0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
    0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89,
    0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD, 0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F,
    0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
    0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E,
    0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45, 0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7,
    0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
    0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8,
]


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


def list_candidate_ports() -> list[str]:
    return [p.device for p in list_ports.comports()]


def auto_find_ld19_port(timeout_s: float = 2.0) -> str | None:
    """Try each serial port; return first that yields valid 0x54/0x2C packets."""
    for port in list_candidate_ports():
        try:
            ser = serial.Serial(port, BAUD, timeout=0.2)
        except Exception:
            continue
        try:
            deadline = time.monotonic() + timeout_s
            buf = b""
            while time.monotonic() < deadline:
                chunk = ser.read(256)
                if not chunk:
                    continue
                buf += chunk
                if len(buf) > 4096:
                    buf = buf[-2048:]
                i = buf.find(b"\x54\x2c")
                while i >= 0 and i + PACKET_LEN <= len(buf):
                    pkt = buf[i : i + PACKET_LEN]
                    if crc8(pkt[:-1]) == pkt[-1]:
                        ser.close()
                        return port
                    i = buf.find(b"\x54\x2c", i + 1)
        finally:
            try:
                ser.close()
            except Exception:
                pass
    return None


@dataclass
class LD19Point:
    angle_deg: float
    range_m: float
    confidence: int


@dataclass
class LD19Scan:
    points: list[LD19Point] = field(default_factory=list)
    hz: float = 0.0
    timestamp: Timestamp = field(default_factory=Timestamp.now)
    closest_m: float = float("inf")
    port: str = ""

    def to_lidar_scan(self) -> LidarScan:
        return LidarScan(
            ranges=[p.range_m for p in self.points],
            angles=[math.radians(p.angle_deg) for p in self.points],
            timestamp=self.timestamp,
            frame_id="laser",
        )


class LD19Reader:
    """Background thread assembling full 360° sweeps from LD19 packets."""

    def __init__(
        self,
        port: str | None = None,
        min_range: float = 0.05,
        max_range: float = 12.0,
        min_confidence: int = 0,
    ) -> None:
        self.port = port or auto_find_ld19_port()
        if not self.port:
            raise RuntimeError(
                "LD19 not found. Plug USB-UART adapter, install CH340/CP2102 driver, "
                f"available ports={list_candidate_ports()}"
            )
        self.min_range = min_range
        self.max_range = max_range
        self.min_confidence = min_confidence
        self._ser = serial.Serial(self.port, BAUD, timeout=0.05)
        self._lock = threading.Lock()
        self._accum: list[LD19Point] = []
        self._last_scan = LD19Scan(port=self.port)
        self._running = False
        self._thread: threading.Thread | None = None
        self._packets = 0
        self._pkt_t0 = time.monotonic()
        self._scan_count = 0
        self._scan_t0 = time.monotonic()
        self.packet_hz = 0.0
        self.scan_hz = 0.0

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="ld19", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        try:
            self._ser.close()
        except Exception:
            pass

    def get_scan(self) -> LD19Scan:
        with self._lock:
            return self._last_scan

    def _parse_packet(self, data: bytes) -> list[LD19Point]:
        # struct: skip first byte already validated; format from gist
        # <xBHH + HB*12 + HHB  but full packet includes header
        if len(data) != PACKET_LEN or data[0] != 0x54 or data[1] != 0x2C:
            return []
        if crc8(data[:-1]) != data[-1]:
            return []
        speed, start_raw = struct.unpack_from("<HH", data, 2)
        offset = 6
        dists: list[int] = []
        confs: list[int] = []
        for _ in range(POINTS_PER_PKT):
            dist, conf = struct.unpack_from("<HB", data, offset)
            dists.append(dist)
            confs.append(conf)
            offset += 3
        end_raw, timestamp = struct.unpack_from("<HH", data, offset)
        start_angle = start_raw / 100.0
        end_angle = end_raw / 100.0
        if end_angle < start_angle:
            end_angle += 360.0
        step = (end_angle - start_angle) / (POINTS_PER_PKT - 1)
        out: list[LD19Point] = []
        for i in range(POINTS_PER_PKT):
            if confs[i] < self.min_confidence:
                continue
            r = dists[i] / 1000.0
            if r < self.min_range or r > self.max_range:
                continue
            ang = (start_angle + step * i) % 360.0
            # Convert LD19 angle convention to robot frame: 0° forward, CCW positive
            # LD19 typically: 0 at forward, increasing with rotation — keep degrees as-is
            out.append(LD19Point(angle_deg=ang, range_m=r, confidence=confs[i]))
        return out

    def _finish_scan(self) -> None:
        # Deduplicate by coarse angle bin (~1°)
        bins: dict[int, LD19Point] = {}
        for p in self._accum:
            key = int(round(p.angle_deg)) % 360
            prev = bins.get(key)
            if prev is None or p.confidence >= prev.confidence:
                bins[key] = p
        points = sorted(bins.values(), key=lambda p: p.angle_deg)
        closest = min((p.range_m for p in points), default=float("inf"))
        self._scan_count += 1
        now = time.monotonic()
        if now - self._scan_t0 >= 1.0:
            self.scan_hz = self._scan_count / (now - self._scan_t0)
            self._scan_count = 0
            self._scan_t0 = now
        scan = LD19Scan(
            points=points,
            hz=self.scan_hz,
            timestamp=Timestamp.now(),
            closest_m=closest,
            port=self.port,
        )
        with self._lock:
            self._last_scan = scan
        self._accum = []

    def _loop(self) -> None:
        buf = b""
        last_angle = None
        while self._running:
            chunk = self._ser.read(512)
            if not chunk:
                continue
            buf += chunk
            if len(buf) > 8192:
                buf = buf[-4096:]
            while True:
                i = buf.find(b"\x54\x2c")
                if i < 0 or i + PACKET_LEN > len(buf):
                    break
                pkt = buf[i : i + PACKET_LEN]
                buf = buf[i + PACKET_LEN :]
                pts = self._parse_packet(pkt)
                if not pts:
                    continue
                self._packets += 1
                now = time.monotonic()
                if now - self._pkt_t0 >= 1.0:
                    self.packet_hz = self._packets / (now - self._pkt_t0)
                    self._packets = 0
                    self._pkt_t0 = now
                # Detect wrap for full revolution
                for p in pts:
                    if last_angle is not None and p.angle_deg + 30 < last_angle:
                        self._finish_scan()
                    last_angle = p.angle_deg
                    self._accum.append(p)
                if len(self._accum) > 600:
                    self._finish_scan()
