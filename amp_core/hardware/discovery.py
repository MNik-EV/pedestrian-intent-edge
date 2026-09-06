"""Hardware discovery helpers (camera indices, serial LiDAR candidates)."""

from __future__ import annotations

import platform
from dataclasses import dataclass, asdict
from typing import Any


@dataclass
class CameraDevice:
    index: int
    width: int
    height: int
    backend: str
    ok: bool
    note: str = ""


@dataclass
class HardwareInventory:
    cameras: list[CameraDevice]
    serial_ports: list[str]
    lidar_likely: list[str]
    imu_likely: list[str]
    platform: str
    recommendations: list[str]

    def to_dict(self) -> dict[str, Any]:
        return {
            "cameras": [asdict(c) for c in self.cameras],
            "serial_ports": self.serial_ports,
            "lidar_likely": self.lidar_likely,
            "imu_likely": self.imu_likely,
            "platform": self.platform,
            "recommendations": self.recommendations,
        }

    @property
    def primary_camera_index(self) -> int | None:
        # Prefer USB PS3 Eye (OpenCV index 1 on this PC) over laptop webcam (0).
        preferred = 1
        for c in self.cameras:
            if c.ok and c.index == preferred:
                return c.index
        for c in self.cameras:
            if c.ok:
                return c.index
        return None

    @property
    def has_camera(self) -> bool:
        return self.primary_camera_index is not None

    @property
    def has_lidar(self) -> bool:
        return bool(self.lidar_likely)


def list_serial_ports() -> list[str]:
    ports: list[str] = []
    if platform.system() == "Windows":
        try:
            from serial.tools import list_ports  # type: ignore

            ports = [p.device for p in list_ports.comports()]
        except Exception:
            ports = []
    else:
        import glob

        ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    return ports


def classify_serial(ports: list[str]) -> tuple[list[str], list[str]]:
    """Heuristic: LD19 often on USB-SERIAL; IMU boards vary — keep separate lists."""
    lidar: list[str] = []
    imu: list[str] = []
    for p in ports:
        pl = p.lower()
        # Without USB descriptor parsing, treat all serial as lidar candidates on robot
        lidar.append(p)
        if "imu" in pl or "acm" in pl:
            imu.append(p)
    return lidar, imu


def probe_cameras(max_index: int = 2) -> list[CameraDevice]:
    """OpenCV probe of camera indices (Windows DirectShow / Linux V4L2)."""
    try:
        import cv2  # type: ignore
    except Exception:
        return [
            CameraDevice(
                index=-1,
                width=0,
                height=0,
                backend="none",
                ok=False,
                note="opencv not installed — pip install opencv-python",
            )
        ]

    found: list[CameraDevice] = []
    backends = []
    if platform.system() == "Windows":
        backends = [("DSHOW", getattr(cv2, "CAP_DSHOW", 700)), ("ANY", 0)]
    else:
        backends = [("V4L2", getattr(cv2, "CAP_V4L2", 200)), ("ANY", 0)]

    for idx in range(max_index):
        opened = False
        for bname, bflag in backends:
            cap = cv2.VideoCapture(idx, bflag) if bflag else cv2.VideoCapture(idx)
            if not cap.isOpened():
                cap.release()
                continue
            ok, frame = cap.read()
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
            cap.release()
            if ok and frame is not None:
                found.append(
                    CameraDevice(
                        index=idx,
                        width=w or int(frame.shape[1]),
                        height=h or int(frame.shape[0]),
                        backend=bname,
                        ok=True,
                        note="webcam/UVC",
                    )
                )
                opened = True
                break
        if not opened:
            continue
    return found


def discover_hardware(max_camera_index: int = 2) -> HardwareInventory:
    cameras = probe_cameras(max_camera_index)
    ports = list_serial_ports()
    lidar, imu = classify_serial(ports)
    recs: list[str] = []
    if any(c.ok for c in cameras):
        ok_idxs = [c.index for c in cameras if c.ok]
        prefer = 1 if 1 in ok_idxs else ok_idxs[0]
        recs.append(f"Use USB camera index {prefer} (prefer PS3 Eye over laptop webcam)")
    else:
        recs.append("No camera found — install opencv-python and check privacy permissions")
    if not lidar:
        recs.append("No LiDAR serial device — running camera-only (no fake LiDAR in live UI)")
    else:
        recs.append(f"Possible LiDAR serial: {lidar[0]}")
    return HardwareInventory(
        cameras=[c for c in cameras if c.ok] or cameras,
        serial_ports=ports,
        lidar_likely=lidar,
        imu_likely=imu,
        platform=platform.system(),
        recommendations=recs,
    )
