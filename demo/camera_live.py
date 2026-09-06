"""Live USB camera capture for the demo (real device only)."""

from __future__ import annotations

import platform
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class CameraFrame:
    bgr: np.ndarray
    fps: float
    index: int
    age_s: float = 0.0
    connected: bool = True


class LiveCamera:
    def __init__(
        self, index: int | None = None, width: int = 640, height: int = 480
    ) -> None:
        # Default: USB PS3 Eye from config/demo_hardware.yaml (not laptop webcam).
        if index is None:
            from demo.defaults import default_camera_index

            index = default_camera_index()
        self.index = index
        if platform.system() == "Windows":
            self._cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
            if not self._cap.isOpened():
                self._cap.release()
                self._cap = cv2.VideoCapture(index)
        else:
            self._cap = cv2.VideoCapture(index)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera index {index}")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, 30)
        # Reduce buffering lag on Windows DirectShow
        try:
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._frame_mono: float | None = None
        self.fps = 0.0
        self._n = 0
        self._t0 = time.monotonic()
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        # Wait briefly for first frame
        for _ in range(50):
            if self._frame is not None:
                break
            time.sleep(0.02)
        if self._frame is None:
            self.stop()
            raise RuntimeError("Camera opened but produced no frames")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self._cap.release()

    def _loop(self) -> None:
        while self._running:
            ok, frame = self._cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            with self._lock:
                self._frame = frame
                self._frame_mono = time.monotonic()
            self._n += 1
            now = time.monotonic()
            if now - self._t0 >= 1.0:
                self.fps = self._n / (now - self._t0)
                self._n = 0
                self._t0 = now

    def read(self) -> CameraFrame:
        with self._lock:
            if self._frame is None:
                raise RuntimeError("No camera frame yet")
            age_s = (
                float("inf")
                if self._frame_mono is None
                else max(0.0, time.monotonic() - self._frame_mono)
            )
            return CameraFrame(
                bgr=self._frame.copy(),
                fps=self.fps,
                index=self.index,
                age_s=age_s,
                connected=True,
            )
