"""MJPEG-over-HTTP camera client for a networked edge camera (Raspberry Pi Zero 2W).

Reads the multipart JPEG stream produced by edge/camera_streamer.py (or any
other MJPEG-over-HTTP source) using only the standard library on the client
side, decodes frames with OpenCV, and exposes the same small interface as
demo/camera_live.py's LiveCamera (start/stop/read/.fps) so callers can treat
a Wi-Fi-streamed Pi camera and a directly-attached USB camera identically.

Frames are located by scanning for JPEG SOI/EOI markers rather than parsing
the multipart boundary/Content-Length headers, which works against the
bundled streamer and most other MJPEG servers without extra assumptions.
Wi-Fi is far less reliable than a wired USB camera, so drops are expected and
handled with automatic reconnect rather than raising.
"""

from __future__ import annotations

import threading
import time
import urllib.request
from dataclasses import dataclass

import cv2
import numpy as np

SOI = b"\xff\xd8"
EOI = b"\xff\xd9"
MAX_BUFFER_BYTES = 2_000_000


@dataclass
class CameraFrame:
    bgr: np.ndarray
    fps: float
    index: int = -1  # not applicable to a network source; kept for interface parity
    age_s: float = 0.0
    connected: bool = False


class NetworkCameraCapture:
    """Background-thread MJPEG/HTTP client with automatic reconnect."""

    def __init__(
        self,
        url: str,
        reconnect_timeout_s: float = 3.0,
        connect_timeout_s: float = 5.0,
        first_frame_timeout_s: float = 6.0,
    ) -> None:
        self.url = url
        self.reconnect_timeout_s = reconnect_timeout_s
        self.connect_timeout_s = connect_timeout_s
        self.first_frame_timeout_s = first_frame_timeout_s
        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._frame_mono: float | None = None
        self.fps = 0.0
        self.connected = False
        self.last_error: str | None = None
        self._n = 0
        self._t0 = time.monotonic()
        self._running = False
        self._thread: threading.Thread | None = None
        self._first_frame = threading.Event()

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._first_frame.clear()
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="net-camera"
        )
        self._thread.start()
        if not self._first_frame.wait(timeout=self.first_frame_timeout_s):
            self.stop()
            raise RuntimeError(
                f"No frame received from {self.url} yet (last_error={self.last_error!r}); "
                "check the Pi is powered on and streaming."
            )

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self.connected = False

    def read(self) -> CameraFrame:
        with self._lock:
            if self._frame is None:
                raise RuntimeError(f"No camera frame yet from {self.url}")
            age_s = (
                float("inf")
                if self._frame_mono is None
                else max(0.0, time.monotonic() - self._frame_mono)
            )
            return CameraFrame(
                bgr=self._frame.copy(),
                fps=self.fps,
                age_s=age_s,
                connected=self.connected,
            )

    def _mark_frame(self, jpg: bytes) -> None:
        img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        with self._lock:
            self._frame = img
            self._frame_mono = time.monotonic()
        self._first_frame.set()
        self._n += 1
        now = time.monotonic()
        if now - self._t0 >= 1.0:
            self.fps = self._n / (now - self._t0)
            self._n = 0
            self._t0 = now

    def _loop(self) -> None:
        while self._running:
            resp = None
            try:
                req = urllib.request.Request(
                    self.url, headers={"User-Agent": "amp-network-camera"}
                )
                resp = urllib.request.urlopen(req, timeout=self.connect_timeout_s)
                self.connected = True
                self.last_error = None
                buf = b""
                while self._running:
                    chunk = resp.read(8192)
                    if not chunk:
                        break
                    buf += chunk
                    if len(buf) > MAX_BUFFER_BYTES:
                        buf = buf[-(MAX_BUFFER_BYTES // 2) :]
                    # Drain every complete frame already buffered so we always
                    # display the freshest one instead of falling behind.
                    while True:
                        start = buf.find(SOI)
                        if start < 0:
                            break
                        end = buf.find(EOI, start + 2)
                        if end < 0:
                            buf = buf[start:]  # keep partial frame for next chunk
                            break
                        self._mark_frame(buf[start : end + 2])
                        buf = buf[end + 2 :]
            except Exception as exc:  # noqa: BLE001 - network is inherently flaky
                self.connected = False
                self.last_error = str(exc)
            finally:
                self.connected = False
                if resp is not None:
                    try:
                        resp.close()
                    except Exception:
                        pass
            if self._running:
                time.sleep(self.reconnect_timeout_s)
