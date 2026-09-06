"""Live local USB-camera capture via OpenCV (bench fallback)."""

from __future__ import annotations

import time
from dataclasses import dataclass

from amp_core.common.types import ImageFrame, Timestamp


@dataclass
class WebcamConfig:
    # Used only when the bench USB fallback is explicitly selected.
    index: int = 1
    width: int = 640
    height: int = 480
    fps_request: float = 30.0


class WebcamCapture:
    """Thread-unsafe single-reader webcam. Call read() from one loop."""

    def __init__(self, cfg: WebcamConfig | None = None) -> None:
        self.cfg = cfg or WebcamConfig()
        self._cap = None
        self._fps = 0.0
        self._frames = 0
        self._t0 = time.monotonic()
        self.last_jpeg: bytes | None = None
        self.last_bgr = None
        self.available = False
        self.error: str | None = None
        self._open()

    def _open(self) -> None:
        try:
            import cv2  # type: ignore
            import platform

            idx = self.cfg.index
            if platform.system() == "Windows":
                cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
                if not cap.isOpened():
                    cap.release()
                    cap = cv2.VideoCapture(idx)
            else:
                cap = cv2.VideoCapture(idx)
            if not cap.isOpened():
                self.error = f"cannot open camera index {idx}"
                self.available = False
                return
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg.height)
            self._cap = cap
            self.available = True
            self.error = None
        except Exception as exc:  # noqa: BLE001
            self.error = str(exc)
            self.available = False

    @property
    def fps(self) -> float:
        return self._fps

    def read(self) -> ImageFrame | None:
        if self._cap is None:
            return None
        import cv2  # type: ignore

        ok, frame = self._cap.read()
        if not ok or frame is None:
            self.error = "camera read failed"
            return None
        # Resize if needed for bandwidth
        h, w = frame.shape[:2]
        if w != self.cfg.width or h != self.cfg.height:
            frame = cv2.resize(frame, (self.cfg.width, self.cfg.height))
            h, w = frame.shape[:2]
        self.last_bgr = frame
        ok_j, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        self.last_jpeg = buf.tobytes() if ok_j else None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self._frames += 1
        now = time.monotonic()
        if now - self._t0 >= 1.0:
            self._fps = self._frames / (now - self._t0)
            self._frames = 0
            self._t0 = now
        return ImageFrame(
            width=w,
            height=h,
            channels=1,
            data=gray.tobytes(),
            encoding="mono8",
            timestamp=Timestamp.now(),
            frame_id="camera",
            fps=self._fps,
        )

    def release(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
            self.available = False
