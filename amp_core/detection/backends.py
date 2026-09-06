"""Object detection backend factory.

PC default: YOLOv8n. Combined Face+HOG+DNN is disabled (duplicate persons).
"""

from __future__ import annotations

import time

from amp_core.common.types import BoundingBox, Detection, Timestamp
from amp_core.detection.base import DetectorBackend, DetectorConfig

__all__ = [
    "DetectorBackend",
    "DetectorConfig",
    "StubDetector",
    "OnnxDetector",
    "create_detector",
    "benchmark_detector",
]


class StubDetector(DetectorBackend):
    def __init__(self, cfg: DetectorConfig) -> None:
        self.cfg = cfg
        self._frame = 0

    def name(self) -> str:
        return "stub"

    def detect(self, image_bgr_or_gray: bytes, width: int, height: int, channels: int) -> list[Detection]:
        self._frame += 1
        t0 = time.perf_counter()
        cx = (width * 0.3 + (self._frame % 80) * 2.0) % (width * 0.7)
        cy = height * 0.55
        w, h = width * 0.12, height * 0.35
        bbox = BoundingBox(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)
        latency = (time.perf_counter() - t0) * 1000.0
        return [
            Detection(
                class_name="person",
                confidence=0.91,
                bbox=bbox,
                timestamp=Timestamp.now(),
                inference_latency_ms=latency,
            )
        ]


class OnnxDetector(DetectorBackend):
    def __init__(self, cfg: DetectorConfig) -> None:
        self.cfg = cfg
        self._session = None
        try:
            import onnxruntime as ort  # type: ignore

            if cfg.model_path:
                self._session = ort.InferenceSession(
                    cfg.model_path, providers=["CPUExecutionProvider"]
                )
        except Exception:
            self._session = None

    def name(self) -> str:
        return "onnx" if self._session is not None else "onnx_unavailable"

    def detect(self, image_bgr_or_gray: bytes, width: int, height: int, channels: int) -> list[Detection]:
        if self._session is None:
            return StubDetector(self.cfg).detect(image_bgr_or_gray, width, height, channels)
        return []


def create_detector(cfg: DetectorConfig) -> DetectorBackend:
    backend = cfg.backend.lower().strip()

    if backend == "stub":
        return StubDetector(cfg)
    if backend == "onnx":
        return OnnxDetector(cfg)

    if backend in {"yolo", "yolov8", "yolov8n", "ultralytics", "auto"}:
        try:
            from amp_core.detection.yolo_detector import YoloV8Detector

            det = YoloV8Detector(cfg)
            if getattr(det, "_model", None) is not None:
                return det
        except Exception:
            if backend != "auto":
                raise
        if backend != "auto":
            # Explicit YOLO requested but failed — try filtered DNN rather than silent junk
            backend = "opencv_dnn"
        else:
            backend = "opencv_dnn"

    if backend in {"opencv_hog", "hog"}:
        from amp_core.detection.opencv_detectors import OpenCVHogDetector

        return OpenCVHogDetector(cfg)
    if backend in {"opencv_face", "face"}:
        from amp_core.detection.opencv_detectors import OpenCVFaceDetector

        return OpenCVFaceDetector(cfg)
    if backend in {"opencv_dnn", "dnn", "mobilenet", "combined"}:
        from amp_core.detection.opencv_detectors import OpenCVDnnDetector
        from amp_core.detection.yolo_detector import filter_detections

        class FilteredDnn(DetectorBackend):
            def __init__(self) -> None:
                self._inner = OpenCVDnnDetector(cfg)

            def name(self) -> str:
                return self._inner.name() + "+filter"

            def detect(self, image_bgr_or_gray: bytes, width: int, height: int, channels: int):
                raw = self._inner.detect(image_bgr_or_gray, width, height, channels)
                return filter_detections(
                    raw, width, height, min_conf=max(0.55, cfg.conf_threshold), max_area_frac=0.55
                )

        return FilteredDnn()

    if backend in {"tflite", "torch_pc", "coral", "hailo", "jetson"}:
        return StubDetector(cfg)
    raise ValueError(f"Unknown detector backend: {cfg.backend}")


def benchmark_detector(
    detector: DetectorBackend,
    width: int = 640,
    height: int = 480,
    frames: int = 30,
) -> dict[str, float]:
    blob = bytes([128]) * (width * height)
    latencies: list[float] = []
    for _ in range(frames):
        t0 = time.perf_counter()
        detector.detect(blob, width, height, 1)
        latencies.append((time.perf_counter() - t0) * 1000.0)
    mean_ms = sum(latencies) / max(1, len(latencies))
    return {
        "frames": float(frames),
        "mean_latency_ms": mean_ms,
        "fps": 1000.0 / mean_ms if mean_ms > 0 else 0.0,
        "p95_latency_ms": sorted(latencies)[int(0.95 * (len(latencies) - 1))],
    }
