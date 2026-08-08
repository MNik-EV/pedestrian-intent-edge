"""Object detection backend abstraction.

Pi = lightweight real-time backend; PC = heavy research backend.
Never hard-code a single model; backends are swappable and benchmarkable.
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Sequence

from amp_core.common.types import BoundingBox, Detection, Timestamp


@dataclass
class DetectorConfig:
    backend: str = "stub"  # stub | onnx | tflite | torch_pc
    model_path: str = ""
    conf_threshold: float = 0.35
    iou_threshold: float = 0.45
    input_width: int = 320
    input_height: int = 320
    class_names: tuple[str, ...] = ("person", "chair", "bottle", "laptop")
    device: str = "cpu"  # cpu | cuda | coral | hailo | jetson


class DetectorBackend(ABC):
    """Inference backend interface (supports future accelerators)."""

    @abstractmethod
    def name(self) -> str:
        raise NotImplementedError

    @abstractmethod
    def detect(self, image_bgr_or_gray: bytes, width: int, height: int, channels: int) -> list[Detection]:
        raise NotImplementedError

    def warmup(self) -> None:
        return None


class StubDetector(DetectorBackend):
    """Deterministic mock detector for PC tests without a model."""

    def __init__(self, cfg: DetectorConfig) -> None:
        self.cfg = cfg
        self._frame = 0

    def name(self) -> str:
        return "stub"

    def detect(self, image_bgr_or_gray: bytes, width: int, height: int, channels: int) -> list[Detection]:
        self._frame += 1
        t0 = time.perf_counter()
        # Place a synthetic person that slowly moves horizontally
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
    """ONNX Runtime backend placeholder — loads when onnxruntime + model exist."""

    def __init__(self, cfg: DetectorConfig) -> None:
        self.cfg = cfg
        self._session = None
        self._load_error: str | None = None
        try:
            import onnxruntime as ort  # type: ignore

            if not cfg.model_path:
                self._load_error = "model_path empty"
            else:
                self._session = ort.InferenceSession(
                    cfg.model_path, providers=["CPUExecutionProvider"]
                )
        except Exception as exc:  # noqa: BLE001 — graceful fallback
            self._load_error = str(exc)
            self._session = None

    def name(self) -> str:
        return "onnx" if self._session is not None else "onnx_unavailable"

    def detect(self, image_bgr_or_gray: bytes, width: int, height: int, channels: int) -> list[Detection]:
        if self._session is None:
            # Fall back to stub so the stack remains operable
            return StubDetector(self.cfg).detect(image_bgr_or_gray, width, height, channels)
        # Real preprocessing/postprocessing is model-specific; documented as
        # requiring a calibrated ONNX export. Until a model is provided, return [].
        return []


def create_detector(cfg: DetectorConfig) -> DetectorBackend:
    backend = cfg.backend.lower()
    if backend == "stub":
        return StubDetector(cfg)
    if backend == "onnx":
        return OnnxDetector(cfg)
    if backend in {"tflite", "torch_pc", "coral", "hailo", "jetson"}:
        # Interface reserved for future accelerators / PC heavy models
        return StubDetector(cfg)
    raise ValueError(f"Unknown detector backend: {cfg.backend}")


def benchmark_detector(
    detector: DetectorBackend,
    width: int = 640,
    height: int = 480,
    frames: int = 30,
) -> dict[str, float]:
    """Simple CPU-side latency/FPS benchmark for candidate models."""
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
