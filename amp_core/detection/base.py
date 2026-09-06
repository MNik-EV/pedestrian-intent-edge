"""Detector interface and config (shared, import-cycle safe)."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from amp_core.common.types import Detection


@dataclass
class DetectorConfig:
    backend: str = "auto"
    model_path: str = ""
    conf_threshold: float = 0.50
    iou_threshold: float = 0.50
    input_width: int = 640
    input_height: int = 640
    class_names: tuple[str, ...] = ("person", "chair", "bottle", "laptop")
    device: str = "cpu"


class DetectorBackend(ABC):
    @abstractmethod
    def name(self) -> str:
        raise NotImplementedError

    @abstractmethod
    def detect(
        self, image_bgr_or_gray: bytes, width: int, height: int, channels: int
    ) -> list[Detection]:
        raise NotImplementedError

    def warmup(self) -> None:
        return None
