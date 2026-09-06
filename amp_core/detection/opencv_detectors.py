"""OpenCV-based detectors for PC webcam (no fake boxes).

Backends:
- opencv_face: Haar face (great for laptop webcam)
- opencv_hog: HOG+SVM person (OpenCV 4.x)
- opencv_dnn: MobileNet-SSD multi-class (auto-downloads once)
- auto/combined: face + hog + dnn merged with NMS
"""

from __future__ import annotations

import time
import urllib.request
from pathlib import Path

from amp_core.common.types import BoundingBox, Detection, Timestamp
from amp_core.detection.base import DetectorBackend, DetectorConfig

SSD_CLASSES = (
    "background",
    "aeroplane",
    "bicycle",
    "bird",
    "boat",
    "bottle",
    "bus",
    "car",
    "cat",
    "chair",
    "cow",
    "diningtable",
    "dog",
    "horse",
    "motorbike",
    "person",
    "pottedplant",
    "sheep",
    "sofa",
    "train",
    "tvmonitor",
)

PROTOTXT_URL = (
    "https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/deploy.prototxt"
)
CAFFEMODEL_URL = "https://github.com/chuanqi305/MobileNet-SSD/raw/master/mobilenet_iter_73000.caffemodel"


def ensure_mobilenet_ssd(models_dir: Path) -> tuple[Path, Path] | None:
    models_dir.mkdir(parents=True, exist_ok=True)
    prototxt = models_dir / "MobileNetSSD_deploy.prototxt"
    caffemodel = models_dir / "MobileNetSSD_deploy.caffemodel"
    try:
        if not prototxt.exists() or prototxt.stat().st_size < 1000:
            urllib.request.urlretrieve(PROTOTXT_URL, prototxt)
        if not caffemodel.exists() or caffemodel.stat().st_size < 1_000_000:
            urllib.request.urlretrieve(CAFFEMODEL_URL, caffemodel)
        if (
            prototxt.exists()
            and caffemodel.exists()
            and caffemodel.stat().st_size > 1_000_000
        ):
            return prototxt, caffemodel
    except Exception:
        return None
    return None


def _bytes_to_bgr(image: bytes, width: int, height: int, channels: int):
    import cv2
    import numpy as np

    if channels == 3 and len(image) == width * height * 3:
        return np.frombuffer(image, dtype=np.uint8).reshape((height, width, 3)).copy()
    if channels == 1 or len(image) == width * height:
        gray = np.frombuffer(image, dtype=np.uint8).reshape((height, width))
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    arr = np.frombuffer(image, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


def _iou(a: BoundingBox, b: BoundingBox) -> float:
    x1 = max(a.x1, b.x1)
    y1 = max(a.y1, b.y1)
    x2 = min(a.x2, b.x2)
    y2 = min(a.y2, b.y2)
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    if inter <= 0:
        return 0.0
    union = a.width * a.height + b.width * b.height - inter
    return inter / union if union > 0 else 0.0


def _nms(dets: list[Detection], iou_thresh: float = 0.4) -> list[Detection]:
    dets = sorted(dets, key=lambda d: d.confidence, reverse=True)
    keep: list[Detection] = []
    for d in dets:
        if all(_iou(d.bbox, k.bbox) < iou_thresh for k in keep):
            keep.append(d)
    return keep


class OpenCVFaceDetector(DetectorBackend):
    def __init__(self, cfg: DetectorConfig) -> None:
        self.cfg = cfg
        import cv2

        path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        self._face = cv2.CascadeClassifier(path)
        if self._face.empty():
            raise RuntimeError("Failed to load Haar face cascade")

    def name(self) -> str:
        return "opencv_face"

    def detect(
        self, image_bgr_or_gray: bytes, width: int, height: int, channels: int
    ) -> list[Detection]:
        import cv2

        t0 = time.perf_counter()
        bgr = _bytes_to_bgr(image_bgr_or_gray, width, height, channels)
        if bgr is None:
            return []
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        faces = self._face.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40)
        )
        latency = (time.perf_counter() - t0) * 1000.0
        out: list[Detection] = []
        for x, y, w, h in faces:
            x1 = max(0, x - int(0.3 * w))
            y1 = max(0, y - int(0.3 * h))
            x2 = min(width, x + int(1.3 * w))
            y2 = min(height, y + int(2.8 * h))
            out.append(
                Detection(
                    class_name="person",
                    confidence=0.78,
                    bbox=BoundingBox(float(x1), float(y1), float(x2), float(y2)),
                    timestamp=Timestamp.now(),
                    inference_latency_ms=latency,
                )
            )
        return out


class OpenCVHogDetector(DetectorBackend):
    def __init__(self, cfg: DetectorConfig) -> None:
        self.cfg = cfg
        import cv2

        if not hasattr(cv2, "HOGDescriptor"):
            raise RuntimeError(
                "cv2.HOGDescriptor missing — pip install 'opencv-python>=4.8,<5'"
            )
        self._hog = cv2.HOGDescriptor()
        self._hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def name(self) -> str:
        return "opencv_hog"

    def detect(
        self, image_bgr_or_gray: bytes, width: int, height: int, channels: int
    ) -> list[Detection]:
        t0 = time.perf_counter()
        bgr = _bytes_to_bgr(image_bgr_or_gray, width, height, channels)
        if bgr is None:
            return []
        rects, weights = self._hog.detectMultiScale(
            bgr, winStride=(8, 8), padding=(8, 8), scale=1.05
        )
        latency = (time.perf_counter() - t0) * 1000.0
        out: list[Detection] = []
        for (x, y, w, h), conf in zip(rects, weights):
            c = float(conf[0]) if hasattr(conf, "__len__") else float(conf)
            score = max(0.35, min(0.99, 0.5 + c / 4.0))
            if score < self.cfg.conf_threshold:
                continue
            out.append(
                Detection(
                    class_name="person",
                    confidence=score,
                    bbox=BoundingBox(float(x), float(y), float(x + w), float(y + h)),
                    timestamp=Timestamp.now(),
                    inference_latency_ms=latency,
                )
            )
        return out


class OpenCVDnnDetector(DetectorBackend):
    """MobileNet-SSD multi-class. Falls back to empty list if model unavailable."""

    def __init__(
        self, cfg: DetectorConfig, models_dir: str | Path | None = None
    ) -> None:
        self.cfg = cfg
        self._net = None
        root = (
            Path(models_dir)
            if models_dir
            else Path(__file__).resolve().parents[2] / "models"
        )
        paths = ensure_mobilenet_ssd(root)
        if paths is None:
            return
        try:
            import cv2

            self._net = cv2.dnn.readNetFromCaffe(str(paths[0]), str(paths[1]))
        except Exception:
            self._net = None

    def name(self) -> str:
        return "opencv_dnn" if self._net is not None else "opencv_dnn_unloaded"

    def detect(
        self, image_bgr_or_gray: bytes, width: int, height: int, channels: int
    ) -> list[Detection]:
        if self._net is None:
            return []
        import cv2

        t0 = time.perf_counter()
        bgr = _bytes_to_bgr(image_bgr_or_gray, width, height, channels)
        if bgr is None:
            return []
        h, w = bgr.shape[:2]
        blob = cv2.dnn.blobFromImage(bgr, 0.007843, (300, 300), 127.5)
        self._net.setInput(blob)
        detections = self._net.forward()
        latency = (time.perf_counter() - t0) * 1000.0
        out: list[Detection] = []
        for i in range(detections.shape[2]):
            conf = float(detections[0, 0, i, 2])
            if conf < self.cfg.conf_threshold:
                continue
            class_id = int(detections[0, 0, i, 1])
            if class_id <= 0 or class_id >= len(SSD_CLASSES):
                continue
            x1 = float(detections[0, 0, i, 3] * w)
            y1 = float(detections[0, 0, i, 4] * h)
            x2 = float(detections[0, 0, i, 5] * w)
            y2 = float(detections[0, 0, i, 6] * h)
            out.append(
                Detection(
                    class_name=SSD_CLASSES[class_id],
                    confidence=conf,
                    bbox=BoundingBox(x1, y1, x2, y2),
                    timestamp=Timestamp.now(),
                    inference_latency_ms=latency,
                )
            )
        return out


class CombinedWebcamDetector(DetectorBackend):
    """Face + HOG + optional MobileNet-SSD."""

    def __init__(self, cfg: DetectorConfig) -> None:
        self.cfg = cfg
        self._parts: list[DetectorBackend] = []
        for factory in (OpenCVFaceDetector, OpenCVHogDetector, OpenCVDnnDetector):
            try:
                det = factory(cfg)
                # Skip unloaded DNN
                if isinstance(det, OpenCVDnnDetector) and det._net is None:
                    continue
                self._parts.append(det)
            except Exception:
                continue
        if not self._parts:
            raise RuntimeError("No OpenCV detectors available")

    def name(self) -> str:
        return "combined[" + "+".join(p.name() for p in self._parts) + "]"

    def detect(
        self, image_bgr_or_gray: bytes, width: int, height: int, channels: int
    ) -> list[Detection]:
        merged: list[Detection] = []
        for part in self._parts:
            merged.extend(part.detect(image_bgr_or_gray, width, height, channels))
        return _nms(merged, iou_thresh=0.4)
