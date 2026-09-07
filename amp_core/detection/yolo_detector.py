"""YOLOv8 detector backend for high-quality PC webcam detection."""

from __future__ import annotations

import time
from pathlib import Path

from amp_core.common.types import BoundingBox, Detection, Timestamp
from amp_core.detection.base import DetectorBackend, DetectorConfig
from amp_core.detection.opencv_detectors import _bytes_to_bgr, _nms


# Useful indoor / robotics classes from COCO
DEFAULT_ALLOW = frozenset(
    {
        "person",
        "bicycle",
        "car",
        "motorcycle",
        "bus",
        "truck",
        "bird",
        "cat",
        "dog",
        "backpack",
        "umbrella",
        "handbag",
        "suitcase",
        "bottle",
        "wine glass",
        "cup",
        "fork",
        "knife",
        "spoon",
        "bowl",
        "banana",
        "apple",
        "sandwich",
        "orange",
        "broccoli",
        "carrot",
        "hot dog",
        "pizza",
        "donut",
        "cake",
        "chair",
        "couch",
        "potted plant",
        "bed",
        "dining table",
        "toilet",
        "tv",
        "laptop",
        "mouse",
        "remote",
        "keyboard",
        "cell phone",
        "microwave",
        "oven",
        "toaster",
        "sink",
        "refrigerator",
        "book",
        "clock",
        "vase",
        "scissors",
        "teddy bear",
        "hair drier",
        "toothbrush",
    }
)


def filter_detections(
    dets: list[Detection],
    frame_w: int,
    frame_h: int,
    *,
    min_conf: float = 0.45,
    min_side_px: float = 24.0,
    max_area_frac: float = 0.92,
    min_area_frac: float = 0.002,
) -> list[Detection]:
    """Drop tiny/absurd/low-confidence boxes; allow large person boxes (webcam selfie)."""
    frame_area = max(1.0, float(frame_w * frame_h))
    kept: list[Detection] = []
    for d in dets:
        if d.confidence < min_conf:
            continue
        w, h = d.bbox.width, d.bbox.height
        if w < min_side_px or h < min_side_px:
            continue
        area_frac = (w * h) / frame_area
        # Only reject near-full-frame junk (often HOG/DNN false positives)
        limit = 0.95 if d.class_name == "person" else max_area_frac
        if area_frac > limit or area_frac < min_area_frac:
            continue
        x1 = max(0.0, min(float(frame_w - 1), d.bbox.x1))
        y1 = max(0.0, min(float(frame_h - 1), d.bbox.y1))
        x2 = max(0.0, min(float(frame_w), d.bbox.x2))
        y2 = max(0.0, min(float(frame_h), d.bbox.y2))
        if x2 - x1 < min_side_px or y2 - y1 < min_side_px:
            continue
        kept.append(
            Detection(
                class_name=d.class_name,
                confidence=d.confidence,
                bbox=BoundingBox(x1, y1, x2, y2),
                timestamp=d.timestamp,
                inference_latency_ms=d.inference_latency_ms,
                track_id=d.track_id,
                keypoints=d.keypoints,
            )
        )
    by_class: dict[str, list[Detection]] = {}
    for d in kept:
        by_class.setdefault(d.class_name, []).append(d)
    out: list[Detection] = []
    for cls, group in by_class.items():
        # Aggressive person NMS — one box per person cluster
        iou = 0.30 if cls == "person" else 0.45
        out.extend(_nms(group, iou_thresh=iou))
    # Extra pass: if multiple persons heavily overlap, keep highest conf only
    persons = [d for d in out if d.class_name == "person"]
    others = [d for d in out if d.class_name != "person"]
    persons = _nms(persons, iou_thresh=0.25)
    return others + persons


class YoloV8Detector(DetectorBackend):
    """Ultralytics YOLOv8n — primary PC detector."""

    def __init__(self, cfg: DetectorConfig) -> None:
        self.cfg = cfg
        self._model = None
        self._error: str | None = None
        model_name = cfg.model_path or "yolov8n.pt"
        # Prefer project models/ folder
        local = Path(__file__).resolve().parents[2] / "models" / Path(model_name).name
        weights = str(local) if local.exists() else model_name
        try:
            from ultralytics import YOLO  # type: ignore

            self._model = YOLO(weights)
            # Warmup with blank image so first real frame is faster
            import numpy as np

            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            self._model.predict(blank, verbose=False, conf=0.5)
            # If downloaded to CWD, copy hint path into models/
            try:
                local.parent.mkdir(parents=True, exist_ok=True)
            except Exception:
                pass
        except Exception as exc:  # noqa: BLE001
            self._error = str(exc)
            self._model = None

    def name(self) -> str:
        if self._model is None:
            return f"yolo_unavailable:{self._error}"
        return Path(self.cfg.model_path or "yolov8n.pt").stem

    def detect(
        self, image_bgr_or_gray: bytes, width: int, height: int, channels: int
    ) -> list[Detection]:
        if self._model is None:
            return []

        t0 = time.perf_counter()
        bgr = _bytes_to_bgr(image_bgr_or_gray, width, height, channels)
        if bgr is None:
            return []
        # Ultralytics expects RGB or BGR numpy; pass BGR ndarray
        results = self._model.predict(
            bgr,
            verbose=False,
            conf=max(0.45, self.cfg.conf_threshold),
            iou=max(0.45, self.cfg.iou_threshold),
            imgsz=640,
            device=self.cfg.device
            if self.cfg.device in {"cpu", "0", "cuda"}
            else "cpu",
        )
        latency = (time.perf_counter() - t0) * 1000.0
        out: list[Detection] = []
        if not results:
            return out
        r0 = results[0]
        names = r0.names
        if r0.boxes is None:
            return out
        kpts_data = r0.keypoints.data if r0.keypoints is not None else None
        for i, box in enumerate(r0.boxes):
            cls_id = int(box.cls.item())
            conf = float(box.conf.item())
            class_name = str(names.get(cls_id, cls_id))
            if class_name not in DEFAULT_ALLOW:
                continue
            xyxy = box.xyxy[0].tolist()
            keypoints = None
            if kpts_data is not None:
                keypoints = [tuple(map(float, kp)) for kp in kpts_data[i].tolist()]
            out.append(
                Detection(
                    class_name=class_name,
                    confidence=conf,
                    bbox=BoundingBox(
                        float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3])
                    ),
                    timestamp=Timestamp.now(),
                    inference_latency_ms=latency,
                    keypoints=keypoints,
                )
            )
        return filter_detections(
            out,
            width,
            height,
            min_conf=max(0.45, self.cfg.conf_threshold),
            max_area_frac=0.92,
        )
