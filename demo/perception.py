"""Live perception: YOLO + bearing-gated LiDAR–camera distance fusion."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

from amp_core.calibration.distance_fusion import fuse_detections_distances
from amp_core.calibration.transforms import (
    CameraIntrinsics,
    ExtrinsicTransform,
    lidar_polar_to_camera,
)
from amp_core.detection.backends import DetectorConfig, create_detector
from demo.camera_source import open_camera_source
from demo.defaults import DEFAULT_EXTRINSICS_PATH, DEFAULT_INTRINSICS_PATH
from demo.ld19_live import LD19Reader


def load_intrinsics(path: Path) -> CameraIntrinsics:
    if not path.exists():
        raise FileNotFoundError(f"Missing camera calibration: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return CameraIntrinsics(
        fx=float(data["fx"]),
        fy=float(data["fy"]),
        cx=float(data["cx"]),
        cy=float(data["cy"]),
        width=int(data.get("image_width", 640)),
        height=int(data.get("image_height", 480)),
        dist_coeffs=tuple(float(x) for x in data.get("distortion", [0, 0, 0, 0, 0])),
    )


def load_extrinsics(path: Path) -> ExtrinsicTransform:
    """Load practical extrinsics.

    IMPORTANT: uses lidar_to_camera_optical() so 2D scan points get Z>0 in the
    camera frame (identity RPY previously projected NOTHING).
    """
    if not path.exists():
        raise FileNotFoundError(f"Missing camera–LiDAR calibration: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return ExtrinsicTransform.lidar_to_camera_optical(
        tx=float(data.get("x", data.get("tx", 0.0))),
        ty=float(data.get("y", data.get("ty", 0.05))),
        tz=float(data.get("z", data.get("tz", 0.0))),
        roll=math.radians(float(data.get("roll_deg", data.get("roll", 0.0)))),
        pitch=math.radians(float(data.get("pitch_deg", data.get("pitch", 0.0)))),
        yaw=math.radians(float(data.get("yaw_deg", data.get("yaw", 0.0)))),
    )


def color_by_range(r: float, rmax: float = 4.0) -> tuple[int, int, int]:
    t = max(0.0, min(1.0, r / rmax))
    return (int(255 * t), int(80), int(255 * (1 - t)))


def undistort_bgr(bgr: np.ndarray, K: CameraIntrinsics) -> np.ndarray:
    """Return an image matching the pinhole model used by projection."""
    if not any(abs(v) > 1e-12 for v in K.dist_coeffs):
        return bgr
    camera_matrix = np.array(
        [[K.fx, 0.0, K.cx], [0.0, K.fy, K.cy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    return cv2.undistort(
        bgr,
        camera_matrix,
        np.asarray(K.dist_coeffs, dtype=np.float64),
        None,
        camera_matrix,
    )


@dataclass
class DemoObject:
    class_name: str
    confidence: float
    bbox: Any
    distance_m: float | None
    bearing_deg: float
    lidar_points_in_box: int
    fusion_method: str = ""
    fusion_conf: float = 0.0
    mono_prior_m: float | None = None


@dataclass
class DemoSnapshot:
    jpeg: bytes
    objects: list[DemoObject]
    lidar_xy: list[tuple[float, float, float]]
    closest_m: float | None
    camera_fps: float
    lidar_hz: float
    detect_ms: float
    projected_count: int
    calib_notes: list[str] = field(default_factory=list)

    def objects_dict(self) -> list[dict[str, Any]]:
        out = []
        for o in self.objects:
            out.append(
                {
                    "class": o.class_name,
                    "confidence": round(o.confidence, 3),
                    "distance_m": None
                    if o.distance_m is None
                    else round(o.distance_m, 3),
                    "bearing_deg": round(o.bearing_deg, 2),
                    "lidar_points": o.lidar_points_in_box,
                    "fusion": o.fusion_method,
                    "fusion_conf": round(o.fusion_conf, 2),
                    "mono_prior_m": None
                    if o.mono_prior_m is None
                    else round(o.mono_prior_m, 2),
                    "bbox": {
                        "x1": o.bbox.x1,
                        "y1": o.bbox.y1,
                        "x2": o.bbox.x2,
                        "y2": o.bbox.y2,
                    },
                }
            )
        return out


class DemoPerception:
    """Real camera + LD19 + YOLO + bearing-gated LiDAR distance fusion."""

    def __init__(
        self,
        camera_index: int | None = None,
        camera_url: str | None = None,
        lidar_port: str | None = None,
        intrinsics_path: str = DEFAULT_INTRINSICS_PATH,
        extrinsics_path: str = DEFAULT_EXTRINSICS_PATH,
    ) -> None:
        self.notes: list[str] = []
        ip = Path(intrinsics_path)
        ep = Path(extrinsics_path)
        missing = [str(path) for path in (ip, ep) if not path.exists()]
        if missing:
            raise RuntimeError(
                "Final IMX219 calibration is required before fused ranging. Missing: "
                + ", ".join(missing)
                + ". Run demo/calibrate_intrinsics.py and then "
                "demo/auto_calibrate_extrinsics.py; legacy PS3 Eye values are invalid."
            )
        self.notes.append("IMX219 intrinsics: loaded")
        self.notes.append("IMX219↔LD19 extrinsics: loaded")
        self.notes.append(
            "distance: bearing-gated LiDAR fusion (+ person height prior)"
        )

        self.K = load_intrinsics(ip)
        if any(abs(v) > 1e-12 for v in self.K.dist_coeffs):
            self.notes.append("lens distortion: corrected before detection/projection")
        self.ext = load_extrinsics(ep)
        self.camera = open_camera_source(
            camera_index=camera_index,
            camera_url=camera_url,
            width=self.K.width,
            height=self.K.height,
        )
        self.notes.append(f"camera source: {type(self.camera).__name__}")
        self.lidar = LD19Reader(port=lidar_port)
        self.detector = create_detector(
            DetectorConfig(backend="auto", conf_threshold=0.40, model_path="yolov8n.pt")
        )
        self.camera.start()
        self.lidar.start()
        self._ema: dict[int, float] = {}

    def close(self) -> None:
        self.camera.stop()
        self.lidar.stop()

    def _smooth(self, key: int, dist: float | None) -> float | None:
        if dist is None:
            return None
        prev = self._ema.get(key)
        if prev is None:
            self._ema[key] = dist
            return dist
        if prev > 1.2 and dist < prev * 0.55:
            dist = 0.75 * prev + 0.25 * dist
        sm = 0.65 * prev + 0.35 * dist
        self._ema[key] = sm
        return sm

    def step(self) -> DemoSnapshot:
        fr = self.camera.read()
        sc = self.lidar.get_scan()
        h, w = fr.bgr.shape[:2]
        K = self.K.scaled_to(w, h)
        bgr = undistort_bgr(fr.bgr, K)

        t0 = time.perf_counter()
        dets = self.detector.detect(bgr.tobytes(), w, h, 3)
        detect_ms = (time.perf_counter() - t0) * 1000.0

        ranges = [p.range_m for p in sc.points]
        angles = [math.radians(p.angle_deg) for p in sc.points]
        projected = lidar_polar_to_camera(ranges, angles, self.ext, K, z_plane=0.0)

        fused = fuse_detections_distances(
            [(d.bbox, d.class_name, float(d.confidence)) for d in dets],
            ranges,
            angles,
            self.ext,
            K,
        )

        vis = bgr.copy()
        for u, v, r in projected:
            if r <= 4.5:
                cv2.circle(vis, (int(u), int(v)), 2, color_by_range(r), -1)

        objects: list[DemoObject] = []
        for d, f in zip(dets, fused):
            key = int(d.bbox.cx // 40) * 1000 + int(d.bbox.cy // 60)
            dist = self._smooth(key, f.distance_m)
            objects.append(
                DemoObject(
                    class_name=d.class_name,
                    confidence=d.confidence,
                    bbox=d.bbox,
                    distance_m=dist,
                    bearing_deg=f.bearing_deg,
                    lidar_points_in_box=f.n_points,
                    fusion_method=f.method,
                    fusion_conf=f.confidence,
                    mono_prior_m=f.mono_prior_m,
                )
            )
            x1, y1, x2, y2 = map(int, (d.bbox.x1, d.bbox.y1, d.bbox.x2, d.bbox.y2))
            ok = dist is not None and f.n_points >= 3
            color = (0, 220, 120) if ok else (0, 160, 255)
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            if dist is not None:
                label = f"{d.class_name}  {dist:.2f} m"
                if f.mono_prior_m is not None:
                    label += f"  (mono~{f.mono_prior_m:.1f})"
            else:
                label = f"{d.class_name}  no-lidar"
            cv2.putText(
                vis,
                label,
                (x1, max(18, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (245, 245, 245),
                2,
            )
            cx = int(d.bbox.cx)
            cv2.line(vis, (cx, y2), (cx, min(h - 1, y2 + 18)), color, 2)

        closest = None if sc.closest_m > 1e8 else sc.closest_m
        cv2.putText(
            vis,
            f"fusion=bearing+cluster  cam={fr.fps:.1f}fps  lidar={sc.hz:.1f}Hz  {self.detector.name()}",
            (10, h - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 255, 200),
            1,
        )

        ok, buf = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        jpeg = buf.tobytes() if ok else b""
        lidar_xy = [
            (
                p.range_m * math.cos(math.radians(p.angle_deg)),
                p.range_m * math.sin(math.radians(p.angle_deg)),
                p.range_m,
            )
            for p in sc.points[::2]
            if p.range_m <= 4.5
        ]
        return DemoSnapshot(
            jpeg=jpeg,
            objects=objects,
            lidar_xy=lidar_xy,
            closest_m=closest,
            camera_fps=fr.fps,
            lidar_hz=sc.hz,
            detect_ms=detect_ms,
            projected_count=len(projected),
            calib_notes=list(self.notes),
        )
