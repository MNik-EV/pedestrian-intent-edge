"""Geometric distance fallback when LiDAR association is unavailable."""

from __future__ import annotations

from amp_core.common.types import BoundingBox

CLASS_HEIGHT_M: dict[str, float] = {
    "person": 1.70,
    "face": 0.25,
    "chair": 0.90,
    "bottle": 0.28,
    "cup": 0.12,
    "laptop": 0.25,
    "tvmonitor": 0.45,
    "tv": 0.45,
    "cell phone": 0.14,
    "book": 0.25,
    "car": 1.50,
    "dog": 0.50,
    "cat": 0.30,
    "potted plant": 0.40,
    "couch": 0.80,
    "dining table": 0.75,
}


def estimate_distance_m(
    bbox: BoundingBox,
    class_name: str,
    fy: float,
    frame_height: int | None = None,
    min_m: float = 0.4,
    max_m: float = 8.0,
) -> float | None:
    """Monocular size ranging: distance ≈ (H_real * fy) / bbox_height_px.

    For close webcam selfies (person fills most of the frame height), switch to
    torso/head heuristics so distance does not collapse unrealistically.
    """
    h_px = bbox.height
    if h_px < 12:
        return None
    name = class_name.lower()
    h_real = CLASS_HEIGHT_M.get(name, 0.5)

    if name == "person" and frame_height:
        frac = h_px / max(1, frame_height)
        aspect = bbox.height / max(1.0, bbox.width)
        # Upper-body / selfie crop: use ~0.9 m torso+head reference
        if frac > 0.55 or aspect < 1.35:
            h_real = 0.95
        elif frac > 0.35:
            h_real = 1.40

    dist = (h_real * fy) / h_px
    return float(max(min_m, min(max_m, dist)))
