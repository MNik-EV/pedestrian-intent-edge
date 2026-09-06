"""Synthetic check: fusion must ignore nearer clutter and order two people correctly."""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from amp_core.calibration.distance_fusion import fuse_detections_distances
from amp_core.calibration.transforms import CameraIntrinsics, ExtrinsicTransform
from amp_core.common.types import BoundingBox


def test_person_distance_rejects_near_clutter() -> None:
    K = CameraIntrinsics(545.0, 548.0, 320.0, 240.0, 640, 480)
    ext = ExtrinsicTransform.lidar_to_camera_optical(0.0, -0.08, -0.03)

    h = K.fy * 1.7 / 2.0
    bbox = BoundingBox(x1=280, y1=240 - h / 2, x2=360, y2=240 + h / 2)

    ranges: list[float] = []
    angles: list[float] = []
    for a_deg in range(-8, 9):
        a = math.radians(a_deg)
        ranges.extend([0.80, 2.00, 2.05])
        angles.extend([a, a, a])

    h2 = K.fy * 1.7 / 3.5
    bbox2 = BoundingBox(x1=480, y1=240 - h2 / 2, x2=560, y2=240 + h2 / 2)
    for a_deg in range(-24, -16):
        a = math.radians(a_deg)
        ranges.extend([3.50, 3.55])
        angles.extend([a, a])

    out = fuse_detections_distances(
        [(bbox, "person", 0.9), (bbox2, "person", 0.85)],
        ranges,
        angles,
        ext,
        K,
    )
    assert out[0].distance_m is not None and 1.7 < out[0].distance_m < 2.3, out[0]
    assert out[1].distance_m is not None and 3.1 < out[1].distance_m < 3.8, out[1]
    assert out[0].distance_m < out[1].distance_m


if __name__ == "__main__":
    test_person_distance_rejects_near_clutter()
    print("PASS")
