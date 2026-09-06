#!/usr/bin/env python3
"""Standalone live LiDAR-on-camera overlay (real sensors only)."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from amp_core.calibration.transforms import lidar_polar_to_camera
from demo.perception import color_by_range, load_extrinsics, load_intrinsics
from demo.camera_live import LiveCamera
from demo.defaults import default_camera_index, default_lidar_port
from demo.ld19_live import LD19Reader


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=default_camera_index())
    ap.add_argument("--lidar-port", default=default_lidar_port())
    ap.add_argument("--intrinsics", default="calibration/camera_intrinsics.yaml")
    ap.add_argument("--extrinsics", default="calibration/lidar_camera_extrinsics.yaml")
    args = ap.parse_args()

    K = load_intrinsics(Path(args.intrinsics))
    ext = load_extrinsics(Path(args.extrinsics))
    cam = LiveCamera(args.camera, K.width, K.height)
    cam.start()
    lidar = LD19Reader(port=args.lidar_port)
    lidar.start()
    print("Overlay running. Press q to quit.")
    try:
        while True:
            fr = cam.read()
            sc = lidar.get_scan()
            ranges = [p.range_m for p in sc.points]
            angles = [math.radians(p.angle_deg) for p in sc.points]
            proj = lidar_polar_to_camera(ranges, angles, ext, K)
            vis = fr.bgr.copy()
            for u, v, r in proj:
                cv2.circle(vis, (int(u), int(v)), 2, color_by_range(r), -1)
            cv2.putText(
                vis,
                f"overlay pts={len(proj)} cam={fr.fps:.1f} lidar={sc.hz:.1f}Hz closest={sc.closest_m if sc.closest_m<1e8 else float('nan'):.2f}m",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 200),
                1,
            )
            cv2.imshow("lidar_camera_overlay", vis)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        cam.stop()
        lidar.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
