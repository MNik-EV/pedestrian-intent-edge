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
from demo.camera_source import open_camera_source
from demo.defaults import (
    DEFAULT_EXTRINSICS_PATH,
    DEFAULT_INTRINSICS_PATH,
    default_lidar_port,
)
from demo.perception import (
    color_by_range,
    load_extrinsics,
    load_intrinsics,
    undistort_bgr,
)
from demo.ld19_live import LD19Reader


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--camera", type=int, default=None, help="force a local USB camera index"
    )
    ap.add_argument("--camera-url", default=None, help="force a Pi MJPEG stream URL")
    ap.add_argument("--lidar-port", default=default_lidar_port())
    ap.add_argument("--intrinsics", default=DEFAULT_INTRINSICS_PATH)
    ap.add_argument("--extrinsics", default=DEFAULT_EXTRINSICS_PATH)
    args = ap.parse_args()

    K = load_intrinsics(Path(args.intrinsics))
    ext = load_extrinsics(Path(args.extrinsics))
    cam = open_camera_source(
        camera_index=args.camera,
        camera_url=args.camera_url,
        width=K.width,
        height=K.height,
    )
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
            frame_K = K.scaled_to(fr.bgr.shape[1], fr.bgr.shape[0])
            proj = lidar_polar_to_camera(ranges, angles, ext, frame_K)
            vis = undistort_bgr(fr.bgr, frame_K).copy()
            for u, v, r in proj:
                cv2.circle(vis, (int(u), int(v)), 2, color_by_range(r), -1)
            cv2.putText(
                vis,
                f"overlay pts={len(proj)} cam={fr.fps:.1f} lidar={sc.hz:.1f}Hz closest={sc.closest_m if sc.closest_m < 1e8 else float('nan'):.2f}m",
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
