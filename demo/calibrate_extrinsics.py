#!/usr/bin/env python3
"""Practical field extrinsic calibration (LiDAR → camera).

HONEST SCOPE:
  This is NOT a formal multi-pose hand-eye / PnP extrinsic optimizer.
  Measure mounting offset with a ruler (tx, ty, tz meters) and approximate
  roll/pitch/yaw (degrees), then nudge with the keyboard while watching the
  live LiDAR-on-camera overlay until it visually aligns.

  Suitable for a demo distance association. Not publication-grade metrology.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from amp_core.calibration.transforms import (
    CameraIntrinsics,
    ExtrinsicTransform,
    lidar_polar_to_camera,
)
from demo.camera_source import open_camera_source
from demo.defaults import (
    DEFAULT_EXTRINSICS_PATH,
    DEFAULT_INTRINSICS_PATH,
    default_lidar_port,
)
from demo.ld19_live import LD19Reader
from demo.perception import undistort_bgr


def load_intrinsics(path: Path) -> CameraIntrinsics:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return CameraIntrinsics(
        fx=float(data["fx"]),
        fy=float(data["fy"]),
        cx=float(data["cx"]),
        cy=float(data["cy"]),
        width=int(data["image_width"]),
        height=int(data["image_height"]),
        dist_coeffs=tuple(float(x) for x in data.get("distortion", [0, 0, 0, 0, 0])),
    )


def color_by_range(r: float, rmax: float = 4.0) -> tuple[int, int, int]:
    t = max(0.0, min(1.0, r / rmax))
    return (int(255 * t), int(80), int(255 * (1 - t)))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--camera", type=int, default=None, help="force a local USB camera index"
    )
    ap.add_argument("--camera-url", default=None, help="force a Pi MJPEG stream URL")
    ap.add_argument("--lidar-port", default=default_lidar_port())
    ap.add_argument("--intrinsics", default=DEFAULT_INTRINSICS_PATH)
    ap.add_argument("--out", default=DEFAULT_EXTRINSICS_PATH)
    # Measure these between the final mounted sensor centres in camera axes.
    ap.add_argument("--tx", type=float, default=0.0, help="camera-frame tx (right)")
    ap.add_argument("--ty", type=float, default=0.0, help="camera-frame ty (down)")
    ap.add_argument("--tz", type=float, default=0.0, help="camera-frame tz (forward)")
    ap.add_argument("--roll", type=float, default=0.0)
    ap.add_argument("--pitch", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    args = ap.parse_args()

    Kpath = Path(args.intrinsics)
    if not Kpath.exists():
        print("Missing intrinsics. Run first:\n  python demo/calibrate_intrinsics.py")
        return 1
    K = load_intrinsics(Kpath)

    tx, ty, tz = args.tx, args.ty, args.tz
    roll, pitch, yaw = args.roll, args.pitch, args.yaw
    step_t, step_r = 0.005, 0.5

    cam = open_camera_source(
        camera_index=args.camera,
        camera_url=args.camera_url,
        width=K.width,
        height=K.height,
    )
    print(f"Using camera source {type(cam).__name__}")
    cam.start()
    lidar = LD19Reader(port=args.lidar_port)
    lidar.start()

    print("Keys: W/X tx  A/D ty  R/F tz  I/K pitch  J/L yaw  U/O roll")
    print("      [/] step size   S save   Q quit")
    print(
        "Method: PRACTICAL FIELD CALIBRATION (ruler + visual nudge), not formal optimization."
    )

    try:
        while True:
            fr = cam.read()
            sc = lidar.get_scan()
            ext = ExtrinsicTransform.lidar_to_camera_optical(
                tx, ty, tz, math.radians(roll), math.radians(pitch), math.radians(yaw)
            )
            ranges = [p.range_m for p in sc.points]
            angles = [math.radians(p.angle_deg) for p in sc.points]
            frame_K = K.scaled_to(fr.bgr.shape[1], fr.bgr.shape[0])
            proj = lidar_polar_to_camera(ranges, angles, ext, frame_K, z_plane=0.0)
            vis = undistort_bgr(fr.bgr, frame_K).copy()
            for u, v, r in proj:
                cv2.circle(vis, (int(u), int(v)), 2, color_by_range(r), -1)
            cv2.putText(
                vis,
                "PRACTICAL FIELD CALIB (not formal optimization)",
                (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 200, 255),
                1,
            )
            cv2.putText(
                vis,
                f"t=({tx:.3f},{ty:.3f},{tz:.3f}) m  rpy=({roll:.1f},{pitch:.1f},{yaw:.1f}) deg  overlay={len(proj)}",
                (10, 46),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (220, 220, 220),
                1,
            )
            cv2.imshow("extrinsic_fine_tune", vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            elif key in (ord("w"), ord("W")):
                tx += step_t
            elif key in (ord("x"), ord("X")):
                tx -= step_t
            elif key in (ord("a"), ord("A")):
                ty += step_t
            elif key in (ord("d"), ord("D")):
                ty -= step_t
            elif key in (ord("r"), ord("R")):
                tz += step_t
            elif key in (ord("f"), ord("F")):
                tz -= step_t
            elif key in (ord("i"), ord("I")):
                pitch += step_r
            elif key in (ord("k"), ord("K")):
                pitch -= step_r
            elif key in (ord("j"), ord("J")):
                yaw += step_r
            elif key in (ord("l"), ord("L")):
                yaw -= step_r
            elif key in (ord("u"), ord("U")):
                roll -= step_r
            elif key in (ord("o"), ord("O")):
                roll += step_r
            elif key == ord("["):
                step_t = max(0.001, step_t / 2)
                step_r = max(0.1, step_r / 2)
            elif key == ord("]"):
                step_t *= 2
                step_r *= 2
            elif key in (ord("s"), ord("S")):
                payload = {
                    "x": float(tx),
                    "y": float(ty),
                    "z": float(tz),
                    "roll_deg": float(roll),
                    "pitch_deg": float(pitch),
                    "yaw_deg": float(yaw),
                    "method": "practical_field_calibration",
                    "disclaimer": (
                        "Ruler mount offsets + keyboard visual fine-tune. "
                        "NOT a formal multi-pose extrinsic optimization."
                    ),
                }
                out = Path(args.out)
                out.parent.mkdir(parents=True, exist_ok=True)
                out.write_text(
                    yaml.safe_dump(payload, sort_keys=False), encoding="utf-8"
                )
                out.with_suffix(".json").write_text(
                    json.dumps(payload, indent=2), encoding="utf-8"
                )
                print("Saved ->", out.resolve())
    finally:
        cam.stop()
        lidar.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
