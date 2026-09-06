#!/usr/bin/env python3
"""Quick camera intrinsic calibration with a printed chessboard.

Hold a chessboard in view; the script auto-captures diverse poses until
--shots images are collected, then solves intrinsics and prints RMS error.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from demo.camera_source import open_camera_source
from demo.defaults import DEFAULT_INTRINSICS_PATH


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--camera", type=int, default=None, help="force a local USB camera index"
    )
    ap.add_argument("--camera-url", default=None, help="force a Pi MJPEG stream URL")
    ap.add_argument("--cols", type=int, default=9, help="inner corners along width")
    ap.add_argument("--rows", type=int, default=6, help="inner corners along height")
    ap.add_argument("--square", type=float, default=0.025, help="square size meters")
    ap.add_argument("--shots", type=int, default=20)
    ap.add_argument("--out", default=DEFAULT_INTRINSICS_PATH)
    ap.add_argument("--camera-model", default="IMX219-120")
    args = ap.parse_args()

    pattern = (args.cols, args.rows)
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0 : args.cols, 0 : args.rows].T.reshape(-1, 2)
    objp *= args.square

    cam = open_camera_source(camera_index=args.camera, camera_url=args.camera_url)
    print(f"Using camera source {type(cam).__name__}")
    cam.start()
    obj_points: list[np.ndarray] = []
    img_points: list[np.ndarray] = []
    last_capture = 0.0
    print(
        f"Show chessboard ({args.cols}x{args.rows} inner corners). Need {args.shots} shots."
    )
    print("Press q to abort.")

    try:
        while len(obj_points) < args.shots:
            fr = cam.read()
            gray = cv2.cvtColor(fr.bgr, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, pattern, None)
            vis = fr.bgr.copy()
            if found:
                term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)
                cv2.drawChessboardCorners(vis, pattern, corners2, found)
                if time.monotonic() - last_capture > 0.7:
                    obj_points.append(objp.copy())
                    img_points.append(corners2)
                    last_capture = time.monotonic()
                    print(f" captured {len(obj_points)}/{args.shots}")
            cv2.putText(
                vis,
                f"shots {len(obj_points)}/{args.shots}  fps={fr.fps:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 180),
                2,
            )
            cv2.imshow("intrinsic_calib", vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cam.stop()
        cv2.destroyAllWindows()

    if len(obj_points) < 8:
        print("Not enough shots for calibration.")
        return 1

    h, w = gray.shape[:2]
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (w, h), None, None
    )
    # Mean reprojection error
    total_err = 0.0
    total_pts = 0
    for i in range(len(obj_points)):
        proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, dist)
        err = cv2.norm(img_points[i], proj, cv2.NORM_L2)
        n = len(proj)
        total_err += err * err
        total_pts += n
    mean_reproj = float(np.sqrt(total_err / max(1, total_pts)))

    result = {
        "image_width": w,
        "image_height": h,
        "fx": float(K[0, 0]),
        "fy": float(K[1, 1]),
        "cx": float(K[0, 2]),
        "cy": float(K[1, 2]),
        "distortion": [float(x) for x in dist.reshape(-1)[:5]],
        "rms_opencv": float(rms),
        "mean_reprojection_error_px": mean_reproj,
        "num_images": len(obj_points),
        "chessboard_cols": args.cols,
        "chessboard_rows": args.rows,
        "square_m": args.square,
        "camera_model": args.camera_model,
        "source": type(cam).__name__,
        "note": "Real chessboard calibration at the configured stream resolution",
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
    (out.with_suffix(".json")).write_text(
        json.dumps(result, indent=2), encoding="utf-8"
    )
    print("\n=== Intrinsic calibration result ===")
    print(f"OpenCV RMS: {rms:.4f} px")
    print(f"Mean reprojection error: {mean_reproj:.4f} px")
    print(
        f"fx={result['fx']:.2f} fy={result['fy']:.2f} cx={result['cx']:.2f} cy={result['cy']:.2f}"
    )
    print("Saved", out.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
