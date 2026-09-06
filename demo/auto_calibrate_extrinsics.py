#!/usr/bin/env python3
"""Automatic practical extrinsic fine-tune using a known box at a known distance.

The translation seed must be measured on the final rigid bracket. The CAD
drawing constrains the mount but does not identify both sensors' optical
centres, so this script never infers extrinsics from the drawing alone.

Place a rectangular box (default 15.5 × 9.5 cm) with its front face ~0.50 m
from the fixture. Draw a ROI around the box face, then the script searches
yaw/pitch/roll + small translation deltas to maximize LiDAR–box alignment
and match the known range.

HONEST: practical field optimizer, not multi-pose hand-eye metrology.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from amp_core.calibration.transforms import (  # noqa: E402
    CameraIntrinsics,
    ExtrinsicTransform,
    lidar_polar_to_camera,
)
from demo.camera_source import open_camera_source  # noqa: E402
from demo.defaults import (  # noqa: E402
    DEFAULT_EXTRINSICS_PATH,
    DEFAULT_INTRINSICS_PATH,
    default_lidar_port,
)
from demo.ld19_live import LD19Reader  # noqa: E402
from demo.perception import color_by_range, load_intrinsics, undistort_bgr  # noqa: E402


@dataclass
class Params:
    tx: float
    ty: float
    tz: float
    roll: float  # deg
    pitch: float
    yaw: float

    def as_ext(self) -> ExtrinsicTransform:
        return ExtrinsicTransform.lidar_to_camera_optical(
            self.tx,
            self.ty,
            self.tz,
            math.radians(self.roll),
            math.radians(self.pitch),
            math.radians(self.yaw),
        )

    def to_dict(self) -> dict:
        return {
            "x": float(self.tx),
            "y": float(self.ty),
            "z": float(self.tz),
            "roll_deg": float(self.roll),
            "pitch_deg": float(self.pitch),
            "yaw_deg": float(self.yaw),
        }


@dataclass
class Capture:
    bgr: np.ndarray
    ranges: list[float]
    angles_rad: list[float]
    roi: tuple[int, int, int, int]  # x1,y1,x2,y2


def select_roi(frame: np.ndarray) -> tuple[int, int, int, int]:
    print(
        "\nDraw a tight rectangle around the FRONT FACE of the box, then ENTER/SPACE."
    )
    print("Press C to cancel.")
    r = cv2.selectROI("Select box face", frame, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow("Select box face")
    x, y, w, h = [int(v) for v in r]
    if w < 20 or h < 20:
        raise RuntimeError("ROI too small — redraw a box around the target face.")
    return x, y, x + w, y + h


def front_cluster(
    ranges: list[float],
    angles: list[float],
    target_m: float,
    band: float = 0.18,
    angle_deg: float = 35.0,
) -> tuple[list[float], list[float]]:
    """Keep LiDAR returns near forward bearing and near the target range."""
    rr, aa = [], []
    for r, a in zip(ranges, angles):
        if r <= 0 or abs(math.degrees(a)) > angle_deg:
            continue
        if abs(r - target_m) > band:
            continue
        rr.append(r)
        aa.append(a)
    return rr, aa


def score(
    params: Params,
    K: CameraIntrinsics,
    cap: Capture,
    target_m: float,
    box_w_m: float,
) -> tuple[float, dict]:
    """Lower is better."""
    x1, y1, x2, y2 = cap.roi
    bw = max(1, x2 - x1)
    bh = max(1, y2 - y1)
    cx = 0.5 * (x1 + x2)
    cy = 0.5 * (y1 + y2)

    fr, fa = front_cluster(cap.ranges, cap.angles_rad, target_m)
    if len(fr) < 5:
        # Fall back to all forward points
        fr, fa = front_cluster(
            cap.ranges, cap.angles_rad, target_m, band=0.45, angle_deg=50.0
        )
    if len(fr) < 3:
        return 1e6, {"reason": "too_few_lidar", "n": len(fr)}

    proj = lidar_polar_to_camera(fr, fa, params.as_ext(), K, z_plane=0.0)
    if not proj:
        return 1e6, {"reason": "no_projection", "n_front": len(fr)}

    inside = [(u, v, r) for u, v, r in proj if x1 <= u <= x2 and y1 <= v <= y2]
    frac = len(inside) / max(1, len(proj))
    use = inside if len(inside) >= 3 else proj
    us = np.array([p[0] for p in use], dtype=np.float64)
    vs = np.array([p[1] for p in use], dtype=np.float64)
    rs = np.array([p[2] for p in use], dtype=np.float64)
    med_r = float(np.median(rs))
    u_err = abs(float(np.median(us)) - cx) / bw
    v_err = abs(float(np.median(vs)) - cy) / bh
    range_err = abs(med_r - target_m)

    # Expected angular width of box vs observed LiDAR span (soft prior)
    if len(us) >= 4:
        obs_w_px = float(np.percentile(us, 90) - np.percentile(us, 10))
        exp_w_px = K.fx * box_w_m / max(0.15, med_r)
        width_err = abs(obs_w_px - exp_w_px) / max(30.0, exp_w_px)
    else:
        width_err = 0.5

    # Prefer points inside ROI; penalize range / center / width mismatch
    cost = (
        (1.0 - frac) * 4.0
        + range_err * 8.0
        + u_err * 2.5
        + v_err * 1.2
        + width_err * 0.8
        + 0.15
        * (abs(params.yaw) / 15.0 + abs(params.pitch) / 15.0 + abs(params.roll) / 15.0)
    )
    metrics = {
        "cost": cost,
        "frac_inside": frac,
        "median_range_m": med_r,
        "range_err_m": range_err,
        "u_err_norm": u_err,
        "v_err_norm": v_err,
        "n_proj": len(proj),
        "n_inside": len(inside),
        "n_front": len(fr),
    }
    return cost, metrics


def optimize(
    K: CameraIntrinsics, cap: Capture, seed: Params, target_m: float, box_w_m: float
) -> tuple[Params, dict]:
    best = seed
    best_cost, best_m = score(best, K, cap, target_m, box_w_m)
    print(f"Seed cost={best_cost:.3f}  metrics={best_m}")

    def run_grid(
        yaw_vals,
        pitch_vals,
        roll_vals,
        dtx_vals,
        dty_vals,
        dtz_vals,
        base: Params,
        label: str,
    ) -> None:
        nonlocal best, best_cost, best_m
        tested = 0
        for yaw in yaw_vals:
            for pitch in pitch_vals:
                for roll in roll_vals:
                    for dtx in dtx_vals:
                        for dty in dty_vals:
                            for dtz in dtz_vals:
                                p = Params(
                                    tx=base.tx + float(dtx),
                                    ty=base.ty + float(dty),
                                    tz=base.tz + float(dtz),
                                    roll=float(roll),
                                    pitch=float(pitch),
                                    yaw=float(yaw),
                                )
                                c, m = score(p, K, cap, target_m, box_w_m)
                                tested += 1
                                if c < best_cost:
                                    best_cost, best, best_m = c, p, m
        print(
            f"{label} tested={tested}  best_cost={best_cost:.3f}  {best.to_dict()}  {best_m}"
        )

    # Stage 1: coarse yaw/pitch + small height/forward nudges
    run_grid(
        np.linspace(-18, 18, 19),
        np.linspace(-8, 8, 9),
        [0.0],
        [0.0],
        [-0.02, 0.0, 0.02],
        [-0.03, 0.0, 0.03],
        seed,
        "Stage 1/3",
    )
    # Stage 2: refine angles + translation around fixture seed
    run_grid(
        np.linspace(best.yaw - 5, best.yaw + 5, 11),
        np.linspace(best.pitch - 4, best.pitch + 4, 9),
        np.linspace(-5, 5, 5),
        np.linspace(-0.025, 0.025, 5),
        np.linspace(-0.025, 0.025, 5),
        np.linspace(-0.035, 0.035, 7),
        seed,
        "Stage 2/3",
    )
    # Stage 3: fine local polish around current best
    run_grid(
        np.linspace(best.yaw - 2.0, best.yaw + 2.0, 9),
        np.linspace(best.pitch - 1.5, best.pitch + 1.5, 7),
        np.linspace(best.roll - 1.5, best.roll + 1.5, 5),
        np.linspace(-0.01, 0.01, 5),
        np.linspace(-0.01, 0.01, 5),
        np.linspace(-0.012, 0.012, 5),
        best,
        "Stage 3/3",
    )
    return best, best_m


def visualize(
    K: CameraIntrinsics, cap: Capture, params: Params, metrics: dict
) -> np.ndarray:
    vis = cap.bgr.copy()
    x1, y1, x2, y2 = cap.roi
    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 220, 255), 2)
    proj = lidar_polar_to_camera(
        cap.ranges, cap.angles_rad, params.as_ext(), K, z_plane=0.0
    )
    for u, v, r in proj:
        inside = x1 <= u <= x2 and y1 <= v <= y2
        color = (0, 255, 80) if inside else color_by_range(r)
        cv2.circle(vis, (int(u), int(v)), 2, color, -1)
    lines = [
        "AUTO EXTRINSIC (practical field)",
        f"t=({params.tx:.3f},{params.ty:.3f},{params.tz:.3f}) m",
        f"rpy=({params.roll:.1f},{params.pitch:.1f},{params.yaw:.1f}) deg",
        f"inside={metrics.get('n_inside')}/{metrics.get('n_proj')}  "
        f"range={metrics.get('median_range_m', float('nan')):.3f}m  "
        f"err={metrics.get('range_err_m', float('nan')):.3f}m",
    ]
    for i, t in enumerate(lines):
        cv2.putText(
            vis,
            t,
            (10, 24 + i * 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (240, 240, 240),
            1,
        )
    return vis


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--camera", type=int, default=None, help="force a local USB camera index"
    )
    ap.add_argument("--camera-url", default=None, help="force a Pi MJPEG stream URL")
    ap.add_argument("--lidar-port", default=default_lidar_port())
    ap.add_argument("--intrinsics", default=DEFAULT_INTRINSICS_PATH)
    ap.add_argument("--out", default=DEFAULT_EXTRINSICS_PATH)
    ap.add_argument(
        "--seed-tx",
        type=float,
        required=True,
        help="measured LiDAR origin X in camera frame [m]",
    )
    ap.add_argument(
        "--seed-ty",
        type=float,
        required=True,
        help="measured LiDAR origin Y in camera frame [m]",
    )
    ap.add_argument(
        "--seed-tz",
        type=float,
        required=True,
        help="measured LiDAR origin Z in camera frame [m]",
    )
    ap.add_argument(
        "--target-m", type=float, default=0.50, help="known box distance (meters)"
    )
    ap.add_argument("--box-w", type=float, default=0.155, help="box width meters")
    ap.add_argument("--box-h", type=float, default=0.095, help="box height meters")
    ap.add_argument("--seconds", type=float, default=2.5, help="capture averaging time")
    args = ap.parse_args()

    Kpath = Path(args.intrinsics)
    if not Kpath.exists():
        print("Run intrinsic calibration first.")
        return 1
    K = load_intrinsics(Kpath)

    print(
        "=== Measured bracket seed in OpenCV camera axes (x right, y down, z forward) ==="
    )
    print(f"  tx={args.seed_tx:.3f}  ty={args.seed_ty:.3f}  tz={args.seed_tz:.3f}")
    print()
    print(
        f"Place the {args.box_w * 100:.1f}×{args.box_h * 100:.1f} cm box at ~{args.target_m:.2f} m."
    )
    print("LiDAR:", args.lidar_port)

    cam = open_camera_source(
        camera_index=args.camera,
        camera_url=args.camera_url,
        width=K.width,
        height=K.height,
    )
    print("Camera source:", type(cam).__name__)
    cam.start()
    lidar = LD19Reader(port=args.lidar_port)
    lidar.start()

    try:
        # Warm up
        t_end = time.monotonic() + 1.5
        while time.monotonic() < t_end:
            cam.read()
            lidar.get_scan()

        fr = cam.read()
        frame_K = K.scaled_to(fr.bgr.shape[1], fr.bgr.shape[0])
        frame_bgr = undistort_bgr(fr.bgr, frame_K)
        # Expected pixel size hint overlay
        hint = frame_bgr.copy()
        exp_w = int(K.fx * args.box_w / args.target_m)
        exp_h = int(K.fy * args.box_h / args.target_m)
        cv2.putText(
            hint,
            f"Expected box ~{exp_w}x{exp_h}px at {args.target_m:.2f}m",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 200),
            2,
        )
        roi = select_roi(hint)

        print(f"Averaging sensors for {args.seconds:.1f}s …")
        frames = []
        all_r: list[float] = []
        all_a: list[float] = []
        t_end = time.monotonic() + args.seconds
        while time.monotonic() < t_end:
            f = cam.read()
            sc = lidar.get_scan()
            frame_K = K.scaled_to(f.bgr.shape[1], f.bgr.shape[0])
            frames.append(undistort_bgr(f.bgr, frame_K))
            for p in sc.points:
                all_r.append(p.range_m)
                all_a.append(math.radians(p.angle_deg))
            time.sleep(0.03)
        bgr = frames[len(frames) // 2]
        K = K.scaled_to(bgr.shape[1], bgr.shape[0])

        # Median-filter dense polar by binning angles
        if not all_r:
            print("No LiDAR points — check COM port / power.")
            return 2
        bins: dict[int, list[float]] = {}
        for r, a in zip(all_r, all_a):
            key = int(round(math.degrees(a) * 2))  # 0.5 deg bins
            bins.setdefault(key, []).append(r)
        ranges, angles = [], []
        for key, rs in sorted(bins.items()):
            ranges.append(float(np.median(rs)))
            angles.append(math.radians(key / 2.0))

        cap = Capture(bgr=bgr, ranges=ranges, angles_rad=angles, roi=roi)
        seed = Params(args.seed_tx, args.seed_ty, args.seed_tz, 0.0, 0.0, 0.0)
        best, metrics = optimize(K, cap, seed, args.target_m, args.box_w)

        vis = visualize(K, cap, best, metrics)
        out_img = Path("experiments") / "extrinsic_auto_result.jpg"
        out_img.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out_img), vis)
        cv2.imshow("extrinsic_auto_result", vis)
        print("\n=== AUTO EXTRINSIC RESULT ===")
        print(json.dumps({**best.to_dict(), **metrics}, indent=2))
        print("Preview saved:", out_img.resolve())
        print("Press any key in the preview window to save & exit…")
        cv2.waitKey(0)

        payload = {
            **best.to_dict(),
            "method": "auto_practical_field_calibration",
            "frame": "lidar_xy_up__to__opencv_camera_optical",
            "fixture": {
                "measured_seed_camera_frame_m": {
                    "x": args.seed_tx,
                    "y": args.seed_ty,
                    "z": args.seed_tz,
                },
                "drawing": "docs/hardware/body.pdf",
                "note": "Seed measured after final rigid assembly; not inferred from CAD alone",
            },
            "target": {
                "distance_m": args.target_m,
                "box_w_m": args.box_w,
                "box_h_m": args.box_h,
            },
            "metrics": metrics,
            "disclaimer": (
                "Automatic practical field calibration with known box distance. "
                "NOT a formal multi-pose extrinsic optimization."
            ),
        }
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
        out.with_suffix(".json").write_text(
            json.dumps(payload, indent=2), encoding="utf-8"
        )
        print("Saved →", out.resolve())
        return 0
    finally:
        cam.stop()
        lidar.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
