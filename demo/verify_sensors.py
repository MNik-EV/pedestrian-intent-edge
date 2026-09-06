#!/usr/bin/env python3
"""Verify REAL LD19 + camera are streaming. No mocks. Exit nonzero if either fails."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from demo.camera_live import LiveCamera
from demo.defaults import default_camera_index, default_lidar_port
from demo.ld19_live import LD19Reader, list_candidate_ports


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=default_camera_index())
    ap.add_argument(
        "--lidar-port",
        default=default_lidar_port(),
        help="e.g. COM3 or /dev/ttyUSB0",
    )
    ap.add_argument("--seconds", type=float, default=3.0)
    args = ap.parse_args()

    report: dict = {"ok": False, "ports": list_candidate_ports()}
    print("Serial ports:", report["ports"] or "(none)")

    cam = None
    lidar = None
    try:
        print(f"Opening camera index {args.camera}...")
        cam = LiveCamera(index=args.camera)
        cam.start()
        print(f"Opening LD19 (port={args.lidar_port or 'auto'})...")
        lidar = LD19Reader(port=args.lidar_port)
        lidar.start()
        t_end = time.monotonic() + args.seconds
        while time.monotonic() < t_end:
            fr = cam.read()
            sc = lidar.get_scan()
            print(
                f"\r camera_fps={fr.fps:5.1f}  lidar_scan_hz={sc.hz:5.1f}  "
                f"pkt_hz={lidar.packet_hz:6.1f}  points={len(sc.points):4d}  "
                f"closest={sc.closest_m if sc.closest_m < 1e9 else float('nan'):.2f}m   ",
                end="",
                flush=True,
            )
            time.sleep(0.2)
        print()
        fr = cam.read()
        sc = lidar.get_scan()
        report.update(
            {
                "ok": fr.fps > 1.0 and (sc.hz > 0.5 or lidar.packet_hz > 10),
                "camera_index": args.camera,
                "camera_fps": fr.fps,
                "camera_shape": list(fr.bgr.shape),
                "lidar_port": lidar.port,
                "lidar_scan_hz": sc.hz,
                "lidar_packet_hz": lidar.packet_hz,
                "lidar_points": len(sc.points),
                "closest_m": None if sc.closest_m > 1e8 else sc.closest_m,
            }
        )
    except Exception as exc:
        report["error"] = str(exc)
        print("\nERROR:", exc)
    finally:
        if cam:
            cam.stop()
        if lidar:
            lidar.stop()

    out = Path("hardware_live_report.json")
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print("Wrote", out.resolve())
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
