# Real-Time LD19–IMX219 Sensor Fusion

Bachelor's thesis implementation for semantic object detection and metric ranging with a
low-cost camera–LiDAR pair. An IMX219-120 CSI camera is captured by a Raspberry Pi Zero
2W and streamed to a laptop; an LD19 2D LiDAR connects directly to the laptop over
USB–serial. The laptop runs detection, geometric fusion, tracking, logging, and the live
FastAPI dashboard.

## Implemented scope

| Component | Status |
|---|---|
| Pi Zero 2W camera relay (`picamera2` → MJPEG/HTTP) | Implemented; physical Pi test required |
| Laptop MJPEG client with reconnect | Implemented and locally integration-tested |
| LD19 packet parsing, CRC8, and 360° scan assembly | Implemented and previously hardware-tested |
| YOLOv8n detection with OpenCV fallbacks | Implemented |
| LiDAR–camera projection and robust object ranging | Implemented |
| Live dashboard and JSONL experiment logging | Implemented |
| IMX219 intrinsic and final-rig extrinsic calibration | Tools ready; values must be measured on the final assembly |
| ROS2, SLAM, navigation, and motor control | Future-work scaffold; not claimed as validated |

No accuracy number is claimed until the experiments in `paper/experimental_setup.md` are
run with measured ground truth. Legacy PS3 Eye calibration and evidence are retained under
`calibration/legacy_ps3eye/` and `experiments/`; they are not valid for the IMX219.

## Architecture

```text
IMX219-120 --CSI--> Pi Zero 2W --MJPEG/Wi-Fi--┐
                                               ├--> laptop: YOLO + fusion + dashboard + logs
LD19 --------USB serial------------------------┘
```

The Pi is deliberately a camera relay only. It does not run AI or fusion. This keeps the
512 MB Zero 2W responsive and makes the laptop the single processing host. See
[the architecture document](docs/ARCHITECTURE.md) for the full data flow.

## Quick start

Requirements: Python 3.11+ on the laptop, Raspberry Pi OS Lite 64-bit on the Pi, and both
devices on the same network.

```bash
python -m pip install -r requirements.txt
python -m pip install -e .
python -m pytest -q
```

1. Configure and start the Pi streamer using [edge/README.md](edge/README.md).
2. Put the Pi URL and LD19 serial port in `config/demo_hardware.yaml`.
3. Check the real links:

   ```bash
   python demo/verify_sensors.py --seconds 10
   ```

4. Calibrate the IMX219 at the exact streaming resolution, then calibrate the rigid
   camera–LiDAR transform using [docs/CALIBRATION.md](docs/CALIBRATION.md).
5. Start the presentation dashboard:

   ```bash
   python demo_show.py
   ```

Open <http://127.0.0.1:8000>. `python app.py` starts the simpler live dashboard. To use a
temporary USB bench camera, pass `--camera 1` or set `camera.mode: usb`; its calibration
must be supplied separately.

## Fusion summary

The camera supplies semantics and bounding boxes. Calibrated intrinsics convert each box
to a horizontal bearing interval. LD19 returns are transformed into the camera frame,
gated by bearing and image location, clustered by depth, and associated uniquely to
detections. A robust front-surface percentile estimates range. A class-size monocular
prior is used only as a weak disambiguation/fallback cue, not as ground truth. Lens
distortion is removed before detection/projection when calibration coefficients exist.

## Repository map

| Path | Purpose |
|---|---|
| `edge/` | Pi Zero 2W camera relay and systemd unit |
| `demo/`, `app.py`, `demo_show.py` | Real-hardware validation path |
| `amp_core/` | ROS-agnostic detection, geometry, fusion, tracking, reliability, and mocks |
| `calibration/` | Generated IMX219 profiles and archived legacy profiles |
| `tests/` | Hardware-free unit/integration tests |
| `paper/` | Thesis chapter skeleton, method, protocol, and result templates |
| `experiments/`, `evaluation/` | Recorded evidence and offline metrics/report tools |
| `ros2_ws/`, `deployment/`, `simulation/` | Explicit future-work scaffold |

## Reproducible research modes

The controlled mock harness supports `lidar_only`, `camera_only`, `fixed_fusion`,
`adaptive_fusion`, and `adaptive_fusion_dynfilter` for labeled degradation/ablation
experiments. It is separate from the real-sensor demo and must not be presented as real
hardware evidence.

For final physical data, `evaluation/evaluate_ranging.py` computes range error and grouped
confidence intervals from the supplied CSV, while `evaluation/evaluate_live_run.py`
summarizes FPS, latency, connection availability, and fused-range coverage from telemetry.

## Documentation

- [Hardware and wiring](docs/HARDWARE.md)
- [Calibration](docs/CALIBRATION.md)
- [Experiment protocol](paper/experimental_setup.md)
- [Current system report](FINAL_SYSTEM_REPORT.md)
- [Pi camera setup](edge/README.md)

Apache-2.0 — see [LICENSE](LICENSE).
