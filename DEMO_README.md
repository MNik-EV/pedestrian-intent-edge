# Live hardware demonstration

## Final signal path

- IMX219-120 → CSI → Raspberry Pi Zero 2W → MJPEG/HTTP over Wi-Fi → laptop
- LD19 → USB–serial at 230400 baud → laptop
- Laptop → YOLO detection + calibrated range fusion + dashboard + JSONL log

No mock LiDAR is used by `app.py` or `demo_show.py`. The legacy PS3 Eye remains only as
an explicit `camera.mode: usb` bench fallback.

## Preparation

1. Start the Pi camera relay using [edge/README.md](edge/README.md).
2. Set `camera.network.url` and `lidar.port` in `config/demo_hardware.yaml`.
3. Verify both sources:

   ```bash
   python demo/verify_sensors.py --seconds 10
   ```

4. Generate IMX219 calibration files using [docs/CALIBRATION.md](docs/CALIBRATION.md).
   The archived PS3 Eye values are invalid for this camera/lens/mount.

## Run

```bash
python demo_show.py
```

Open <http://127.0.0.1:8000>. The page shows the annotated RGB image, LiDAR radar,
classified objects, object ranges/bearings, detector latency, camera/scan rates, and
calibration metadata. A simpler UI is available with `python app.py`.

Useful overrides:

```bash
python demo_show.py --camera-url http://192.168.1.50:8000/stream.mjpg --lidar-port COM17
python demo_show.py --camera 1  # local USB bench fallback only
```

## What is real

| Item | Evidence |
|---|---|
| LD19 acquisition | CRC8-validated packets and assembled scans from `demo/ld19_live.py` |
| Camera acquisition | Decoded frames from the Pi MJPEG stream |
| Object semantics | Real pretrained YOLO/MobileNet inference |
| Object range | Calibrated bearing/depth-cluster LiDAR association; monocular cue is labeled |
| Dashboard | Live WebSocket telemetry and JPEG stream |
| Recording | `experiments/SHOW_<timestamp>/telemetry.jsonl` |

## What must still be measured on the physical final rig

- IMX219 intrinsic matrix/distortion and reprojection error
- final camera–LiDAR transform and independent projection error
- distance MAE/RMSE across ranges, bearings, object classes, and lighting
- end-to-end latency/jitter and long-run stability

These are deliberately not pre-filled. The thesis result tables remain `NOT YET
MEASURED` until the supplied protocol is executed.

## Presentation acceptance checklist

- [ ] Pi preview opens from the laptop browser
- [ ] `verify_sensors.py` exits 0 with plausible FPS, scan rate, and point count
- [ ] final IMX219 calibration files exist and their resolution matches the stream
- [ ] LiDAR dots align on targets at centre and both sides of the image
- [ ] known-distance checks at several ranges are within the measured acceptance bound
- [ ] `demo_show.py` runs for at least 10 minutes without an unhandled exception
- [ ] a fresh `SHOW_*` telemetry folder is created
- [ ] dashboard labels the actual detector and camera source

## Failure policy

If either real sensor is absent, the live demo fails with a diagnostic message. For a
hardware-free software rehearsal use `python scripts/run_mock_stack.py`; clearly label its
output as simulation.
