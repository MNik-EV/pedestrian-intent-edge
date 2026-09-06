# AMP Live Hardware Demo — tonight

## One command

```bash
python app.py
```

Then open: http://127.0.0.1:8000

Optional:

```bash
python app.py
# defaults from config/demo_hardware.yaml → USB PS3 Eye index 1, LiDAR COM17
```

On this PC: **camera index 1 = USB PS3 Eye** (default everywhere). Index 0 = laptop webcam — do not use.

## What is REAL tonight

| Item | Status |
|------|--------|
| USB camera stream | Real |
| LD19 LiDAR UART @ 230400 | Real (must appear as a COM/`ttyUSB` device) |
| LiDAR→camera overlay | Real projection using calib files |
| Object detection | Real pretrained YOLO/MobileNet (no training) |
| Object distance | Real median of LiDAR points falling inside each box |
| Dashboard | Live WebSocket updates |
| JSONL experiment log | Real numeric telemetry under `experiments/DEMO_<timestamp>/` |

## What is NOT tonight (next phase)

- SLAM / mapping
- Localization / path planning / motor control
- ROS2
- Formal multi-pose extrinsic optimization (we use **practical field calibration**)
- Claiming centimeter metrology accuracy

## Setup order (do this once on the robot laptop)

### 0) Drivers / cables
- LD19 must show up as a serial port (CH340 / CP2102 USB-UART common).
- Check: `python demo/verify_sensors.py`

### 1) Verify both sensors
```bash
python demo/verify_sensors.py --seconds 5
```
Expect camera FPS > 10 and LiDAR scan Hz roughly ~5–15 (or high packet Hz).

### 2) Intrinsic calibration (chessboard)
Print a chessboard. Default expects **9×6 inner corners**, 25 mm squares (override with flags).
```bash
python demo/calibrate_intrinsics.py --shots 20
```
Writes `calibration/camera_intrinsics.yaml` and prints mean reprojection error (px).

### 3) Extrinsic calibration (fixture + automatic box tune)

OpenCV camera frame: **x right, y down, z forward**. For this fixture:
- LiDAR **8 cm above** camera → `ty = -0.08`
- Camera **3 cm forward** of LiDAR → `tz = -0.03`
- Lateral aligned → `tx = 0`

Place a **15.5 × 9.5 cm** box with front face at **0.50 m**, then:

```bash
python demo/auto_calibrate_extrinsics.py --target-m 0.50 --box-w 0.155 --box-h 0.095
```

Draw a ROI around the box face; the script auto-searches yaw/pitch/roll + small translation and saves `calibration/lidar_camera_extrinsics.yaml`.

Manual keyboard fine-tune (optional):
```bash
python demo/calibrate_extrinsics.py
```

**Honest label:** practical field calibration, **not** formal multi-pose hand-eye metrology.

### 4) Professor showcase demo
```bash
python demo_show.py
```
Open http://127.0.0.1:8000

Or the simpler dashboard:
```bash
python app.py
```

## Acceptance checklist

- [ ] `python app.py` starts with one command
- [ ] Browser shows live annotated camera + LiDAR radar
- [ ] Object distances roughly match a known held distance (±15 cm typical indoor)
- [ ] `experiments/DEMO_*/telemetry.jsonl` grows with real numbers
- [ ] Stable for ~10 minutes

## If LiDAR is not detected

This PC currently may show **no COM ports** until the USB-UART adapter is plugged and its Windows driver is installed. The demo **will refuse to start with fake LiDAR** — that is intentional.
