# System architecture

## Validated thesis path

```text
┌──────────────────────────────┐       Wi-Fi / HTTP MJPEG
│ Raspberry Pi Zero 2W         │──────────────────────────┐
│ IMX219-120 --CSI--> picamera2│                          │
│ camera relay only            │                          ▼
└──────────────────────────────┘              ┌───────────────────────────┐
                                              │ Laptop                    │
┌──────────────────────────────┐  USB serial  │ frame decode + undistort  │
│ LD19 2D LiDAR                │─────────────>│ YOLO object detection     │
│ 230400 baud, CRC8 packets    │              │ geometric association     │
└──────────────────────────────┘              │ tracking + reliability    │
                                              │ dashboard + experiment log│
                                              └───────────────────────────┘
```

The Pi Zero 2W is intentionally not the inference computer. It captures the CSI-only
camera and relays compressed frames over Wi-Fi. The LD19 is attached directly to the
laptop, where both streams meet. This partition avoids CPU/RAM pressure on the Zero 2W
and keeps all fusion decisions on one host.

## Live data flow

1. `edge/camera_streamer.py` captures 640×480 RGB frames through `picamera2`, encodes
   MJPEG, and exposes `/stream.mjpg`.
2. `NetworkCameraCapture` decodes the latest JPEG and reconnects after network loss.
3. `LD19Reader` validates 47-byte packets with CRC8 and assembles full revolutions.
4. `DemoPerception` undistorts the image, runs the detector, projects the scan using the
   measured camera matrix and rigid transform, and estimates each object's range.
5. The dashboard publishes annotated JPEG, radar points, detections, timing, and fusion
   metadata. `DemoLogger` records telemetry for later evaluation.

## Coordinate frames

- LiDAR: `x` forward, `y` left, `z` up; scan angles are counter-clockwise.
- OpenCV camera: `X` right, `Y` down, `Z` forward.
- `ExtrinsicTransform.lidar_to_camera_optical()` first applies the fixed axis mapping and
  then the calibrated rotation/translation expressed in the camera frame.

The [mechanical drawing](hardware/body.pdf) documents the rigid bracket. It does not fully
define sensor optical origins, so translation and rotation are measured/calibrated after
final assembly.

## Two intentionally separate runtimes

| Runtime | Purpose | Sensor truth |
|---|---|---|
| `demo/perception.py`, `app.py`, `demo_show.py` | Live physical validation/presentation | Real camera and real LD19; startup fails when either source is unavailable |
| `amp_core/pipeline.py`, `scripts/run_mock_stack.py` | Repeatable simulation and ablation | Controlled mock world and optional laptop webcam |

Keeping these paths explicit prevents simulated LiDAR output from being mistaken for
physical measurements. Shared calibration/fusion functions keep the method consistent.

## Future-work boundary

`ros2_ws/`, `deployment/`, `simulation/`, SLAM, navigation, safety, and motor interfaces
describe a possible autonomous-robot extension. The ROS2 node tree remains a scaffold and
has not been built or validated on the Pi Zero 2W. It is not part of the implemented
thesis claim.
