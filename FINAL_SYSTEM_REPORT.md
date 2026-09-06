# Current system report

**Project:** Real-Time LiDAR–Camera Fusion for Object-Aware Ranging

**Software version:** 0.2.0-thesis
**Updated:** 2026-09-06

## Outcome

The implemented system combines a networked IMX219-120 camera with a laptop-connected
LD19. It detects objects, associates planar LiDAR returns with image detections, estimates
metric range, visualizes both modalities, and records experiment telemetry. The Pi Zero 2W
is a camera bridge; AI and fusion run on the laptop.

## Implemented software

- Pi `picamera2` MJPEG streamer and boot service
- reconnecting laptop stream client
- real LD19 serial parser with official CRC8 table and full-scan assembly
- YOLOv8n plus real OpenCV detector fallbacks
- image undistortion, coordinate transformation, bearing gating, depth clustering,
  unique point reservation, robust range estimation, and labeled monocular fallback
- IOU/motion-evidence object tracker
- FastAPI/WebSocket presentation dashboard and JSONL experiment logger
- intrinsic and practical target-based extrinsic calibration tools supporting the Pi stream
- hardware-free unit/integration suite and controlled mock/ablation harness

## Physical evidence and calibration status

The repository contains prior LD19 + USB-camera bring-up logs and a real PS3 Eye
chessboard calibration. Those prove earlier software/hardware integration but do not
calibrate the final IMX219 rig. They are archived under `experiments/` and
`calibration/legacy_ps3eye/`.

The final IMX219 and bracket cannot be tested from this development session because no SSH
or physical sensor access was supplied. The scripts, profiles, and protocol are ready for
the owner to run. Until then:

- IMX219 intrinsic values: **NOT YET MEASURED**
- final-rig camera–LiDAR extrinsics: **NOT YET MEASURED**
- final distance/latency/robustness metrics: **NOT YET MEASURED**

## Algorithm claim

The camera provides semantic detections while LiDAR provides metric depth. For each
detection, the algorithm computes its calibrated bearing span, selects projected LiDAR
returns, clusters candidate depths, selects a cluster using geometric support and a weak
class-size prior, and reports a robust front-surface range and confidence. This addresses
camera-only scale ambiguity and LiDAR-only lack of semantics. It does not create depth when
both sensors lack evidence.

## Known limitations

- MJPEG over Wi-Fi has variable transport latency and no hardware synchronization.
- A 2D LiDAR measures only one horizontal plane; small/high/low objects may not intersect it.
- Wide-angle calibration quality strongly affects edge projection.
- Pretrained COCO classes may not match every thesis object or local scene.
- Class-size monocular priors vary with object instance and pose.
- The practical extrinsic optimizer is not multi-pose metrology.

## Scope boundary

ROS2 wrappers, mock SLAM, navigation, safety, and deployment assets are retained as future
extensions. They are not presented as a tested autonomous robot and are not required for
the perception/fusion thesis demonstration.

## Acceptance status

| Item | Status |
|---|---|
| Laptop unit/integration suite | Automated and passing at this revision |
| Network-camera protocol | Locally tested with a synthetic MJPEG server |
| Earlier LD19 hardware bring-up | Recorded |
| Final Pi Zero 2W + IMX219 stream | Owner physical test required |
| Final-rig calibration | **NOT YET MEASURED** |
| Quantitative thesis experiments | **NOT YET MEASURED** |
