# Final System Report

**Project:** Adaptive Multimodal Perception and Sensor Fusion for Robust Low-Cost Autonomous Indoor Robots  
**Software version:** 0.1.0-research  
**Date:** 2026-08-08  

## Architecture

Modular stack with ROS-agnostic `amp_core` algorithms, ROS2 Jazzy package wrappers for Raspberry Pi 5, and a FastAPI/WebSocket control-center dashboard. PC hosts development, mocks, training, and offline evaluation; Pi hosts real-time runtime and safety.

## Hardware

Baseline: Raspberry Pi 5, LD19, PS3 Eye; optional IMU, encoders, motor controller via `config/robot.yaml`. Development PC environment (Windows 11) had no WSL/Docker/ROS2 and no LD19/PS3 Eye attached at scaffold time — mock sensors validate software paths.

## Software

- Python 3.11+ `amp_core` (LiDAR processing, vision quality, detector backends, MOT, reliability, adaptive EKF, dynamic filter, safety, navigation, SLAM mock, degradation injection, experiment logging)
- `web_dashboard` live UI (camera overlays, LiDAR radar, map, objects, distances, fusion, teleop)
- `ros2_ws/src/*` ament_python packages
- `evaluation/` ATE/RPE + HTML report generator
- systemd units for Pi deployment

## Algorithms (summary)

- Heuristic sensor reliability from measurable features (fixed / heuristic / learned interface)
- Bounded covariance adaptation for EKF updates
- Temporal motion evidence for dynamic vs static tracks
- LiDAR–camera projection + median-range association for object depth

## Data flow

Sensors → perception → reliability → adaptive fusion → (optional) dynamic filter → SLAM → navigation → **safety supervisor** → motors; telemetry → dashboard + experiment store.

## Installation / deployment

See `docs/INSTALLATION.md`, `docs/DEPLOYMENT.md`. Release via `scripts/build_release.sh`.

## Performance

**NOT YET MEASURED** on Raspberry Pi 5 hardware. PC mock tests exercise functional paths only.

## Known limitations

- LD19 low-level serial reader requires hardware bring-up verification against firmware CRC/packet docs
- ONNX detector requires a calibrated exported model; stub backend used for CI/mock
- Occupancy mock SLAM is for integration; production Pi should bind `slam_toolbox` / Cartographer via `slam_integration`
- Windows PC cannot `colcon build` without WSL/Ubuntu
- No fabricated accuracy claims

## Research contribution

Platform enables controlled comparison of adaptive multimodal fusion under labeled degradation with full experiment replay — contribution quality depends on forthcoming experiments.

## Experimental methodology

See `docs/RESEARCH_PROTOCOL.md` and `paper/`.

## Future work

- Learned reliability estimator  
- Pi detector benchmark matrix (ONNX/TFLite)  
- External accelerators (Coral/Hailo/Jetson) via inference interface  
- Nav2 integration  
- Motion-capture ground truth  

## Acceptance checklist

| Item | Status |
|------|--------|
| PC mock stack runs | Automated tests + `run_mock_stack.py` |
| Unit/integration tests | Automated |
| Dashboard UI | Implemented (served by FastAPI) |
| Experiment record/replay/eval | Implemented |
| Release packaging scripts | Implemented |
| Pi physical sensors / SLAM / nav field test | **Pending hardware** |
| Research metrics | **NOT YET MEASURED** |
