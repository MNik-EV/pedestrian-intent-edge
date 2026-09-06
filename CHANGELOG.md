# Changelog

## 0.2.0-thesis — 2026-09-06

### Added
- Raspberry Pi Zero 2W + IMX219-120 `picamera2` MJPEG edge streamer
- Reconnecting laptop network-camera client and source factory
- Network-camera support in live demo, verification, and calibration workflows
- Final-rig experiment protocol and expanded thesis-writing structure

### Changed
- Final architecture is laptop-centric: LD19 direct USB–serial, camera via Pi relay
- Active calibration filenames now target IMX219; legacy PS3 Eye values are archived
- Documentation distinguishes real hardware, controlled mocks, and future ROS2 work
- Live frames are undistorted when measured distortion coefficients are available

### Fixed
- Camera stream thread cleanup and connection-state handling after network loss
- Calibration scripts no longer assume PS3 Eye geometry or old mount offsets

## 0.1.0-research — 2026-08-08

### Added
- Initial AMP research platform scaffold
- `amp_core` ROS-agnostic algorithms (LiDAR, vision quality, detection abstraction, tracking, reliability, adaptive EKF, dynamic filter, safety, navigation, SLAM mock)
- PC mock stack + FastAPI/WebSocket dashboard
- Experiment logger (JSONL/CSV/SQLite) with reproducibility metadata
- Evaluation tools (ATE/RPE) with NOT YET MEASURED placeholders
- ROS2 Jazzy package tree (`ros2_ws/src`)
- Deployment scripts and systemd units for Raspberry Pi 5
- Documentation and paper support templates
