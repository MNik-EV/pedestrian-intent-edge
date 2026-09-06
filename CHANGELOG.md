# Changelog

## 0.3.0-thesis — 2026-09-06

### Added
- Precision-weighted (inverse-variance) fusion of LiDAR-cluster range and the monocular
  size prior, replacing the earlier fixed 0.65/0.35 blend, in
  `amp_core/calibration/distance_fusion.py`
- Explicit variance models: LD19 noise-floor variance for the LiDAR cluster, anthropometric
  height-uncertainty propagation for the monocular person prior
- Self-supervised cross-modal disagreement z-score and `consistency_flag`
  (`single_source`/`consistent`/`conflict`) reported per detection, with no ground truth
  required
- `amp_core/calibration/cross_modal_monitor.py`: rolling EMA of the disagreement z-score,
  feeding a new symmetric confidence discount in `amp_core/reliability/estimator.py` when
  disagreement between LiDAR and camera persists across frames
- Live dashboards (`app.py`, `demo_show.py`) color-code detections by cross-modal
  consistency and show the rolling disagreement/conflict rate
- `evaluation/ranging_metrics.py` and `evaluate_live_run.py` extended to report accuracy
  grouped by `consistency_flag` and the conflict rate/z-score distribution from live
  telemetry (no manual ground truth needed for the latter)
- H4 hypothesis and results-table scaffolding for validating the consistency flag against
  measured error (`paper/experimental_setup.md`, `paper/results_template.md`)
- 14 new unit tests (`tests/unit/test_uncertainty_fusion.py`) covering the variance models,
  inverse-variance fusion, consistency flagging, and the reliability discount

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
