# Thesis outline

**Proposed title:** Real-Time Camera–LiDAR Fusion for Semantic Object Detection and
Robust Ranging Using Low-Cost Hardware

## Abstract

State the problem, exact hardware, laptop-centric architecture, geometric fusion method,
experimental protocol, measured results, and limitations in 200–300 words. Fill the
numeric result sentences only after running `evaluation/evaluate_ranging.py`.

## Chapter 1 — Introduction

1. Motivation: low-cost perception for indoor monitoring/robotics.
2. Camera strengths: texture, colour, and object semantics.
3. Camera weaknesses: monocular scale ambiguity, low light, glare, blur, and occlusion.
4. 2D LiDAR strengths: illumination-independent metric range and 360° geometry.
5. 2D LiDAR weaknesses: sparse planar sampling and no semantic class.
6. Research question: can calibrated camera–LiDAR association improve object-aware
   ranging over a monocular baseline on this low-cost rig?
7. Contributions:
   - a Pi Zero 2W CSI-camera relay and laptop fusion architecture;
   - CRC-validated LD19 acquisition;
   - bearing-gated, depth-clustered, exclusive LiDAR association;
   - reproducible calibration, logging, evaluation, and dashboard tools.
8. Scope: perception/ranging only; ROS2 autonomy is future work.

## Chapter 2 — Background and related work

1. Pinhole camera model and lens distortion.
2. Rigid transforms and heterogeneous sensor calibration.
3. Early, middle, and late multimodal fusion.
4. 2D LiDAR–camera projection and data association.
5. Object detection on constrained/edge systems.
6. Reliability-aware fusion and temporal tracking.
7. Literature gap for low-cost planar LiDAR plus wide-angle monocular cameras.

Insert peer-reviewed citations after a database search. Do not treat this outline as a
source and do not invent titles, authors, or benchmark numbers.

## Chapter 3 — System design

1. Hardware bill of materials and rigid bracket drawing.
2. Compute partition: IMX219/CSI on Pi; LD19/USB and processing on laptop.
3. MJPEG transport choice and its latency trade-off.
4. Software modules and data flow.
5. Coordinate frames, units, and configuration management.
6. Honest separation of live hardware and controlled simulation paths.

## Chapter 4 — Methodology

Use `paper/methodology.md` as the technical core:

1. camera intrinsic calibration and undistortion;
2. LiDAR packet validation and scan assembly;
3. YOLO detections and filtering;
4. LiDAR-to-camera transformation;
5. bearing/image gating, depth clustering, scoring, and robust range;
6. confidence, monocular fallback, and temporal smoothing;
7. tracking and controlled adaptive-fusion harness.

## Chapter 5 — Implementation

1. Pi streamer and systemd user service.
2. reconnecting network-camera client.
3. LD19 background reader.
4. laptop inference/fusion loop.
5. dashboard/WebSocket and experiment logger.
6. calibration and evaluation CLIs.
7. automated tests and failure handling.

## Chapter 6 — Experimental design

Follow `paper/experimental_setup.md`. Define hypotheses, independent/dependent variables,
ground-truth procedure, repetitions, exclusions, and metrics before examining results.

## Chapter 7 — Results

1. calibration quality;
2. ranging error overall and by distance/bearing/scene/class;
3. camera-only versus camera–LiDAR comparison;
4. latency, FPS, dropout, and stability;
5. qualitative success and failure cases.

Populate tables from generated JSON/Markdown. Until data exist, retain `NOT YET MEASURED`.

## Chapter 8 — Discussion

Interpret—not repeat—the results. Discuss practical significance, failure modes, 2D scan
plane limitations, wide-angle distortion, network jitter, pretrained-class limitations,
monocular-size assumptions, and threats to validity.

## Chapter 9 — Conclusion and future work

Summarize only supported findings. Future work may include timestamped low-latency video,
formal multi-pose calibration, learned association/reliability, depth/3D sensors, ROS2,
SLAM, navigation, and optimized onboard inference on stronger hardware.

## Appendices

- wiring and setup commands;
- configuration/calibration files;
- experiment CSV schema;
- selected source listings or repository link/commit;
- safety and reproducibility checklist.
