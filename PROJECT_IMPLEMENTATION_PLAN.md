# Project Implementation Plan

> Historical note (2026-09-06): this was the original Pi 5/ROS2 scaffold plan. The
> implemented bachelor's-thesis path now uses a Pi Zero 2W as an IMX219 camera relay,
> an LD19 directly connected to the laptop, and laptop-side AI/fusion. See `README.md` and
> `docs/ARCHITECTURE.md` for the current architecture. ROS2/navigation items below remain
> future work and must not be reported as completed physical validation.

**Title:** Adaptive Multimodal Perception and Sensor Fusion for Robust Low-Cost Autonomous Indoor Robots

**Codename:** `amp-robot` (Adaptive Multimodal Perception Robot)

## 1. Environment Discovery Summary

| Item | Status |
|------|--------|
| Host OS | Windows 11 Pro (Build 26200), x64 |
| CPU | Intel i7-11800H (8 cores) |
| RAM | ~24 GB |
| Python | 3.11.15 |
| Node.js / npm | 24.18.0 / 11.16.0 |
| Git | 2.54.0 |
| WSL | **Not installed** |
| Docker | **Not installed** |
| Native ROS2 | **Not available on this PC** |
| USB camera | Built-in UVC webcam (PS3 Eye / LD19 not attached) |
| Repository | Empty (greenfield) |

### Supported Target Environments

| Role | OS | ROS2 | Notes |
|------|-----|------|-------|
| **Dev / Research PC** | Windows 11 or Ubuntu 24.04 | Optional | Pure-Python mock stack + evaluation; full ROS2 on Ubuntu/WSL |
| **Deploy Pi 5** | Ubuntu 24.04 LTS (ARM64) | **ROS2 Jazzy** | Real-time runtime |

**Chosen ROS2 distribution:** ROS2 Jazzy Jalisco (LTS, Ubuntu 24.04, ARM64-ready for Pi 5).

## 2. Architectural Principle (PC ≠ Pi)

```
┌─────────────────────────────┐          ┌──────────────────────────────┐
│  PC (Research Workstation)  │          │  Raspberry Pi 5 (Runtime)    │
│  - Development & tests      │   Wi-Fi  │  - Sensor drivers            │
│  - Simulation / mocks       │◄────────►│  - Lightweight perception    │
│  - Model training           │  ROS DDS │  - Adaptive fusion           │
│  - Offline evaluation       │  / REST  │  - SLAM / navigation / safety│
│  - Heavy detector backends  │          │  - Dashboard + logging       │
│  - Experiment replay        │          │  - systemd services          │
└─────────────────────────────┘          └──────────────────────────────┘
```

- **Pi** runs the real-time robot stack (safety-critical, local).
- **PC** develops, trains, evaluates, and replays experiments.
- Algorithm core is **ROS-agnostic** (`amp_core`) so Windows can unit-test without ROS2.
- ROS2 nodes are thin wrappers around `amp_core` for deployment.

## 3. Package Map

| Package | Responsibility |
|---------|----------------|
| `amp_core` | Shared algorithms (fusion, reliability, tracking, transforms) |
| `robot_bringup` | Launch files, compose stack |
| `robot_description` | URDF / TF tree |
| `lidar_driver` | LD19 + mock |
| `camera_driver` | PS3 Eye / V4L2 + mock |
| `vision_perception` | Features, optical flow, VO cues |
| `object_detection` | Backend abstraction (ONNX / stub / PC heavy) |
| `object_tracking` | Multi-object tracker + dynamic/static |
| `lidar_perception` | Filter, sectors, obstacles, confidence features |
| `sensor_reliability` | Confidence estimators (fixed / heuristic / learned iface) |
| `sensor_fusion` | Adaptive EKF with bounded covariance |
| `dynamic_obstacle_filter` | Dynamic vs static separation |
| `slam_integration` | Modular SLAM backend (Cartographer/slam_toolbox iface + mock) |
| `navigation` | Planning + safety supervisor |
| `robot_control` | Motor cmd interface (optional hardware) |
| `diagnostics` | Health / hardware discovery |
| `telemetry` | Aggregated telemetry bus |
| `web_dashboard` | FastAPI + React control center |
| `data_logger` | JSONL / CSV / SQLite / rosbag hooks |
| `experiment_manager` | experiment_id, metadata, record/replay |
| `evaluation_tools` | ATE/RPE, plots, reports |
| `simulation` | Mock world + Gazebo hooks |

## 4. Phased Delivery

| Phase | Deliverable | Status |
|-------|-------------|--------|
| 1 | Discovery + this plan | Done |
| 2 | Skeleton, config, mocks, scripts | Next |
| 3 | LiDAR + camera pipelines | |
| 4 | Detection + tracking + distance | |
| 5 | LiDAR–camera association | |
| 6 | SLAM integration (modular) | |
| 7 | Reliability + adaptive fusion | |
| 8 | Dynamic obstacle filtering | |
| 9 | Navigation + safety | |
| 10 | Web dashboard | |
| 11 | Logging + experiments | |
| 12 | Evaluation + paper assets | |
| 13 | Deployment / systemd / release | |
| 14 | Automated tests | |
| 15 | Documentation + FINAL_SYSTEM_REPORT | |

## 5. Development Strategy on Windows

Without WSL/ROS2:

1. Implement all science/algorithms in `amp_core` (pure Python 3.11).
2. Run `scripts/run_mock_stack.py` for end-to-end PC validation.
3. Ship ROS2 package trees ready to `colcon build` on Ubuntu 24.04 / Pi 5.
4. Provide Docker recipes for Linux ROS2 builds when Docker/WSL becomes available.
5. Hardware validation checklist documented for physical Pi + LD19 + PS3 Eye.

## 6. Research Contribution Focus

**Claim to evaluate (not fabricate):** Adaptive sensor reliability weighting + dynamic filtering improves localization / navigation robustness under controlled sensor degradation vs fixed fusion and single-sensor baselines.

**Modes:** `lidar_only` | `camera_only` | `fixed_fusion` | `adaptive_fusion` | `adaptive_fusion_dynfilter`

**Metrics:** ATE, RPE, RMSE, detection P/R, tracking MOTA-like, collision rate, latency, CPU/RAM.

Results marked **NOT YET MEASURED** until real experiments exist.

## 7. Safety Invariants

- Safety supervisor runs on Pi only; independent of dashboard/Wi-Fi/AI.
- Emergency stop on range, heartbeat loss, critical driver failure.
- Web joystick cannot bypass local safety.

## 8. Versioning

- Software version: `0.1.0-research`
- Config schema version: `1`
- Experiment metadata schema: `1`
