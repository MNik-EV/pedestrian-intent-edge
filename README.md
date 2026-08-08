# AMP Robot

**Adaptive Multimodal Perception and Sensor Fusion for Robust Low-Cost Autonomous Indoor Robots**

Research-grade software stack for a Raspberry Pi 5 mobile robot with LD19 LiDAR, PS3 Eye camera, and optional IMU / encoders / motor controller.

## PC ≠ Pi

| Machine | Role |
|---------|------|
| **PC** | Development, tests, simulation, mock sensors, dataset processing, model training, offline evaluation, plots, paper experiments |
| **Raspberry Pi 5** | Real-time runtime: drivers, lightweight perception, adaptive fusion, SLAM, navigation, safety, dashboard, logging |

Everything visible live on the dashboard is **recordable and replayable** under an `experiment_id` for fair method comparisons (LiDAR-only vs fixed fusion vs adaptive fusion ± dynamic filtering).

## Quick start (Windows / PC mock — no ROS2 required)

```bash
python -m pip install -r requirements.txt
python scripts/discover_hardware.py
python -m pytest tests -q
python scripts/run_mock_stack.py
```

Open http://127.0.0.1:8000 — control token default: `dev-token-change-me`

Record / evaluate:

```bash
python scripts/record_experiment.py --seconds 10 --mode adaptive_fusion
python evaluation/run_experiment.py --experiment experiments/EXP_...
python evaluation/generate_report.py --experiment experiments/EXP_...
```

## Raspberry Pi 5 (ROS2 Jazzy + runtime)

See [docs/INSTALLATION.md](docs/INSTALLATION.md) and [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md).

```bash
./scripts/build_release.sh
# copy dist/robot_release_*.tar.gz to Pi
sudo ./install.sh && sudo ./configure.sh
sudo systemctl start robot.service
# http://<PI_IP>:8000
```

## Research modes

- `lidar_only` / `camera_only` / `fixed_fusion` / `adaptive_fusion` / `adaptive_fusion_dynfilter`

Metrics (ATE, RPE, …) are **NOT YET MEASURED** until controlled experiments with ground truth are run. Do not treat placeholders as results.

## Documentation

| Doc | Path |
|-----|------|
| Architecture | [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) |
| Implementation plan | [PROJECT_IMPLEMENTATION_PLAN.md](PROJECT_IMPLEMENTATION_PLAN.md) |
| Final system report | [FINAL_SYSTEM_REPORT.md](FINAL_SYSTEM_REPORT.md) |
| API | [docs/API.md](docs/API.md) |
| Research protocol | [docs/RESEARCH_PROTOCOL.md](docs/RESEARCH_PROTOCOL.md) |

## License

Apache-2.0 — see [LICENSE](LICENSE).
