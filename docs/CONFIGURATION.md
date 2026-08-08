# Configuration

All runtime parameters live under `config/`:

| File | Purpose |
|------|---------|
| `robot.yaml` | Hardware enables, fusion mode, safety |
| `lidar.yaml` | LD19 port, ranges, sectors |
| `camera.yaml` | Device, resolution, exposure |
| `fusion.yaml` | EKF / reliability |
| `navigation.yaml` | Speeds, SLAM backend |
| `detection.yaml` | Detector backend / model path |
| `dashboard.yaml` | UI / auth |
| `logging.yaml` | JSONL / SQLite / experiments |

Never hard-code Pi device paths in source. Prefer environment overrides for secrets (`AMP_CONTROL_TOKEN`).
