# Dataset

## Recording mode

Enable via experiment manager / `scripts/record_experiment.py`.

Recorded streams (logical):

- camera metadata (+ optional selective frames)
- LiDAR scans / sectors
- IMU / odom if enabled
- detections, tracks, confidences, poses

High-rate raw imagery should use rosbag/mcap on Pi rather than giant JSONL files (`config/logging.yaml`).

## Replay

```bash
bash scripts/replay_experiment.sh experiments/EXP_xxx
```

Offline method comparison must use the **same** recorded raw inputs with different fusion modes where applicable.
