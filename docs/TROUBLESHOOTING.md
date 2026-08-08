# Troubleshooting

| Symptom | Check |
|---------|-------|
| No LiDAR | `discover_hardware.py`, serial permissions (`dialout`), `lidar.yaml` port |
| No camera | `/dev/video*`, UVC quirks for PS3 Eye, exclusive device lock |
| Dashboard unreachable | `robot_web` status, firewall, bind `0.0.0.0:8000` |
| Constant ESTOP | Obstacle too close, LiDAR timeout, clear via `/api/navigation/clear_estop` |
| High CPU on Pi | Lower camera FPS, use stub/onnx-lite detector, enable frame skip |
| Auth 403 | Token mismatch with `/etc/amp-robot.env` |
| ROS2 build fails on Windows | Expected — use Ubuntu/Pi or mock stack |

Logs: `journalctl -u robot_core -u robot_web -f` and `experiments/*/events.jsonl`.
