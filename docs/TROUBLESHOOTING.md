# Troubleshooting

| Symptom | Check |
|---|---|
| Pi reports no camera | Ribbon orientation, connector latch, `rpicam-hello --list-cameras`, current Pi OS packages |
| Laptop cannot open stream | Pi/laptop subnet, hostname resolution, firewall, URL, and `http://<pi-ip>:8000/` in a browser |
| Stream freezes/reconnects | Reduce Pi FPS/JPEG quality, improve Wi-Fi, use IP instead of mDNS, inspect service log |
| No LD19 | USB–UART driver, port name, 230400 baud, wiring/power, exclusive port lock |
| Overlay is shifted | IMX219 calibration profile, unchanged resolution, measured seed signs, rigid mount, rerun extrinsics |
| Edge overlay bends | Repeat wide-angle intrinsic calibration with chessboard corners near image edges |
| Wrong object range | Verify scan plane crosses the object, reject reflective/glass cases, inspect LiDAR cluster count and fusion confidence |
| YOLO unavailable | Install `ultralytics`; check `yolov8n.pt`; the app reports the selected fallback backend |

Run `python demo/verify_sensors.py --seconds 10` first. It writes
`hardware_live_report.json` with the actual source, rates, port, point count, and error.
