# Deployment

## Implemented deployment

The deployed thesis system has two processes:

1. Pi Zero 2W: `edge/camera_streamer.py`, optionally managed by
   `edge/systemd/amp-camera-stream.service`.
2. Laptop: `python demo_show.py` (or `python app.py`) with the LD19 connected locally.

Before presentation, verify the stream from a browser, run
`python demo/verify_sensors.py --seconds 10`, confirm current IMX219 calibration files,
and perform a known-distance sanity check. Use a trusted/private LAN because the MJPEG
stream is intentionally unauthenticated.

## Future ROS2 deployment assets

The top-level `deployment/` units and `ros2_ws/` packages describe a future Raspberry Pi
5/Ubuntu/ROS2 robot runtime. They are retained for extensibility but are scaffold code,
not the tested deployment described above.
