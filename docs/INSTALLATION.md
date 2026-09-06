# Installation

## Laptop

Supported development/runtime hosts are Windows 11 and current Ubuntu releases with
Python 3.11+.

```bash
python -m venv .venv
# Windows: .venv\Scripts\activate
# Linux:   source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install -e .
python -m pytest -q
```

Install the USB–UART driver required by the LD19 adapter (commonly CH340 or CP210x), then
set its `COMx` or `/dev/ttyUSBx` port in `config/demo_hardware.yaml`.

## Raspberry Pi Zero 2W

Use Raspberry Pi OS Lite 64-bit and follow [edge/README.md](../edge/README.md). `picamera2`
must be installed from apt so its libcamera components match the OS image.

## Models

`yolov8n.pt` is ignored by git and may be downloaded by Ultralytics on first use. The
bundled MobileNet-SSD files provide an offline OpenCV fallback. See `models/README.md`.

## Optional future stack

Ubuntu 24.04/ROS2 Jazzy, the `ros2_ws/` packages, and `deployment/` scripts target a
future onboard-compute robot. They are not needed for the laptop-centric thesis demo and
have not been validated on the Pi Zero 2W.
