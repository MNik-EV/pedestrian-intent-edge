# Installation

## Supported environments

| Role | OS | Notes |
|------|----|-------|
| Dev PC | Windows 11 / Ubuntu 24.04 | Mock stack works without ROS2 |
| Robot | Ubuntu 24.04 ARM64 on Raspberry Pi 5 | ROS2 **Jazzy** |

## PC (this repository)

```bash
python -m pip install -r requirements.txt
python -m pip install -e .
python -m pytest tests -q
python scripts/run_mock_stack.py
```

Optional: install WSL2 + Ubuntu 24.04 + ROS2 Jazzy for full `colcon build`.

## Raspberry Pi 5

1. Flash **Ubuntu 24.04 LTS (64-bit)** for Raspberry Pi.
2. Install ROS2 Jazzy (official docs): https://docs.ros.org/en/jazzy/Installation.html
3. Copy `robot_release_*.tar.gz` from PC `dist/`.
4. Extract and run:

```bash
sudo ./install.sh
sudo ./configure.sh
./health_check.sh
sudo systemctl start robot.service
```

5. Open `http://<PI_IP>:8000`.

## Hardware

- LD19: USB serial (`config/lidar.yaml` → `port`)
- PS3 Eye: UVC (`config/camera.yaml` → `device`)
- Optional IMU / encoders / motors: enable flags in `config/robot.yaml`
