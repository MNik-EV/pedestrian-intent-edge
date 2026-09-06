# AMP Camera Edge — Raspberry Pi Zero 2W + IMX219-120

The Pi Zero 2W's only job in this system is to capture from the IMX219-120
CSI camera and relay it over Wi-Fi as an MJPEG HTTP stream. It does **no**
AI, no fusion, and does not touch the LiDAR — those all run on the laptop
(see `demo/perception.py` and `amp_core/vision/network_camera.py`). The
LD19 LiDAR stays wired directly to the laptop over USB-serial, unrelated to
the Pi.

This split exists because the IMX219-120 is a CSI-ribbon camera (not USB/UVC)
so it can only be captured by a device with a CSI port — the Pi — while a
Zero 2W (quad-core Cortex-A53 @ 1 GHz, 512 MB RAM) is too constrained to also
run YOLO detection and sensor fusion in real time. Streaming raw MJPEG and
doing all the heavy compute on the laptop is the standard, low-risk pattern
for this hardware pairing.

## 1. One-time Pi setup

Flash **Raspberry Pi OS Lite (64-bit)** (already done per project notes). Current
Raspberry Pi OS uses the libcamera/Picamera2 stack and normally needs no camera toggle in
`raspi-config`. With the camera connected while power is off, boot and run:

```bash
# Verify detection and a headless still capture:
rpicam-hello --list-cameras
rpicam-jpeg --nopreview --output camera-test.jpg

# Install Picamera2 via apt if the Lite image did not include it. Apt keeps the Python
# package matched to libcamera; do not start with a standalone pip installation.
sudo apt update
sudo apt install -y python3-picamera2 --no-install-recommends
```

After copying this `edge/` directory to the Pi, the same installation and checks can be
performed automatically. The installer is safe to run again after an interrupted setup:

```bash
cd ~/amp_edge
chmod +x setup_pi.sh
./setup_pi.sh
```

It installs the camera packages, verifies CSI discovery, captures
`~/imx219-camera-test.jpg`, installs the streamer in `~/amp_edge`, enables its per-user
systemd service at boot, and verifies the local HTTP endpoint.

Official references: [Raspberry Pi camera software](https://www.raspberrypi.com/documentation/computers/camera_software.html)
and the [Picamera2 manual](https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf).

Copy this `edge/` folder onto the Pi (e.g. `scp -r edge/ pi@<pi-ip>:~/amp_edge`
from the laptop, or `git clone` the whole repo on the Pi and just run the
script from `edge/` in place).

## 2. Run the streamer

```bash
cd ~/amp_edge   # or wherever you copied edge/
python3 camera_streamer.py --width 640 --height 480 --fps 15 --port 8000
```

From the laptop (or any browser on the same network), open:

```
http://<pi-hostname>.local:8000/
```

(or `http://<pi-ip>:8000/`) — you should see a live preview `<img>` tag and a
raw stream link. Find the Pi's hostname/IP with `hostname` / `hostname -I` on
the Pi, or `ping raspberrypi.local` from the laptop if mDNS/Avahi is working.

If the image is choppy or the Pi struggles, lower `--fps` (e.g. 10) or
`--quality` (e.g. 50) before raising resolution — the Zero 2W's CPU is the
bottleneck, not the network.

## 3. Point the laptop at it

Edit `config/demo_hardware.yaml` on the laptop:

```yaml
camera:
  mode: network
  network:
    url: "http://<pi-hostname-or-ip>:8000/stream.mjpg"
```

Then run the laptop-side demo as usual (`python app.py` or `python
demo_show.py`) — it will pull frames from the Pi instead of a USB camera.
`camera.mode: usb` remains available as a bench-debug fallback (a laptop-
attached USB camera) if the Pi isn't powered on.

## 4. Auto-start on boot (optional but recommended for demo day)

The supplied unit is a per-user service and assumes the folder is `~/amp_edge`. This avoids
hard-coding the Raspberry Pi username:

```bash
mkdir -p ~/.config/systemd/user
cp systemd/amp-camera-stream.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now amp-camera-stream.service
systemctl --user status amp-camera-stream.service
```

To start it after boot even before interactive login, run
`sudo loginctl enable-linger "$USER"`. Edit the unit if the folder is not `~/amp_edge`.

## 5. Physical mount and re-calibration

The LiDAR and camera are rigidly fixed together by a 3D-printed bracket (CAD
drawing: `docs/hardware/body.pdf`). Once the IMX219-120 + Pi Zero 2W and the
LD19 are mounted on that bracket in their final positions, re-run the
extrinsic calibration on the laptop so the LiDAR-camera geometry used by
`amp_core/calibration/distance_fusion.py` matches the real assembled rig:

First measure the LiDAR origin relative to the camera optical centre in camera axes,
then supply those values (metres) as the optimizer seed:

```bash
python demo/auto_calibrate_extrinsics.py \
  --seed-tx <right> --seed-ty <down> --seed-tz <forward> \
  --target-m 1.0 --box-w 0.30 --box-h 0.30
```

A rigid mount is what makes a one-time calibration stay valid. The drawing does not
identify the sensors' internal reference origins, so do not infer the final transform from
CAD dimensions alone. If either sensor shifts after calibration, rerun the procedure.
