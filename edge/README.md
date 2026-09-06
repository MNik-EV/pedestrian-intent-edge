# AMP Camera Edge — Raspberry Pi Zero 2W + IMX219-120

The Pi Zero 2W's only job in this system is to capture from the IMX219-120
CSI camera and relay it over Wi-Fi as an MJPEG HTTP stream. It does **no**
AI, no fusion, and does not touch the LiDAR — those all run on the laptop
(see `demo/perception.py` and `amp_core/vision/network_camera.py`). The
LD19 LiDAR stays wired directly to the laptop over USB-serial, unrelated to
the Pi.

This split exists because the IMX219-120 is a CSI-ribbon camera (not USB/UVC)
so it can only be captured by a device with a CSI port — the Pi — while a
Zero 2W (single-core-class A53 quad @ 1 GHz, 512 MB RAM) is too weak to also
run YOLO detection and sensor fusion in real time. Streaming raw MJPEG and
doing all the heavy compute on the laptop is the standard, low-risk pattern
for this hardware pairing.

## 1. One-time Pi setup

Flash **Raspberry Pi OS Lite (64-bit)** (already done per project notes),
then on the Pi:

```bash
sudo raspi-config
# Interface Options -> Camera -> Enable  (reboot if prompted)

# Verify the camera is detected and can capture (no picamera2 needed for this check):
rpicam-hello --list-cameras
rpicam-hello -t 2000        # should show a 2s preview log with no errors

# Install picamera2 via apt (NOT pip — pip wheels for picamera2/libcamera are
# unreliable on Pi OS Lite; the apt package brings the matching libcamera build):
sudo apt update
sudo apt install -y python3-picamera2 --no-install-recommends
```

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

```bash
sudo cp systemd/amp-camera-stream.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now amp-camera-stream.service
sudo systemctl status amp-camera-stream.service
```

Edit the `WorkingDirectory`/`ExecStart` paths in the unit file first if you
copied `edge/` somewhere other than `/home/pi/amp_edge`.

## 5. Physical mount and re-calibration

The LiDAR and camera are rigidly fixed together by a 3D-printed bracket (CAD
drawing: `docs/hardware/body.pdf`). Once the IMX219-120 + Pi Zero 2W and the
LD19 are mounted on that bracket in their final positions, re-run the
extrinsic calibration on the laptop so the LiDAR-camera geometry used by
`amp_core/calibration/distance_fusion.py` matches the real assembled rig:

```bash
python demo/auto_calibrate_extrinsics.py
```

A rigid mount is what makes a one-time calibration stay valid — if the
camera or LiDAR shift relative to each other after mounting, re-run it again.
