# Hardware

## Bill of materials

| Item | Interface | Role |
|---|---|---|
| Raspberry Pi Zero 2W, Raspberry Pi OS Lite 64-bit | Wi-Fi to laptop | CSI capture and MJPEG relay only |
| IMX219-120, 8 MP camera module | CSI ribbon to Pi | RGB semantics/object detection input |
| LDROBOT LD19, 2D LiDAR | USB–UART to laptop, 230400 baud | Metric range and geometry input |
| Rigid 3D-printed bracket | Mechanical | Keeps camera–LiDAR extrinsics fixed |
| Laptop | USB + Wi-Fi | AI, fusion, dashboard, and recording |

The optional IMU, encoders, motors, and Pi 5 mentioned by early scaffold files are not
required by the implemented thesis system.

## Wiring

1. With Pi power off, insert the IMX219 ribbon into the Zero 2W CSI connector with the
   contacts in the orientation required by the board/cable.
2. Power the camera/Pi from a stable 5 V supply. Do not power the LD19 motor from a weak
   GPIO rail.
3. Connect the LD19 through its correct USB–UART adapter to the laptop. Confirm the
   adapter voltage/pinout against the module documentation before power-on.
4. Put Pi and laptop on the same trusted LAN and set the Pi URL in
   `config/demo_hardware.yaml`.

## Physical mount

The supplied [SolidWorks drawing](hardware/body.pdf) is an A4, 1:2 drawing in millimetres.
Visible envelope dimensions include 135 mm length and 70 mm height/width, with 2.2 mm and
2.5 mm mounting holes and a 71.47 mm arm feature. These dimensions describe the printed
part, not the sensors' optical centres.

After printing and assembly:

- Ensure neither sensor can rotate or translate relative to the other.
- Keep the LiDAR scan plane clear of the bracket and cables.
- Record the camera optical-centre to LiDAR-origin offsets in OpenCV camera axes:
  `+x` right, `+y` down, `+z` forward.
- Run intrinsic calibration for the exact IMX219 mode and extrinsic calibration for the
  assembled rig. Recalibrate after any mechanical change.

Do not derive final extrinsics from the drawing alone: the CAD sheet does not locate both
internal sensor reference origins.

## Verification

On the Pi:

```bash
rpicam-hello --list-cameras
systemctl --user status amp-camera-stream.service
```

On the laptop:

```bash
python demo/verify_sensors.py --seconds 10
```

A passing report requires live camera frames and valid CRC-checked LD19 scans. The script
does not substitute mock LiDAR data.
