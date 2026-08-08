# Hardware

## Bill of materials (baseline)

1. Raspberry Pi 5 (8GB recommended)
2. LD19 2D LiDAR (USB serial)
3. Sony PS3 Eye USB camera
4. Optional: IMU, wheel encoders, motor controller, battery monitor

## Discovery

```bash
python scripts/discover_hardware.py
# → hardware_report.json
```

## Notes

- Optional sensors are toggled in `config/robot.yaml` without architecture changes.
- Physical validation checklist is in `FINAL_SYSTEM_REPORT.md` acceptance section.
- This development PC may not have LD19/PS3 Eye attached; use mock drivers until hardware is connected.
