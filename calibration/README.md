# Calibration profiles

The live IMX219 pipeline expects these generated files:

- `imx219_intrinsics.yaml` — camera matrix and distortion at the deployed stream mode
- `imx219_ld19_extrinsics.yaml` — final rigid LiDAR-to-camera transform

Generate them with the procedures in [docs/CALIBRATION.md](../docs/CALIBRATION.md). JSON
copies are written for inspection and external analysis.

`legacy_ps3eye/` contains measured results from the earlier USB-camera prototype. They are
kept for provenance only and are invalid for IMX219 projection or ranging.
