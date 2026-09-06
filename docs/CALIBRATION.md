# Camera–LiDAR calibration

Calibration is mandatory for defensible range association. The old PS3 Eye results under
`calibration/legacy_ps3eye/` are historical evidence only and must not be copied to the
IMX219 profile.

## 1. Lock the operating mode

Start the Pi relay at the same resolution used for the final experiment (default
640×480). Do not change resolution, lens focus, or sensor mounting after calibration.

## 2. IMX219 intrinsics and distortion

Print a flat 9×6-inner-corner chessboard with accurately measured 25 mm squares. Capture
at least 20 sharp views spanning the centre, edges, distances, and tilts:

```bash
python demo/calibrate_intrinsics.py --cols 9 --rows 6 --square 0.025 --shots 20
```

The command uses the configured network camera and writes
`calibration/imx219_intrinsics.yaml` plus JSON. Inspect the reported reprojection error and
repeat if corners cover little of the wide-angle image or images are blurred. The live
pipeline undistorts frames before projection whenever non-zero coefficients exist.

## 3. Measure a transform seed

With the final bracket assembled, measure the vector from the camera optical centre to
the LD19 scan origin, expressed in camera axes: `x` right, `y` down, `z` forward. Record
metres and signs. The CAD drawing constrains the bracket but does not expose both internal
sensor origins.

## 4. Practical extrinsic optimization

Place a flat-front rectangular target at a tape-measured distance. Keep it visible in the
camera and intersected by the LiDAR plane. Supply the measured transform seed; for
example, replace the values below with the actual assembly measurements:

```bash
python demo/auto_calibrate_extrinsics.py \
  --seed-tx 0.000 --seed-ty -0.050 --seed-tz -0.020 \
  --target-m 1.000 --box-w 0.300 --box-h 0.300
```

Draw a tight ROI around the target face. The optimizer searches small rotation and
translation changes and writes `calibration/imx219_ld19_extrinsics.yaml` plus JSON.

For visual fine tuning:

```bash
python demo/calibrate_extrinsics.py --tx 0.000 --ty -0.050 --tz -0.020
```

This is a practical target-based calibration, not a formal multi-pose hand–eye method.
Report it using that wording.

## 5. Independent validation

Do not validate on the same target pose used for fitting. Place targets at multiple
bearings and distances, then record projected-pixel error and fused range error. Suggested
distances are 0.5, 1, 2, 3, and 4 m where the room permits. Save raw observations and
summary statistics; never replace missing measurements with estimates.

## Time alignment

The live implementation uses the freshest decoded camera frame and freshest completed
LiDAR revolution on the laptop. Wi-Fi latency and scan time are limitations. For moving
targets, measure stream latency/jitter and keep motion speed controlled; hardware-triggered
synchronization is outside this rig's scope.
