# Experimental setup and protocol

## Status

Final-rig results are **NOT YET MEASURED**. This protocol is designed to be executed by the
owner with the assembled Pi Zero 2W + IMX219-120 + LD19 rig. Do not merge earlier PS3 Eye
measurements into the final quantitative table.

## Research questions

- RQ1: What object-range error does calibrated camera–LiDAR fusion achieve from 0.5–4 m?
- RQ2: How does error change with bearing, lighting, clutter, class, and LiDAR support?
- RQ3: Does fused ranging reduce error/scale ambiguity relative to the monocular fallback?
- RQ4: Is the system fast and stable enough for a live laptop demonstration?

## Hypotheses fixed before data collection

- H1: fused ranging has lower MAE than monocular size ranging overall.
- H2: range error and missing associations increase near image/scan limits and under clutter.
- H3: poor lighting reduces detection availability more than LiDAR range availability.

Reject or retain these hypotheses based on measured intervals/effect sizes, not preference.

## Equipment and locked configuration

- Pi Zero 2W with Raspberry Pi OS Lite 64-bit and IMX219-120 via CSI
- LD19 via laptop USB–UART at 230400 baud
- rigid printed bracket documented in `docs/hardware/body.pdf`
- laptop model/CPU/RAM/OS recorded in experiment metadata
- fixed stream resolution, FPS, JPEG quality, detector weights, confidence threshold,
  calibration files, and git commit for all comparison runs
- tape measure or laser distance meter with stated resolution/uncertainty

## Calibration acceptance

1. Collect at least 20 diverse chessboard views.
2. Report OpenCV RMS and mean reprojection error; do not discard views solely to improve
   the number unless a predeclared blur/corner-detection rule fails.
3. Fit extrinsics on one or more target poses.
4. Validate on independent centre/left/right poses and report pixel overlay and range error.
5. Recalibrate after any bracket movement or stream-resolution change.

## Ranging experiment matrix

### Core person/target matrix

- ground-truth front-surface distances: 0.5, 1.0, 2.0, 3.0, 4.0 m where space permits;
- camera bearings: approximately −30°, 0°, +30°;
- scenes: normal light, low light, glare/backlight, and foreground clutter;
- repetitions: at least 5 independently repositioned trials per cell.

This yields up to 300 samples. If a distance is physically impossible, mark it `not_run`
with a reason rather than entering a fabricated value.

### Secondary classes

Choose at least three detector-supported rigid objects that intersect the LD19 plane. Use
1, 2, and 3 m; centre/left/right; and at least five repetitions. Record object dimensions
and exclude classes the pretrained model cannot reliably detect.

## Ground truth and sample definition

Measure from the LD19 scan origin to the object's first surface along the LiDAR ray. This
matches the algorithm's front-surface percentile. One sample is one stable placement and
one predeclared aggregation window (for example, median of 30 consecutive fused outputs).
Keep raw frame-level telemetry; do not select the best frame.

Copy `evaluation/ranging_measurements_template.csv` into a new experiment folder. Fill one
row per aggregated placement, including scene, class, ground truth, estimate, method,
bearing, confidence, and LiDAR support. Run:

```bash
python evaluation/evaluate_ranging.py \
  --input experiments/FINAL_<date>/ranging_measurements.csv \
  --out-dir experiments/FINAL_<date>
```

The evaluator produces overall and predeclared group summaries with MAE, RMSE, bias,
median absolute error, P95 absolute error, relative MAE, and a bootstrap 95% MAE interval.

## Baselines

For the same saved placement/window compare:

1. monocular class-size estimate;
2. naive projected-point median if retained as an ablation;
3. implemented bearing-gated clustered LiDAR range;
4. full selection/blending logic, with method labels preserved.

Do not compare methods on different physical placements unless randomized and balanced.

## Runtime and stability

Run at least three 10-minute sessions. Record camera FPS, LiDAR scan rate, detection
latency, CPU/RAM, reconnect count, dropped/empty detections, and fused-range coverage.
Report median and P95 latency plus successful-session duration. Network interruptions must
remain in the record and be discussed.

For each final `SHOW_*` folder run:

```bash
python evaluation/evaluate_live_run.py --experiment experiments/SHOW_<timestamp>
```

## Exclusion rules

Declare before analysis:

- exclude a trial only for logged operator/setup errors, corrupt files, or ground-truth
  measurement failure;
- detector miss is an outcome, not an exclusion;
- no-LiDAR association is an outcome, not an exclusion;
- report counts and reasons for every excluded trial.

## Statistical reporting

Report sample count with every metric. Prefer confidence intervals and paired error
differences over only percentages. Show error versus distance and bearing, not only one
aggregate. Separate calibration-fit data from independent test data and separate
LiDAR-associated outputs from monocular fallback.

## Reproducibility checklist

- [ ] clean git commit hash saved
- [ ] hardware/OS/Python/OpenCV/model versions saved
- [ ] configuration and calibration files copied or hashed
- [ ] target ground truth and uncertainty recorded
- [ ] trial order randomized where practical
- [ ] all missing/excluded samples accounted for
- [ ] generated JSON/Markdown summaries archived
- [ ] no simulated run presented as physical evidence
