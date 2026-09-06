# Bachelor's Thesis Writing Scaffold

## Proposed title

Design and Implementation of a Real-Time Camera–LiDAR Fusion System for Object
Classification and Distance Estimation on Low-Cost Hardware

## Abstract

A camera provides rich appearance and semantic information but is limited for absolute
distance estimation, low light, direct glare, and low-texture scenes. A 2D LiDAR measures
metric range independently of image texture but cannot by itself determine what an object
is. In this project, an 8-megapixel IMX219-120 camera is connected via the CSI interface to
a Raspberry Pi Zero 2W and streams video as MJPEG over the network to a laptop. An LD19
LiDAR connects directly to the laptop over USB–serial. Object detection, geometric
calibration, LiDAR-point-to-detection association, tracking, display, and data logging all
run on the laptop.

The proposed algorithm first computes each detection box's angular interval from the
calibrated camera model, then selects LiDAR points consistent with that viewing ray and
clusters them in depth. The front-surface range is extracted with a robust statistic, and
the LiDAR-cluster range and the monocular size prior are combined by inverse-variance
(precision) weighting — driven by each cue's own uncertainty in that frame — rather than a
fixed blend ratio. Disagreement between the two independent modalities is reported as a
self-supervised consistency signal, with no ground truth required. Final evaluation covers
MAE, RMSE, bias, the 95th-percentile absolute error, availability rate, system latency, and
the cross-modal conflict rate. Numeric result: **[to be completed after the physical
experiment is run]**.

Keywords: sensor fusion, 2D LiDAR, IMX219, LD19, object detection, distance estimation,
real-time processing

## Chapter 1 — Introduction

### 1.1 Problem statement

No single low-cost sensor covers every perception need on its own. A camera can recognize
a person, chair, vehicle, or other object from appearance, but pixel size alone, without
knowledge of scale, does not yield a reliable absolute distance. Conversely, the LD19
measures range but sees each return only as a distance and an angle. The core problem this
research addresses is correctly attributing the LiDAR's metric measurement to the object
the camera has detected.

### 1.2 Motivation

- lower cost than 3D LiDAR or a depth camera;
- compensating for the camera's low-light/texture weaknesses with LiDAR ranging;
- adding semantic meaning to LiDAR point clouds, which otherwise carry no class label;
- building a testable platform for robotics and environment-monitoring projects.

### 1.3 Research question

Can a calibrated association between camera detection boxes and LD19 depth clusters reduce
object-distance error relative to a monocular-only estimate, and remain stable across
varied scene conditions?

### 1.4 Objectives

1. Real-world bring-up of the IMX219 on the Pi Zero 2W with stable video transport;
2. reception and validation of LD19 packets;
3. real-time object classification with a pretrained model;
4. camera intrinsic calibration and the rigid camera–LiDAR transform;
5. robust distance estimation in the presence of clutter points;
6. reproducible logging and evaluation of results.

### 1.5 Defensible novelty/contribution

This project does not claim to invent a new neural network. Its engineering-research
contribution is a complete, low-cost, decomposed, and measurable pipeline in which the
naive "nearest point to the box center" association is replaced by angular gating, depth
clustering, exclusive point assignment, a robust statistic, and a loggable confidence
score.

This version adds two further, specific, testable contributions, both implemented in code
and covered by unit tests
(`amp_core/calibration/distance_fusion.py`,
`amp_core/calibration/cross_modal_monitor.py`,
`tests/unit/test_uncertainty_fusion.py`):

1. **Principled combination instead of a fixed blend.** Instead of a fixed 65%/35% ratio
   between the LiDAR range and the monocular prior, both sources are given an explicit
   variance (an LD19 noise floor for the LiDAR side; error propagation from the assumed
   human-height uncertainty for the monocular side) and combined by inverse-variance
   weighting — the minimum-variance linear unbiased estimator for two independent
   measurements. This combination adapts, per detection, to the actual data quality in
   that instant rather than to a predetermined ratio.
2. **A self-supervised cross-modal disagreement flag.** Because the two modalities
   independently measure the same physical quantity, their disagreement relative to their
   own stated uncertainty (a z-score) is computable with no ground-truth label at all. This
   signal is used both to label each detection (`consistent`/`conflict`) and — as an
   exponential moving average over recent frames — to symmetrically discount both the
   LiDAR and camera confidence in `amp_core/reliability/estimator.py`; the system does not
   attempt to decide which sensor is at fault, it simply stops hiding the disagreement.
   Hypothesis H4 in `paper/experimental_setup.md` tests this claim against real data: if
   the flag is meaningful, `conflict` samples should show larger measured error than
   `consistent` samples.

### 1.6 Scope

The primary output is an object's class, box, bearing, and distance. ROS2, SLAM,
navigation, and motor control exist in the repository only as future work and are not part
of this thesis's tested claims.

## Chapter 2 — Background and related work

### 2.1 Camera model

Explain the pinhole model, the matrix \(K\), the principal point, the pixel focal length,
and radial/tangential distortion. State why corner coverage matters when calibrating a
120° lens.

### 2.2 2D LiDAR principles

Describe time-of-flight ranging, polar representation, scan rate, per-point confidence,
the horizontal-plane limitation, and behavior on transparent/reflective surfaces.

### 2.3 Object detection

Describe the general detector structure, confidence, IoU, and non-maximum suppression. For
YOLO and camera–LiDAR fusion papers, add real citations from a university database: **[a
literature search and genuine references are required here]**.

### 2.4 Fusion strategies

Compare early, mid, and late fusion. The present system performs geometric fusion at the
detection-output level; it therefore requires no network retraining or multi-sensor
dataset, but its quality depends on calibration.

## Chapter 3 — System design

### 3.1 Hardware architecture

Insert the architecture figure from `docs/ARCHITECTURE.md`. Explain the reasoning behind
the compute split: the Pi Zero 2W is a camera relay only, due to its memory/CPU
constraints, while the laptop performs inference and fusion. The LD19 connects directly to
the laptop, so its data path is unaffected by the camera's Wi-Fi/USB-Ethernet link.

### 3.2 Enclosure and mounting

Insert the `docs/hardware/body.pdf` drawing along with a photo of the final assembly.
State the part's geometric dimensions separately from the sensors' internal optical
origins. Any repositioning after calibration is disallowed, or requires recalibration.

### 3.3 Software architecture

Describe, in data-flow order, the edge, network-reception, LD19, detector, transforms,
distance_fusion, tracker, dashboard, logger, and evaluation modules.

## Chapter 4 — Proposed method

Use `paper/methodology.md` for the exact relations.

### 4.1 Preprocessing

The frame is undistorted using the real, measured IMX219 coefficients. If the resolution
differs from the calibrated one, the intrinsic parameters are scaled proportionally.

### 4.2 Coordinate transformation

The LiDAR's polar point is converted to Cartesian coordinates and mapped into the camera
frame via \(p_C = R\,p_L + t\). After the \(Z_C > 0\) condition, the pixel projection is
computed with the matrix \(K\).

### 4.3 Robust association

For each box, the bearing interval is determined. Points are filtered by range, bearing,
and image region. Gaps larger than 0.30 m start a new depth cluster. The implemented cost
function combines consistency with the prior, point count, and depth. The cluster's 20th
percentile is reported as the front surface. Assignment proceeds from the highest-confidence
detection first, and consumed points are reserved.

### 4.3.1 Inverse-variance weighting and the cross-modal consistency flag

For the exact relations, see Section 6 of `paper/methodology.md`. In summary: the LiDAR
cluster's range receives a sample variance (floored at the sensor's own noise level), and
the monocular depth receives a variance propagated from the assumed height/width
uncertainty; the two are then combined by inverse-variance weighting rather than a fixed
ratio. A disagreement z-score between the two sources, relative to their combined
uncertainty, is computed and labeled `consistent`/`conflict`. This label is recorded in
each detection's output, in live telemetry, and — as a rolling average — feeds a third
input to the reliability-estimation module (`amp_core/reliability/estimator.py`, Section 8
of `paper/methodology.md`).

### 4.4 Tracking and smoothing

Describe class-aware IoU/center-distance matching, minimum hit count, maximum age, box EMA,
and sustained motion evidence. State the smoothing/latency trade-off explicitly.

### 4.5 Error handling

A stream drop triggers reconnection; a missing first frame, missing LD19, or missing final
calibration raises an explicit error. In the real run, no synthetic data is substituted for
the LiDAR.

## Chapter 5 — Implementation

Include a table of the exact Python/OpenCV/Ultralytics/Pi-OS versions and the final commit
hash. For each module, document its input, output, thread model, execution rate, and
expected failure behavior. The dashboard screenshot must come from the final run and show
the detector/camera-source label.

## Chapter 6 — Experiments

Execute the `paper/experimental_setup.md` protocol without post-hoc selection. Every
placement must have a ground truth and a fixed sampling window. Build the CSV from the
template and run the evaluator. A detector miss or a no-LiDAR case is a result, not data to
be discarded.

## Chapter 7 — Results

Populate the `paper/results_template.md` tables only with real output:

- calibration error;
- MAE/RMSE/bias/P95 and confidence interval;
- breakdown by `consistency_flag` for hypothesis H4 (is `conflict` error genuinely larger
  than `consistent` error?), together with the conflict rate and z-score distribution from
  `evaluate_live_run.py`, which requires no ground truth at all;
- breakdown by distance, bearing, scene, class, and method;
- detection rate and fused-distance formation rate;
- median/P95 latency and 10-minute stability.

Sample text once numbers exist: "For n = [...] samples, method [...] achieved an MAE of
[...] meters. The 95% confidence interval was [...]. The paired comparison against the
baseline showed a difference of [...]." Leave all brackets in place until data exist.

## Chapter 8 — Discussion

Compare results against the hypotheses and analyze the causes of observed behavior. Threats
to validity include measurement-tape precision, the number of classes, a single
environment, network jitter, the LD19's limited scan plane, human-body pose, model domain
shift, and practical calibration quality. Maintain the distinction between "statistical
significance" and "practical importance."

## Chapter 9 — Conclusion

Summarize only data-supported findings. Future work may include better synchronization,
multi-pose calibration, a learned association/reliability model, multi-sensor dataset
collection, ROS2/SLAM, and stronger inference hardware.

## Final writing checklist

- [ ] Every figure has a number, units, a caption, and a source.
- [ ] Every reported number is traceable to an experiment file and a commit.
- [ ] PS3 Eye results are never mixed with IMX219 results.
- [ ] Simulation is clearly separated from physical testing.
- [ ] Chapter 2's references are real and consistently styled.
- [ ] Limitations and negative results are not omitted.
- [ ] Final code, calibration, and CSV files are committed to GitHub.
- [ ] Hypothesis H4 (conflict vs. consistent error) has been tested against real data and
      the result — support or rejection — is stated explicitly, not just run and left
      unreported.
