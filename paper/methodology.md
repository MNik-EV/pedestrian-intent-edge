# Methodology

This document describes the implemented algorithms. Constants are defaults from the code
and should be reported as engineering choices, not universal optima.

## 1. Sensor observations

The IMX219 supplies an RGB frame and the detector supplies class label (c_i), confidence
(s_i), and box (B_i=(u_1,v_1,u_2,v_2)). The LD19 supplies polar returns
((r_j,\alpha_j,q_j)), where range is in metres, angle in radians, and (q_j) is device
confidence. Packets are accepted only after header, length, and CRC8 validation. Points
outside the configured physical range are removed and a revolution is assembled at angle
wrap.

## 2. Camera calibration and undistortion

A planar chessboard is observed in diverse poses at the deployed stream resolution.
OpenCV estimates camera matrix

\[
K=\begin{bmatrix}f_x&0&c_x\\0&f_y&c_y\\0&0&1\end{bmatrix}
\]

and radial/tangential distortion coefficients. Before object detection and projection, the
frame is undistorted using the measured coefficients. If stream resolution changes from
((W_0,H_0)) to ((W,H)), (f_x,c_x) scale by (W/W_0), and (f_y,c_y) by (H/H_0).
The live fusion application refuses to start when the final IMX219 calibration files are
missing; archived PS3 Eye values are never silently reused.

## 3. Rigid LiDAR-to-camera transform

LiDAR axes are (x_L) forward, (y_L) left, (z_L) up. OpenCV camera axes are (X_C)
right, (Y_C) down, (Z_C) forward. The fixed axis mapping is

\[
R_0=\begin{bmatrix}0&-1&0\\0&0&-1\\1&0&0\end{bmatrix}.
\]

A calibrated small rotation (R_\Delta) and translation (t), expressed in the camera
frame, produce

\[
p_C=R_\Delta R_0 p_L+t.
\]

Only points with positive camera depth are projected:

\[
u=f_x X_C/Z_C+c_x,\qquad v=f_y Y_C/Z_C+c_y.
\]

Translation is measured from the final assembled sensor origins and refined against a
shared target. The CAD drawing alone is insufficient to locate internal optical origins.

## 4. Object detection

The primary laptop backend is a pretrained YOLOv8n model. It returns real inference boxes;
it does not synthesize detections. Non-maximum suppression and confidence thresholds
reduce duplicates. A MobileNet-SSD/OpenCV DNN backend provides an offline fallback. The
`StubDetector` is restricted to tests and explicitly labeled mock runs.

## 5. Bearing-gated depth association

For each box, its horizontal angular interval is computed from the left/right pixels:

\[
\theta(u)=\tan^{-1}((u-c_x)/f_x).
\]

The implementation expands the box by 8 pixels and adds angular padding
\(\max(0.015,0.15\Delta\theta)\) radians. Candidate LiDAR points must satisfy the bearing
interval, configured range 0.35–4.5 m, approximate horizontal box support, and a relaxed
vertical support covering the object's middle region. This rejects returns that are close
in Euclidean pixel distance but outside the object's viewing ray.

Candidate depths are sorted and split whenever adjacent camera-frame depths differ by
more than 0.30 m. Clusters normally require at least two points. For cluster (k), the
implemented score is

\[
J_k=2P_k-0.05n_k+0.02\tilde z_k,
\]

where (n_k) is support count, (\tilde z_k) median depth, and (P_k) is distance from a
weak monocular size prior when available. A strong extra penalty applies to a cluster much
nearer than a person's prior, suppressing foreground clutter swallowed by a tall box.
The minimum-score cluster is selected.

The final LiDAR range is the 20th percentile of the selected cluster rather than its
minimum (noise-sensitive) or median (can lie behind the front surface):

\[
\hat r_i=Q_{0.20}(\{r_j:j\in k^*\}).
\]

Detections are processed in descending detector confidence, and assigned LiDAR points are
reserved. This prevents two overlapping boxes from claiming the same depth cluster.

## 6. Monocular cue and uncertainty-aware fusion

For a fully visible person, a weak nominal-height prior is

\[
z_{mono}=f_y H_{nominal}/h_{px},\qquad H_{nominal}=1.70\text{ m}.
\]

It selects among ambiguous clusters; it is not a metric label on its own. For other classes
the code uses a deliberately weak nominal-width cue. Base fusion confidence before any
cross-check is \(\min(1,n/12)\), where \(n\) is the LiDAR cluster's support count.

### 6.1 Turning each cue into a variance

Rather than combining the LiDAR range and the monocular depth with a fixed, arbitrarily
chosen ratio, each is first given an explicit variance so they can be combined by how
certain they actually are in that frame:

\[
\sigma^2_{LiDAR}=\max\!\big(\sigma^2_{floor},\ \operatorname{Var}(\{r_j : j\in k^*\})\big),
\qquad \sigma_{floor}=0.03\text{ m},
\]

i.e. the sample variance of the selected cluster's ranges, floored at the LD19's
order-of-magnitude single-shot noise so a lucky 2-point cluster is never treated as
noise-free. For the monocular depth, the prior's fractional uncertainty is propagated
through \(z_{mono}=f_yH/h_{px}\) to first order:

\[
\sigma^2_{mono}=\Big(z_{mono}\cdot\frac{\sigma_H}{H}\Big)^2,\qquad
\frac{\sigma_H}{H}=\begin{cases}0.07/1.70 & \text{person (stated anthropometric assumption)}\\[2pt]
0.25 & \text{other classes (deliberately wide, no comparable prior)}\end{cases}.
\]

These are engineering assumptions, stated here explicitly, not measured per-unit values.

### 6.2 Precision-weighted combination

When both cues are available, they are combined by inverse-variance (precision) weighting
— the minimum-variance linear unbiased estimator of two independent measurements of the
same quantity:

\[
\hat z=\frac{z_{LiDAR}/\sigma^2_{LiDAR}+z_{mono}/\sigma^2_{mono}}
{1/\sigma^2_{LiDAR}+1/\sigma^2_{mono}},\qquad
\sigma^2_{\hat z}=\Big(\frac{1}{\sigma^2_{LiDAR}}+\frac{1}{\sigma^2_{mono}}\Big)^{-1}.
\]

This replaces an earlier fixed \(0.65/0.35\) blend with a ratio that adapts per detection:
a tight, well-supported LiDAR cluster dominates; a sparse or noisy one yields more to the
monocular prior. \(\sigma^2_{\hat z}\) is reported alongside the distance for later analysis.

### 6.3 Cross-modal disagreement as a self-supervised signal

The two cues are independent estimates of the same physical quantity, so their
disagreement relative to their own stated uncertainty is informative even without ground
truth:

\[
Z=\frac{|z_{LiDAR}-z_{mono}|}{\sqrt{\sigma^2_{LiDAR}+\sigma^2_{mono}}}.
\]

\(Z\le 2\) is labelled `consistent` (confidence raised by 0.25, capped at 1); \(Z>2\) is
labelled `conflict` (confidence scaled by 0.6). The estimate is still fused in both cases —
precision weighting already discounts whichever source has higher variance — but a
conflict is reported, not hidden inside an averaged number. The system does not attempt to
decide *which* sensor is wrong; §8 uses the same signal, aggregated over recent detections,
to symmetrically discount both single-sensor confidences when disagreement is persistent.
This is testable without any additional instrumentation: §7's evaluation protocol records
`z_score`/`consistency_flag` per sample precisely so that measured error can be compared
between `consistent` and `conflict` groups (H4 in `experimental_setup.md`) — if the flag is
meaningful, `conflict` samples should show larger measured error, even though the flag
itself never sees the ground truth.

Each output records the method label, support count, base and cross-checked confidence,
fused variance, z-score, consistency flag, and optional monocular prior for later analysis.

If no valid LiDAR cluster exists, the live display labels the absence (`consistency_flag =
single_source`) and may report a class-size monocular estimate as a fallback. Results must
separate LiDAR-associated, cross-checked, and monocular-fallback samples.

## 7. Temporal processing

Bounding boxes are associated by class-aware IoU and centre proximity. Tracks require
multiple hits, age out after missed frames, and use exponential smoothing to reduce box
jitter. Motion labels depend on sustained image-plane velocity evidence rather than class
alone. Object range uses an EMA, with additional damping for implausibly sudden near jumps.

## 8. Reliability-aware controlled harness

The separate simulation harness computes LiDAR quality (valid ratio, density, range jumps,
temporal consistency) and camera quality (brightness, sharpness, feature count/spread, and
consistency proxies). Confidence (c) scales measurement covariance with bounded inverse
weighting:

\[
s_R=\operatorname{clip}(1/\operatorname{clip}(c,c_{min},c_{max}),s_{min},s_{max}),
\qquad R'=s_RR_0.
\]

It supports LiDAR-only, camera-only, fixed, adaptive, and adaptive-plus-dynamic-filter
ablations under labeled synthetic degradation. This pose-fusion harness is not evidence
that ROS2/SLAM/navigation ran on the physical Pi Zero 2W.

A third, independent input augments \(c\): the rolling mean \(|Z|\) from §6.3
(`CrossModalMonitor`, exponential moving average across recent detections). Once at least
5 cross-checked detections have been observed, both \(c_{lidar}\) and \(c_{camera}\) are
scaled by a shared, symmetric penalty

\[
p=\operatorname{clip}\big(1-0.15\max(0,\overline{|Z|}-1),\ 0.5,\ 1\big),
\]

i.e. no discount while disagreement stays within about one combined sigma, a floor of 0.5
so persistent conflict cannot zero out either sensor's trust, and — deliberately — the same
penalty applied to both sensors, since the monitor cannot attribute fault to either one from
disagreement alone. This is a second, independent reliability cue: single-modality quality
features (brightness, point density, ...) can look nominal while the two sensors still
disagree about the world, and vice versa.

## 9. Computational complexity

For (N) scan points and (M) detections, point preparation is (O(N)). Per-detection
gating is (O(N)), and cluster sorting is (O(K\log K)) for (K\le N) candidates. With a
roughly 360-point planar scan and few objects, detector inference dominates runtime.

## 10. Validity limitations

- Camera and LiDAR are not hardware-trigger synchronized.
- A planar scan can miss objects outside its height.
- Nominal class size varies with instance and pose.
- Wide-angle edge quality depends on chessboard coverage and lens model.
- Reflective/transparent surfaces can produce absent or biased LiDAR returns.
- Pretrained detector domain shift is not solved by geometric fusion.
- §6's variance models (LiDAR noise floor, anthropometric height std-dev, generic-class
  fractional std) are stated engineering assumptions, not values calibrated per-unit or
  per-population; the \(Z>2\) conflict threshold is likewise a fixed choice, not tuned on
  held-out data. Report them as such rather than as measured uncertainties.
