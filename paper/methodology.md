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

## 6. Monocular cue and fusion confidence

For a fully visible person, a weak nominal-height prior is

\[
z_{mono}=f_y H_{nominal}/h_{px},\qquad H_{nominal}=1.70\text{ m}.
\]

It selects among ambiguous clusters; it is not a metric label. For other classes the code
uses a deliberately weak nominal-width cue. Base fusion confidence is
\(\min(1,n/12)\). Agreement within 0.45 m increases confidence by 0.25. For a person with
disagreement greater than 1.2 m, the output is marked as a LiDAR/monocular blend
\(0.65r_{LiDAR}+0.35z_{mono}\), and confidence is reduced. Each output records the method,
support count, confidence, and optional prior for later analysis.

If no valid LiDAR cluster exists, the live display labels the absence and may report a
class-size monocular estimate as a fallback. Results must separate LiDAR-associated and
monocular-fallback samples.

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
