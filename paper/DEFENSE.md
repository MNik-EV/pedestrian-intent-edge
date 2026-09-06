# Proposed Presentation and Defense Scenario

## 15-slide structure

1. Title, student/advisor names, and a photo of the final assembly.
2. Problem: the camera understands "what" an object is but is weak on absolute distance;
   the LD19 is the opposite.
3. Objective and research question, stated as one testable sentence.
4. Hardware and approximate budget **[fill in current prices]**.
5. Mechanical drawing and the importance of a rigid mount.
6. Architecture: CSI → Pi → Wi-Fi → Laptop, and LD19 → USB → Laptop.
7. Real LD19 acquisition, CRC8 validation, and 360° scan assembly.
8. Intrinsic/extrinsic calibration and the coordinate frame.
9. YOLO and its box/class/confidence output.
10. The fusion algorithm in five steps, with a per-stage figure.
11. Live dashboard and telemetry.
12. Experimental protocol and baselines.
13. Real results and plots **[after measurement]**.
14. Failure cases and honest limitations.
15. Summary of the contribution and future work.

## Short demo narration

"On the left we see the real IMX219 image, sent over the network via the Pi Zero 2W. The
AI model detects the object's class and box. The colored points are real LD19 returns,
projected into the image plane through calibration. The distance figure is computed from
the depth cluster aligned with the box, and the point count and confidence are also
logged. If the LiDAR lacks sufficient data for this object, the system does not hide that
— it shows the fallback method with a distinct label. The box color reflects the
cross-modal validation state: green means the LiDAR and the monocular prior agree; amber
means a disagreement beyond either source's own uncertainty was detected and explicitly
reported."

### Live demonstration of the conflict flag (optional, rehearse beforehand)

Hold a printed or displayed photo of a standing person in front of the camera (not the
person themselves). YOLO detects it as "person," and the monocular prior assumes a
relatively far distance based on a 1.7 m height, while the LD19 sees the flat, nearby
surface of the photo itself. The box should turn amber with a `[conflict]` label — a live,
repeatable demonstration of exactly when the system "doubts" both sensors, instead of
confidently reporting a wrong distance.

## Demo run-through on defense day

1. Power on the Pi, camera, and LD19 at least 10 minutes beforehand.
2. Open `http://<pi-ip>:8000/` to prove the stream is live.
3. Run `python demo/verify_sensors.py --seconds 10` and show a passing result.
4. Check the final calibration files and commit hash in advance.
5. Run `python demo_show.py` and put the browser in full-screen.
6. Place an object at 1 m and 2 m, then off to the side; do not claim an accuracy you have
   not measured.
7. Only demonstrate a brief Wi-Fi drop/reconnect if it has been rehearsed beforehand.
8. Show the generated `SHOW_*` folder at the end.

## Prepared answers to likely questions

### Why doesn't the Pi Zero 2W run YOLO itself?

Because of its 512 MB of RAM and limited compute, separating capture from inference gives
better rate and stability. The research goal is low-cost fusion, not proving inference on
the weakest possible CPU.

### Why was MJPEG chosen?

It is simple to implement and debug, has broad browser/OS support, and its decode cost on
the laptop is acceptable. Its cost — higher bandwidth and jitter than low-latency pipelines
— is reported as a stated limitation.

### What is the role of AI, and what is the role of fusion?

YOLO is the AI component, responsible for semantic detection. Geometric fusion relates
YOLO's output to the LD19's metric range. Neither replaces the other.

### Why is 2D LiDAR sufficient?

For objects that the scan plane intersects, it gives low-cost metric range. It is
insufficient for small objects or objects outside that plane — this limitation is the
motivation for future work with 3D LiDAR or a depth camera.

### Wasn't the CAD calibration sufficient on its own?

No. The CAD drawing gives the bracket's shape, but it does not fully specify the LD19's
internal origin or the lens's exact optical center/orientation. A seed measurement and
target-based refinement are therefore both required.

### Were simulation results mixed with real tests?

No. The real hardware path and the simulation harness are kept separate, and the source of
every piece of data is recorded. Final results are reported only from the IMX219 rig's CSV
and telemetry.

### What is the project's novelty?

It is not mere integration: the robust association method gates points by bearing, clusters
them in depth, resolves ambiguity with a weak prior, assigns them exclusively, and reports
a robust-statistic distance — and the entire chain is calibrated, logged, testable, and
honestly separated. Two more specific, tested contributions: (1) combining the LiDAR range
and the monocular prior by inverse-variance weighting instead of a fixed ratio, which
automatically adapts to each source's real quality at that moment; (2) reporting cross-modal
disagreement (a z-score) as a self-supervised reliability signal, with no ground-truth label
required — something not typically seen in student-level fusion demos.

### Why wasn't a fixed ratio (e.g., a 50/50 average) good enough for combining the two senses?

Because each source's quality changes frame to frame: a LiDAR cluster with 15 dense points
should be trusted far more than a sparse 2-point cluster, and a fixed number cannot express
that. Inverse-variance weighting is the minimum-variance linear unbiased estimator — a
standard statistical result for combining two independent measurements, not an arbitrary
choice.

### If the conflict flag doesn't make a decision by itself, what is it for?

Its purpose is not to eliminate a choice but to preserve honesty and traceability: instead
of hiding a disagreement inside an averaged number, it is made explicit so that (1)
suspect samples can be separated in later analysis (hypothesis H4), and (2) when
disagreement persists, `amp_core/reliability/estimator.py` can symmetrically lower both
sensors' confidence. The system deliberately does not decide "which sensor is correct,"
since that is not provable without additional information.

## Backup materials

- a pre-recorded video of the final run;
- a screenshot of the dashboard and the calibration overlay;
- a printed `ranging_summary.md`;
- a spare cable/adapter and a reliable fixed IP or hotspot;
- a local copy of the model to avoid a download on presentation day.
