"""Rolling cross-modal (LiDAR vs. monocular) disagreement monitor.

amp_core.calibration.distance_fusion computes, per detection, a z-score of
how far the LiDAR-cluster range and the monocular size-prior depth disagree
relative to their own stated uncertainties. A single frame's z-scores are
noisy; this module keeps an exponential moving average across frames and
exposes it as an extra reliability feature.

The point is a self-supervised check that needs no ground truth: if the two
independent modalities keep disagreeing by more than their own uncertainty
models predict, *something* is off (e.g. a class instance far from the
nominal-size prior, or a surface returning biased/absent LiDAR data) even
though neither sensor's own single-modality quality features (brightness,
point density, ...) necessarily show it. The monitor is deliberately
symmetric and does not attribute blame to either sensor — see
amp_core/reliability/estimator.py, where a sustained high score discounts
both lidar and camera confidence rather than just one.
"""

from __future__ import annotations

from typing import Sequence

from amp_core.calibration.distance_fusion import FusedDistance


class CrossModalMonitor:
    def __init__(self, alpha: float = 0.2) -> None:
        self.alpha = alpha
        self._mean_abs_z = 0.0
        self._n_seen = 0
        self._n_conflicts = 0

    def update(self, fused: Sequence[FusedDistance]) -> None:
        z_scores = [abs(f.z_score) for f in fused if f.z_score is not None]
        if not z_scores:
            return
        frame_mean = sum(z_scores) / len(z_scores)
        if self._n_seen == 0:
            self._mean_abs_z = frame_mean
        else:
            self._mean_abs_z = (1 - self.alpha) * self._mean_abs_z + self.alpha * frame_mean
        self._n_seen += len(z_scores)
        self._n_conflicts += sum(1 for f in fused if f.consistency_flag == "conflict")

    def features(self) -> dict[str, float]:
        return {
            "mean_abs_zscore": self._mean_abs_z,
            "n_samples": float(self._n_seen),
            "n_conflicts": float(self._n_conflicts),
        }

    def reset(self) -> None:
        self._mean_abs_z = 0.0
        self._n_seen = 0
        self._n_conflicts = 0
