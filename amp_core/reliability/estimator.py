"""Sensor reliability estimation with fixed / heuristic / learned interfaces."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from amp_core.common.types import ReliabilityMode, SensorConfidence, Timestamp
from amp_core.lidar.processing import scan_quality_features
from amp_core.common.types import LidarScan
from amp_core.vision.quality import VisionQualityFeatures


@dataclass
class ReliabilityConfig:
    mode: ReliabilityMode = ReliabilityMode.MODE_HEURISTIC_ADAPTIVE
    fixed_lidar: float = 0.8
    fixed_camera: float = 0.6
    fixed_imu: float = 0.7
    fixed_odom: float = 0.7
    # Heuristic weights
    lidar_valid_w: float = 0.35
    lidar_density_w: float = 0.25
    lidar_jump_w: float = 0.20
    lidar_temporal_w: float = 0.20
    # Cross-modal disagreement discount (see cross_modal_monitor.py): symmetric
    # penalty on both lidar_c and camera_c once the rolling mean |z-score| of
    # LiDAR-vs-monocular distance disagreement exceeds 1 sigma-of-uncertainty.
    cross_modal_zscore_free: float = 1.0
    cross_modal_penalty_per_sigma: float = 0.15
    cross_modal_penalty_floor: float = 0.5
    cross_modal_min_samples: int = 5


def _cross_modal_penalty(
    cross_modal_features: dict[str, float] | None, cfg: ReliabilityConfig
) -> float:
    """Symmetric [floor, 1.0] multiplier applied to both lidar_c and camera_c.

    Cannot tell which sensor is at fault, so it discounts both equally once
    persistent LiDAR-vs-monocular disagreement (see cross_modal_monitor.py)
    exceeds what each source's own stated uncertainty predicts.
    """
    if not cross_modal_features:
        return 1.0
    n = cross_modal_features.get("n_samples", 0.0)
    if n < cfg.cross_modal_min_samples:
        return 1.0
    z = cross_modal_features.get("mean_abs_zscore", 0.0)
    excess = max(0.0, z - cfg.cross_modal_zscore_free)
    penalty = 1.0 - cfg.cross_modal_penalty_per_sigma * excess
    return max(cfg.cross_modal_penalty_floor, min(1.0, penalty))


class ReliabilityEstimator(ABC):
    @abstractmethod
    def estimate(
        self,
        lidar_features: dict[str, float] | None = None,
        camera_features: VisionQualityFeatures | None = None,
        imu_features: dict[str, float] | None = None,
        odom_features: dict[str, float] | None = None,
        cross_modal_features: dict[str, float] | None = None,
    ) -> SensorConfidence:
        raise NotImplementedError


class FixedReliabilityEstimator(ReliabilityEstimator):
    def __init__(self, cfg: ReliabilityConfig) -> None:
        self.cfg = cfg

    def estimate(
        self,
        lidar_features: dict[str, float] | None = None,
        camera_features: VisionQualityFeatures | None = None,
        imu_features: dict[str, float] | None = None,
        odom_features: dict[str, float] | None = None,
        cross_modal_features: dict[str, float] | None = None,
    ) -> SensorConfidence:
        feats: dict[str, float] = {}
        if lidar_features:
            feats.update({f"lidar_{k}": v for k, v in lidar_features.items()})
        if camera_features:
            feats.update({f"cam_{k}": v for k, v in camera_features.to_dict().items()})
        if cross_modal_features:
            feats.update({f"xmodal_{k}": v for k, v in cross_modal_features.items()})
        return SensorConfidence(
            lidar=self.cfg.fixed_lidar,
            camera=self.cfg.fixed_camera,
            imu=self.cfg.fixed_imu,
            odom=self.cfg.fixed_odom,
            timestamp=Timestamp.now(),
            features=feats,
        ).clamped()


class HeuristicReliabilityEstimator(ReliabilityEstimator):
    """Confidence from measurable features only (no invented randomness)."""

    def __init__(self, cfg: ReliabilityConfig) -> None:
        self.cfg = cfg

    def estimate(
        self,
        lidar_features: dict[str, float] | None = None,
        camera_features: VisionQualityFeatures | None = None,
        imu_features: dict[str, float] | None = None,
        odom_features: dict[str, float] | None = None,
        cross_modal_features: dict[str, float] | None = None,
    ) -> SensorConfidence:
        feats: dict[str, float] = {}
        lidar_c = self.cfg.fixed_lidar
        if lidar_features:
            feats.update({f"lidar_{k}": v for k, v in lidar_features.items()})
            lidar_c = (
                self.cfg.lidar_valid_w * lidar_features.get("valid_ratio", 0.0)
                + self.cfg.lidar_density_w * lidar_features.get("point_density", 0.0)
                + self.cfg.lidar_jump_w
                * (1.0 - lidar_features.get("range_jump_rate", 1.0))
                + self.cfg.lidar_temporal_w
                * lidar_features.get("temporal_consistency", 0.0)
            )

        camera_c = self.cfg.fixed_camera
        if camera_features:
            d = camera_features.to_dict()
            feats.update({f"cam_{k}": v for k, v in d.items()})
            # Reprojection error: lower is better; map 0..2 px -> 1..0
            reproj_q = max(0.0, 1.0 - d["mean_reprojection_error"] / 2.0)
            camera_c = (
                0.20 * d["feature_count"]
                + 0.15 * d["feature_spread"]
                + 0.20 * d["brightness"]
                + 0.20 * d["sharpness"]
                + 0.15 * d["optical_flow_consistency"]
                + 0.10 * reproj_q
            )
            camera_c *= max(0.2, 1.0 - 0.7 * d["motion_blur_score"])

        imu_c = self.cfg.fixed_imu
        if imu_features:
            feats.update({f"imu_{k}": v for k, v in imu_features.items()})
            sat = imu_features.get("saturation", 0.0)
            noise = imu_features.get("noise", 0.0)
            temporal = imu_features.get("temporal_consistency", 1.0)
            imu_c = (1.0 - sat) * (1.0 - min(1.0, noise)) * temporal

        odom_c = self.cfg.fixed_odom
        if odom_features:
            feats.update({f"odom_{k}": v for k, v in odom_features.items()})
            consistency = odom_features.get("encoder_consistency", 1.0)
            slip = odom_features.get("slip_indicator", 0.0)
            disagree = odom_features.get("lidar_disagreement", 0.0)
            odom_c = consistency * (1.0 - slip) * (1.0 - min(1.0, disagree))

        if cross_modal_features:
            feats.update({f"xmodal_{k}": v for k, v in cross_modal_features.items()})
            penalty = _cross_modal_penalty(cross_modal_features, self.cfg)
            feats["xmodal_penalty"] = penalty
            lidar_c *= penalty
            camera_c *= penalty

        return SensorConfidence(
            lidar=lidar_c,
            camera=camera_c,
            imu=imu_c,
            odom=odom_c,
            timestamp=Timestamp.now(),
            features=feats,
        ).clamped()


class LearnedReliabilityEstimator(ReliabilityEstimator):
    """Interface for a future ML reliability model. Falls back to heuristic."""

    def __init__(self, cfg: ReliabilityConfig, model_path: str = "") -> None:
        self.cfg = cfg
        self.model_path = model_path
        self._fallback = HeuristicReliabilityEstimator(cfg)
        self._model = None  # reserved

    def estimate(
        self,
        lidar_features: dict[str, float] | None = None,
        camera_features: VisionQualityFeatures | None = None,
        imu_features: dict[str, float] | None = None,
        odom_features: dict[str, float] | None = None,
        cross_modal_features: dict[str, float] | None = None,
    ) -> SensorConfidence:
        # MODE_LEARNED reserved — no fabricated ML scores
        conf = self._fallback.estimate(
            lidar_features, camera_features, imu_features, odom_features, cross_modal_features
        )
        conf.features["learned_model_loaded"] = 1.0 if self._model is not None else 0.0
        return conf


def create_reliability_estimator(cfg: ReliabilityConfig) -> ReliabilityEstimator:
    if cfg.mode == ReliabilityMode.MODE_FIXED:
        return FixedReliabilityEstimator(cfg)
    if cfg.mode == ReliabilityMode.MODE_LEARNED:
        return LearnedReliabilityEstimator(cfg)
    return HeuristicReliabilityEstimator(cfg)


def lidar_confidence_from_scan(
    scan: LidarScan, cfg: ReliabilityConfig | None = None
) -> SensorConfidence:
    cfg = cfg or ReliabilityConfig()
    feats = scan_quality_features(scan)
    return create_reliability_estimator(cfg).estimate(lidar_features=feats)
