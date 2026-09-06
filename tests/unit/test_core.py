"""Unit tests for AMP core (run on PC without hardware / ROS2)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from amp_core.calibration.transforms import (  # noqa: E402
    CameraIntrinsics,
    ExtrinsicTransform,
    associate_detection_with_lidar,
    wrap_angle,
)
from amp_core.common.types import (  # noqa: E402
    BoundingBox,
    Detection,
    FusionMode,
    LidarScan,
    Timestamp,
)
from amp_core.detection.backends import DetectorConfig, create_detector, benchmark_detector  # noqa: E402
from amp_core.degradation.inject import DegradationConfig, DegradationEngine  # noqa: E402
from amp_core.dynamic_filter.filter import DynamicObstacleFilter  # noqa: E402
from amp_core.fusion.ekf import AdaptiveEKF, FusionConfig, Measurement2D, bounded_r_scale  # noqa: E402
from amp_core.lidar.processing import (  # noqa: E402
    filter_scan,
    LidarFilterConfig,
    sector_distances,
    scan_quality_features,
)
from amp_core.pipeline import AmpPipeline, PipelineConfig  # noqa: E402
from amp_core.reliability.estimator import (  # noqa: E402
    ReliabilityConfig,
    create_reliability_estimator,
)
from amp_core.common.types import ReliabilityMode  # noqa: E402
from amp_core.safety.supervisor import SafetyConfig, SafetySupervisor  # noqa: E402
from amp_core.common.types import SectorDistances, Twist2D  # noqa: E402
from amp_core.tracking.mot import MultiObjectTracker  # noqa: E402


def _scan(n: int = 360, r: float = 2.0) -> LidarScan:
    angles = [-math.pi + 2 * math.pi * i / n for i in range(n)]
    return LidarScan(ranges=[r] * n, angles=angles, timestamp=Timestamp.now())


def test_wrap_angle() -> None:
    assert abs(wrap_angle(math.pi + 0.1) + (math.pi - 0.1)) < 1e-6


def test_lidar_filter_and_sectors() -> None:
    scan = _scan()
    filt = filter_scan(scan, LidarFilterConfig(min_range=0.2, max_range=10.0))
    assert len(filt.ranges) == 360
    sectors = sector_distances(filt)
    assert 1.5 < sectors.front < 2.5
    feats = scan_quality_features(filt)
    assert 0.9 <= feats["valid_ratio"] <= 1.0


def test_detector_stub_and_tracker() -> None:
    det = create_detector(DetectorConfig(backend="stub"))
    out = det.detect(b"\x00" * 100, 640, 480, 1)
    assert out and out[0].class_name == "person"
    tr = MultiObjectTracker()
    tracks = None
    for _ in range(5):
        tracks = tr.update(out)
    assert tracks and tracks[0].track_id >= 1
    bench = benchmark_detector(det, frames=5)
    assert bench["fps"] > 0


def test_reliability_heuristic_from_features() -> None:
    est = create_reliability_estimator(
        ReliabilityConfig(mode=ReliabilityMode.MODE_HEURISTIC_ADAPTIVE)
    )
    conf = est.estimate(
        lidar_features={
            "valid_ratio": 0.95,
            "point_density": 0.9,
            "range_jump_rate": 0.05,
            "temporal_consistency": 0.95,
        }
    )
    assert 0.7 <= conf.lidar <= 1.0


def test_bounded_covariance_adaptation() -> None:
    cfg = FusionConfig()
    assert bounded_r_scale(1.0, cfg) <= cfg.r_scale_max
    # Confidence is floored at conf_floor (0.05) → scale = 1/0.05 = 20
    assert bounded_r_scale(0.01, cfg) == pytest.approx(1.0 / cfg.conf_floor)
    assert bounded_r_scale(0.01, cfg) <= cfg.r_scale_max
    ekf = AdaptiveEKF(FusionConfig(mode=FusionMode.ADAPTIVE_FUSION))
    from amp_core.common.types import SensorConfidence

    conf = SensorConfidence(0.9, 0.5, 0.7, 0.7, Timestamp.now())
    ekf.predict(0.1)
    ekf.update(Measurement2D(1.0, 0.0, 0.0, "lidar"), conf)
    st = ekf.state()
    assert abs(st.pose.x - 1.0) < 0.5


def test_safety_estop_independent() -> None:
    s = SafetySupervisor(SafetyConfig(emergency_stop_distance=0.25))
    s.heartbeat()
    s.note_lidar()
    sectors = SectorDistances(
        0.1, 2, 2, 2, 2, 2, 2, 2, Timestamp.now()
    )
    status = s.filter_command(Twist2D(0.3, 0.0), sectors)
    assert status.action.value == "ESTOP"
    assert status.limited.linear == 0.0


def test_projection_and_association() -> None:
    K = CameraIntrinsics(500, 500, 320, 240, 640, 480)
    T = ExtrinsicTransform.from_xyz_rpy(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    # Point ahead at 2m in lidar -> camera with identity
    proj = [(320.0, 240.0, 2.0), (325.0, 242.0, 2.1)]
    d = associate_detection_with_lidar(320, 240, proj, pixel_radius=20)
    assert d is not None and 1.9 < d < 2.2


def test_degradation_labeled() -> None:
    eng = DegradationEngine(DegradationConfig(lidar_drop_prob=1.0, seed=1))
    scan = _scan(20)
    out = eng.degrade_scan(scan)
    assert out is not None
    assert len(out.ranges) == 0


def test_pipeline_step_mock() -> None:
    pipe = AmpPipeline(
        PipelineConfig(
            fusion_mode=FusionMode.ADAPTIVE_FUSION,
            force_mock_camera=True,
            force_mock_lidar=True,
            detector_backend="stub",
        )
    )
    snap = pipe.step()
    d = snap.to_dict()
    assert "pose" in d and "sectors" in d and "confidence" in d
    assert "hardware" in d
    pipe.close()


def test_dynamic_filter_toggle() -> None:
    scan = _scan(90, r=1.5)
    filt = DynamicObstacleFilter()
    res = filt.filter(scan, [])
    assert len(res.static_scan.ranges) == len(scan.ranges)
