"""End-to-end perception / fusion / navigation pipeline for mock and runtime."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

from amp_core.calibration.transforms import (
    CameraIntrinsics,
    ExtrinsicTransform,
    associate_detection_with_lidar,
    bearing_from_bbox_center,
    lidar_polar_to_camera,
)
from amp_core.common.types import FusionMode, Pose2D, Timestamp, Twist2D
from amp_core.detection.backends import DetectorConfig, create_detector
from amp_core.dynamic_filter.filter import DynamicFilterConfig, DynamicObstacleFilter
from amp_core.fusion.ekf import AdaptiveEKF, FusionConfig, Measurement2D
from amp_core.lidar.processing import (
    LidarFilterConfig,
    extract_obstacles,
    filter_scan,
    scan_quality_features,
    sector_distances,
)
from amp_core.mocks.sensors import MockRobot
from amp_core.navigation.planner import NavigationConfig, SimpleNavigator
from amp_core.reliability.estimator import ReliabilityConfig, create_reliability_estimator
from amp_core.safety.supervisor import SafetyConfig, SafetySupervisor
from amp_core.slam.backends import SlamConfig, create_slam
from amp_core.tracking.mot import MultiObjectTracker, TrackerConfig
from amp_core.vision.quality import compute_vision_features, synthetic_feature_grid


@dataclass
class PipelineConfig:
    fusion_mode: FusionMode = FusionMode.ADAPTIVE_FUSION
    dynamic_filtering: bool = True
    detector_backend: str = "stub"
    enable_navigation: bool = False


@dataclass
class PipelineSnapshot:
    timestamp: Timestamp
    pose: dict
    twist: dict
    sectors: dict
    objects: list[dict]
    confidence: dict
    fusion: dict
    safety: dict
    system: dict
    map_preview: dict
    lidar_points: list[dict]
    projected_lidar: list[dict]
    camera_meta: dict
    events: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": self.timestamp.to_dict(),
            "pose": self.pose,
            "twist": self.twist,
            "sectors": self.sectors,
            "objects": self.objects,
            "confidence": self.confidence,
            "fusion": self.fusion,
            "safety": self.safety,
            "system": self.system,
            "map": self.map_preview,
            "lidar_points": self.lidar_points,
            "projected_lidar": self.projected_lidar,
            "camera": self.camera_meta,
            "events": self.events,
        }


class AmpPipeline:
    """Orchestrates mock or injected sensor data through the research stack."""

    def __init__(self, cfg: PipelineConfig | None = None) -> None:
        self.cfg = cfg or PipelineConfig()
        self.robot = MockRobot()
        self.detector = create_detector(DetectorConfig(backend=self.cfg.detector_backend))
        self.tracker = MultiObjectTracker(TrackerConfig())
        self.reliability = create_reliability_estimator(ReliabilityConfig())
        fusion_cfg = FusionConfig(mode=self.cfg.fusion_mode)
        self.ekf = AdaptiveEKF(fusion_cfg)
        self.dyn = DynamicObstacleFilter(
            DynamicFilterConfig(enabled=self.cfg.dynamic_filtering)
        )
        self.slam = create_slam(SlamConfig(dynamic_filtering=self.cfg.dynamic_filtering))
        self.nav = SimpleNavigator(NavigationConfig())
        self.safety = SafetySupervisor(SafetyConfig())
        self.intrinsics = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480)
        self.extrinsics = ExtrinsicTransform.from_xyz_rpy(0.05, 0.0, 0.10, 0.0, 0.0, 0.0)
        self._t_last = time.monotonic()
        self.events: list[str] = []

    def set_cmd(self, linear: float, angular: float) -> None:
        self.robot.cmd = Twist2D(linear, angular)

    def set_goal(self, x: float, y: float, yaw: float = 0.0) -> None:
        self.nav.set_goal(Pose2D(x, y, yaw))

    def step(self) -> PipelineSnapshot:
        now = time.monotonic()
        dt = max(0.01, min(0.5, now - self._t_last))
        self._t_last = now

        self.safety.heartbeat()
        self.robot.step(dt)
        scan_raw = self.robot.lidar.sense(self.robot.world)
        self.safety.note_lidar()
        scan = filter_scan(scan_raw, LidarFilterConfig())
        sectors = sector_distances(scan)
        obstacles = extract_obstacles(scan)
        frame = self.robot.camera.capture(self.robot.world)

        dets = self.detector.detect(frame.data, frame.width, frame.height, frame.channels)
        projected = lidar_polar_to_camera(
            scan.ranges, scan.angles, self.extrinsics, self.intrinsics
        )
        # Associate distances for new detections by temporary index keys later mapped
        tracks = self.tracker.update(dets, dt=dt)
        # Enrich distances / bearings
        for tr in tracks:
            dist = associate_detection_with_lidar(tr.bbox.cx, tr.bbox.cy, projected)
            if dist is not None:
                tr.distance_m = dist
            tr.bearing_deg = math.degrees(
                bearing_from_bbox_center(tr.bbox.cx, frame.width, self.intrinsics.fx, self.intrinsics.cx)
            )

        feats_lidar = scan_quality_features(scan)
        feats_cam = compute_vision_features(
            frame.data,
            frame.width,
            frame.height,
            synthetic_feature_grid(frame.width, frame.height),
        )
        conf = self.reliability.estimate(lidar_features=feats_lidar, camera_features=feats_cam)

        # Ego motion from commanded twist (mock odom)
        self.ekf.predict(dt)
        self.ekf.set_velocity(
            self.robot.cmd.linear * math.cos(self.robot.world.pose.yaw),
            self.robot.cmd.linear * math.sin(self.robot.world.pose.yaw),
            self.robot.cmd.angular,
        )
        # LiDAR pose hint = ground-truth-ish mock (scan matching residual omitted)
        self.ekf.update(
            Measurement2D(
                self.robot.world.pose.x,
                self.robot.world.pose.y,
                self.robot.world.pose.yaw,
                "lidar",
            ),
            conf,
        )
        self.ekf.update(
            Measurement2D(
                self.robot.world.pose.x,
                self.robot.world.pose.y,
                self.robot.world.pose.yaw,
                "odom",
            ),
            conf,
        )
        fusion_state = self.ekf.state()

        dyn_res = self.dyn.filter(scan, tracks)
        slam_scan = dyn_res.static_scan if self.cfg.dynamic_filtering else scan
        slam_pose = self.slam.update(slam_scan, fusion_state.pose)

        if self.cfg.enable_navigation:
            nav_cmd = self.nav.compute(slam_pose, sectors)
        else:
            nav_cmd = self.robot.cmd
        safety = self.safety.filter_command(nav_cmd, sectors, lidar_healthy=True)
        self.robot.cmd = safety.limited

        lidar_pts = [
            {"x": r * math.cos(a), "y": r * math.sin(a), "r": r}
            for r, a in list(zip(scan.ranges, scan.angles))[::3]
        ]
        proj_pts = [{"u": u, "v": v, "r": r} for u, v, r in projected[::5]]

        return PipelineSnapshot(
            timestamp=Timestamp.now(),
            pose=slam_pose.to_dict(),
            twist=safety.limited.to_dict(),
            sectors=sectors.to_dict(),
            objects=[t.to_dict() for t in tracks],
            confidence=conf.to_dict(),
            fusion=fusion_state.to_dict(),
            safety=safety.to_dict(),
            system={
                "cpu_percent": 0.0,  # filled by diagnostics on Pi
                "ram_percent": 0.0,
                "temperature_c": None,
                "camera_fps": self.robot.camera.fps,
                "lidar_fps": self.robot.lidar.fps,
                "detection_fps": self.robot.camera.fps,
                "fusion_hz": fusion_state.update_rate_hz,
                "slam_hz": 1.0 / dt,
            },
            map_preview=self.slam.get_occupancy(),
            lidar_points=lidar_pts,
            projected_lidar=proj_pts,
            camera_meta={
                "width": frame.width,
                "height": frame.height,
                "encoding": frame.encoding,
                "fps": frame.fps,
                # base64 would be heavy; dashboard may request /api/camera/frame
            },
            events=list(self.events[-20:]),
        )
