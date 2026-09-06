"""Controlled simulation and repeatable-ablation pipeline.

This module is not the final live LD19/IMX219 runtime. ``demo.perception`` owns
that real-hardware path. Here, mock sensors are used only when explicitly
requested so simulated LiDAR can never be labelled as a live serial device.
Both paths share the same robust object-distance fusion algorithm.
"""

from __future__ import annotations

import base64
import math
import time
from dataclasses import dataclass, field
from typing import Any

from amp_core.calibration.cross_modal_monitor import CrossModalMonitor
from amp_core.calibration.distance_fusion import fuse_detections_distances
from amp_core.calibration.transforms import (
    CameraIntrinsics,
    ExtrinsicTransform,
    bearing_from_bbox_center,
    lidar_polar_to_camera,
)
from amp_core.common.types import FusionMode, LidarScan, Pose2D, Timestamp, Twist2D
from amp_core.detection.backends import DetectorConfig, create_detector
from amp_core.dynamic_filter.filter import DynamicFilterConfig, DynamicObstacleFilter
from amp_core.fusion.ekf import AdaptiveEKF, FusionConfig, Measurement2D
from amp_core.hardware.discovery import discover_hardware
from amp_core.lidar.processing import (
    LidarFilterConfig,
    filter_scan,
    scan_quality_features,
    sector_distances,
)
from amp_core.mocks.sensors import MockRobot
from amp_core.navigation.planner import NavigationConfig, SimpleNavigator
from amp_core.reliability.estimator import (
    ReliabilityConfig,
    create_reliability_estimator,
)
from amp_core.safety.supervisor import SafetyConfig, SafetySupervisor
from amp_core.slam.backends import SlamConfig, create_slam
from amp_core.tracking.mot import MultiObjectTracker, TrackerConfig
from amp_core.vision.distance import estimate_distance_m
from amp_core.vision.quality import compute_vision_features, synthetic_feature_grid
from amp_core.vision.webcam import WebcamCapture, WebcamConfig


@dataclass
class PipelineConfig:
    fusion_mode: FusionMode = FusionMode.ADAPTIVE_FUSION
    dynamic_filtering: bool = True
    detector_backend: str = "auto"
    enable_navigation: bool = False
    # auto = probe hardware; true/false force
    force_mock_camera: bool = False
    force_mock_lidar: bool = False
    camera_index: int | None = None
    camera_width: int = 640
    camera_height: int = 480


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
    hardware: dict
    camera_jpeg_b64: str | None = None
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
            "hardware": self.hardware,
            "camera_jpeg_b64": self.camera_jpeg_b64,
            "events": self.events,
        }


class AmpPipeline:
    """Orchestrate explicitly labelled mock sensors for controlled experiments."""

    def __init__(self, cfg: PipelineConfig | None = None) -> None:
        self.cfg = cfg or PipelineConfig()
        self.inventory = discover_hardware()
        self.robot = MockRobot()

        cam_idx = self.cfg.camera_index
        if cam_idx is None:
            cam_idx = self.inventory.primary_camera_index

        self.webcam: WebcamCapture | None = None
        self.camera_live = False
        if not self.cfg.force_mock_camera and cam_idx is not None:
            self.webcam = WebcamCapture(
                WebcamConfig(
                    index=cam_idx,
                    width=self.cfg.camera_width,
                    height=self.cfg.camera_height,
                )
            )
            self.camera_live = bool(self.webcam.available)
        self.active_camera_index = cam_idx if self.camera_live else None
        self.camera_mock = bool(self.cfg.force_mock_camera)

        # Real LD19 I/O intentionally lives in demo/ld19_live.py. This harness
        # must not turn "some serial port exists" into mock points labelled LIVE.
        self.lidar_live = False
        self.lidar_mock = bool(self.cfg.force_mock_lidar)
        self.lidar_available = self.lidar_live or self.lidar_mock

        det_backend = self.cfg.detector_backend
        if det_backend == "auto" and not (self.camera_live or self.camera_mock):
            det_backend = "stub"
        self.detector = create_detector(
            DetectorConfig(
                backend=det_backend,
                conf_threshold=0.50,
                iou_threshold=0.50,
                model_path="yolov8n.pt",
            )
        )
        self.tracker = MultiObjectTracker(
            TrackerConfig(
                min_hits=2,
                max_age=20,
                iou_threshold=0.30,
                dynamic_speed_mps=0.45,
                dynamic_confirm_frames=12,
                person_containment_iou=0.20,
            )
        )
        self.reliability = create_reliability_estimator(ReliabilityConfig())
        self.cross_modal = CrossModalMonitor()
        # Prefer camera-only fusion weights when no LiDAR
        fusion_mode = self.cfg.fusion_mode
        if not self.lidar_available and fusion_mode == FusionMode.ADAPTIVE_FUSION:
            fusion_mode = FusionMode.CAMERA_ONLY
        fusion_cfg = FusionConfig(mode=fusion_mode)
        self.ekf = AdaptiveEKF(fusion_cfg)
        self.dyn = DynamicObstacleFilter(
            DynamicFilterConfig(
                enabled=self.cfg.dynamic_filtering and self.lidar_available
            )
        )
        self.slam = create_slam(
            SlamConfig(dynamic_filtering=self.cfg.dynamic_filtering)
        )
        self.nav = SimpleNavigator(NavigationConfig())
        # Soften safety when LiDAR absent so webcam-only PC mode is usable
        safety_cfg = SafetyConfig()
        if not self.lidar_available:
            safety_cfg.lidar_timeout_s = 1e9
            safety_cfg.emergency_stop_distance = 0.05
            safety_cfg.min_obstacle_distance = 0.05
        self.safety = SafetySupervisor(safety_cfg)
        self.intrinsics = CameraIntrinsics(
            fx=500.0,
            fy=500.0,
            cx=self.cfg.camera_width / 2,
            cy=self.cfg.camera_height / 2,
            width=self.cfg.camera_width,
            height=self.cfg.camera_height,
        )
        self.extrinsics = ExtrinsicTransform.lidar_to_camera_optical(
            0.05, 0.0, 0.10, 0.0, 0.0, 0.0
        )
        self._t_last = time.monotonic()
        self.events: list[str] = []
        self.last_jpeg_b64: str | None = None
        self._det_fps = 0.0
        self._det_frames = 0
        self._det_t0 = time.monotonic()

        self.events.append(
            f"sources: camera={('local_live idx=' + str(cam_idx)) if self.camera_live else ('mock' if self.camera_mock else 'none')} "
            f"lidar={'mock' if self.lidar_mock else 'none'} "
            f"detector={self.detector.name()}"
        )
        if self.inventory.has_lidar and not self.lidar_mock:
            self.events.append(
                "serial port detected but real LD19 is not bound in this harness; use app.py"
            )
        for r in self.inventory.recommendations:
            self.events.append(f"recommend: {r}")

    def set_cmd(self, linear: float, angular: float) -> None:
        self.robot.cmd = Twist2D(linear, angular)

    def set_goal(self, x: float, y: float, yaw: float = 0.0) -> None:
        self.nav.set_goal(Pose2D(x, y, yaw))

    def close(self) -> None:
        if self.webcam is not None:
            self.webcam.release()

    def _vision_sectors_from_objects(
        self, objects: list[dict], default: float = 12.0
    ) -> dict:
        """Approximate sector mins from monocular object distances (camera-only)."""
        buckets = {
            "front": default,
            "front_left": default,
            "front_right": default,
            "left": default,
            "right": default,
            "rear_left": default,
            "rear": default,
            "rear_right": default,
        }

        def assign(bearing: float, dist: float) -> None:
            a = ((bearing + 180) % 360) - 180
            mapping = [
                ("front", 0, 22.5),
                ("front_left", 45, 22.5),
                ("front_right", -45, 22.5),
                ("left", 90, 22.5),
                ("right", -90, 22.5),
                ("rear_left", 135, 22.5),
                ("rear", 180, 22.5),
                ("rear_right", -135, 22.5),
            ]
            for name, center, half in mapping:
                diff = abs(((a - center + 180) % 360) - 180)
                if diff <= half:
                    buckets[name] = min(buckets[name], dist)

        for o in objects:
            if o.get("distance_m") is None or o.get("bearing_deg") is None:
                continue
            assign(float(o["bearing_deg"]), float(o["distance_m"]))
        return buckets

    def step(self) -> PipelineSnapshot:
        now = time.monotonic()
        dt = max(0.01, min(0.5, now - self._t_last))
        self._t_last = now

        self.safety.heartbeat()
        self.robot.step(dt)

        # ---- Camera ----
        frame = None
        jpeg_b64 = None
        cam_fps = 0.0
        if self.camera_live and self.webcam is not None:
            frame = self.webcam.read()
            cam_fps = self.webcam.fps
            if self.webcam.last_jpeg:
                jpeg_b64 = base64.b64encode(self.webcam.last_jpeg).decode("ascii")
                self.last_jpeg_b64 = jpeg_b64
        elif self.camera_mock:
            frame = self.robot.camera.capture(self.robot.world)
            cam_fps = self.robot.camera.fps

        # ---- LiDAR (explicit simulation only; real LD19 uses demo/perception.py) ----
        projected: list[tuple[float, float, float]] = []
        lidar_pts: list[dict] = []
        lidar_fps = 0.0
        scan = None
        if self.lidar_mock:
            scan_raw = self.robot.lidar.sense(self.robot.world)
            self.safety.note_lidar()
            scan = filter_scan(scan_raw, LidarFilterConfig())
            sectors_obj = sector_distances(scan)
            sectors = sectors_obj.to_dict()
            sectors["source"] = "mock_lidar"
            lidar_fps = self.robot.lidar.fps
            projected = lidar_polar_to_camera(
                scan.ranges, scan.angles, self.extrinsics, self.intrinsics
            )
            lidar_pts = [
                {"x": r * math.cos(a), "y": r * math.sin(a), "r": r}
                for r, a in list(zip(scan.ranges, scan.angles))[::3]
            ]
            feats_lidar = scan_quality_features(scan)
        else:
            sectors = {
                "front": 12.0,
                "front_left": 12.0,
                "front_right": 12.0,
                "left": 12.0,
                "right": 12.0,
                "rear_left": 12.0,
                "rear": 12.0,
                "rear_right": 12.0,
                "timestamp": Timestamp.now().to_dict(),
                "source": "none_lidar_not_connected",
            }
            feats_lidar = {
                "valid_ratio": 0.0,
                "point_density": 0.0,
                "range_jump_rate": 1.0,
                "mean_range": 0.0,
                "temporal_consistency": 0.0,
            }

        # ---- Detection / tracking ----
        dets = []
        if frame is not None:
            # Prefer BGR bytes for DNN if webcam has last_bgr
            if (
                self.camera_live
                and self.webcam is not None
                and self.webcam.last_bgr is not None
            ):
                bgr = self.webcam.last_bgr
                h, w = bgr.shape[:2]
                dets = self.detector.detect(bgr.tobytes(), w, h, 3)
            else:
                dets = self.detector.detect(
                    frame.data, frame.width, frame.height, frame.channels
                )
            self._det_frames += 1
            if now - self._det_t0 >= 1.0:
                self._det_fps = self._det_frames / (now - self._det_t0)
                self._det_frames = 0
                self._det_t0 = now

        tracks = self.tracker.update(dets, dt=dt)
        fw = frame.width if frame else self.cfg.camera_width
        fused_distances = []
        if scan is not None and tracks:
            fused_distances = fuse_detections_distances(
                [(tr.bbox, tr.class_name, tr.confidence) for tr in tracks],
                scan.ranges,
                scan.angles,
                self.extrinsics,
                self.intrinsics,
            )
            self.cross_modal.update(fused_distances)
        for index, tr in enumerate(tracks):
            fused = fused_distances[index] if index < len(fused_distances) else None
            dist = fused.distance_m if fused is not None else None
            if dist is None:
                dist = estimate_distance_m(
                    tr.bbox,
                    tr.class_name,
                    self.intrinsics.fy,
                    frame_height=frame.height if frame else None,
                )
            tr.distance_m = dist
            tr.bearing_deg = (
                fused.bearing_deg
                if fused is not None
                else math.degrees(
                    bearing_from_bbox_center(
                        tr.bbox.cx, fw, self.intrinsics.fx, self.intrinsics.cx
                    )
                )
            )

        objects = [t.to_dict() for t in tracks]
        for index, obj in enumerate(objects):
            fused = fused_distances[index] if index < len(fused_distances) else None
            obj["fusion_consistency"] = (
                fused.consistency_flag if fused is not None else "no_lidar"
            )
            obj["fusion_zscore"] = (
                None if fused is None or fused.z_score is None else round(fused.z_score, 2)
            )
            obj["fusion_variance_m2"] = (
                None
                if fused is None or fused.variance_m2 is None
                else round(fused.variance_m2, 4)
            )
        if not self.lidar_available:
            vis = self._vision_sectors_from_objects(objects)
            sectors.update(vis)
            sectors["source"] = "camera_geometry"

        # ---- Reliability / fusion ----
        if frame is not None:
            feats_cam = compute_vision_features(
                frame.data,
                frame.width,
                frame.height,
                synthetic_feature_grid(frame.width, frame.height, step=50),
            )
        else:
            from amp_core.vision.quality import VisionQualityFeatures

            feats_cam = VisionQualityFeatures(0, 0, 0, 0, 0, 1.0, 1.0)

        conf = self.reliability.estimate(
            lidar_features=feats_lidar,
            camera_features=feats_cam,
            cross_modal_features=self.cross_modal.features(),
        )

        self.ekf.predict(dt)
        self.ekf.set_velocity(
            self.robot.cmd.linear * math.cos(self.robot.world.pose.yaw),
            self.robot.cmd.linear * math.sin(self.robot.world.pose.yaw),
            self.robot.cmd.angular,
        )
        # Visual odometry cue: integrate cmd as odom; camera pose soft-update when live
        self.ekf.update(
            Measurement2D(
                self.robot.world.pose.x,
                self.robot.world.pose.y,
                self.robot.world.pose.yaw,
                "odom",
            ),
            conf,
        )
        if frame is not None:
            self.ekf.update(
                Measurement2D(
                    self.robot.world.pose.x,
                    self.robot.world.pose.y,
                    self.robot.world.pose.yaw,
                    "camera",
                ),
                conf,
            )
        if self.lidar_available and scan is not None:
            self.ekf.update(
                Measurement2D(
                    self.robot.world.pose.x,
                    self.robot.world.pose.y,
                    self.robot.world.pose.yaw,
                    "lidar",
                ),
                conf,
            )
        fusion_state = self.ekf.state()

        if self.lidar_available and scan is not None:
            dyn_res = self.dyn.filter(scan, tracks)
            slam_scan = dyn_res.static_scan if self.cfg.dynamic_filtering else scan
            slam_pose = self.slam.update(slam_scan, fusion_state.pose)
        else:
            empty = LidarScan(ranges=[], angles=[], timestamp=Timestamp.now())
            slam_pose = self.slam.update(empty, fusion_state.pose)

        from amp_core.common.types import SectorDistances

        sector_obj = SectorDistances(
            front=float(sectors.get("front", 12)),
            front_left=float(sectors.get("front_left", 12)),
            front_right=float(sectors.get("front_right", 12)),
            left=float(sectors.get("left", 12)),
            right=float(sectors.get("right", 12)),
            rear_left=float(sectors.get("rear_left", 12)),
            rear=float(sectors.get("rear", 12)),
            rear_right=float(sectors.get("rear_right", 12)),
            timestamp=Timestamp.now(),
        )
        if self.cfg.enable_navigation:
            nav_cmd = self.nav.compute(slam_pose, sector_obj)
        else:
            nav_cmd = self.robot.cmd
        safety = self.safety.filter_command(
            nav_cmd, sector_obj, lidar_healthy=self.lidar_available
        )
        self.robot.cmd = safety.limited

        proj_pts = [{"u": u, "v": v, "r": r} for u, v, r in projected[::5]]

        return PipelineSnapshot(
            timestamp=Timestamp.now(),
            pose=slam_pose.to_dict(),
            twist=safety.limited.to_dict(),
            sectors=sectors,
            objects=objects,
            confidence=conf.to_dict(),
            fusion=fusion_state.to_dict(),
            safety=safety.to_dict(),
            system={
                "cpu_percent": 0.0,
                "ram_percent": 0.0,
                "temperature_c": None,
                "camera_fps": cam_fps,
                "lidar_fps": lidar_fps,
                "detection_fps": self._det_fps,
                "fusion_hz": fusion_state.update_rate_hz,
                "slam_hz": 1.0 / dt,
                "detector": self.detector.name(),
            },
            map_preview=self.slam.get_occupancy(),
            lidar_points=lidar_pts,
            projected_lidar=proj_pts,
            camera_meta={
                "width": frame.width if frame else self.cfg.camera_width,
                "height": frame.height if frame else self.cfg.camera_height,
                "encoding": "jpeg"
                if jpeg_b64
                else (frame.encoding if frame else "none"),
                "fps": cam_fps,
                "source": "local_usb"
                if self.camera_live
                else ("mock" if self.camera_mock else "none"),
            },
            hardware={
                "camera_live": self.camera_live,
                "lidar_live": self.lidar_live,
                "camera_source": "local_usb"
                if self.camera_live
                else ("mock" if self.camera_mock else "none"),
                "lidar_source": "mock" if self.lidar_mock else "none",
                "lidar_mock": self.lidar_mock,
                "camera_index": self.active_camera_index,
                "detector": self.detector.name(),
                "inventory": self.inventory.to_dict(),
            },
            camera_jpeg_b64=jpeg_b64 or self.last_jpeg_b64,
            events=list(self.events[-30:]),
        )
