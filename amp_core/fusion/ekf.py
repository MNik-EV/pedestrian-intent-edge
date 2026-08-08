"""Adaptive EKF-style sensor fusion with bounded covariance adaptation."""

from __future__ import annotations

import math
from dataclasses import dataclass

from amp_core.common.types import (
    Covariance2D,
    FusionMode,
    FusionState,
    Pose2D,
    SensorConfidence,
    Timestamp,
    Twist2D,
)
from amp_core.calibration.transforms import wrap_angle


@dataclass
class FusionConfig:
    mode: FusionMode = FusionMode.ADAPTIVE_FUSION
    # Base measurement variances (meters^2 / rad^2)
    base_r_lidar: float = 0.05
    base_r_camera: float = 0.12
    base_r_odom: float = 0.08
    base_r_imu_yaw: float = 0.02
    # Process noise
    q_pos: float = 0.02
    q_yaw: float = 0.01
    q_vel: float = 0.05
    # Covariance adaptation bounds
    conf_floor: float = 0.05
    conf_ceil: float = 1.0
    r_scale_min: float = 0.5
    r_scale_max: float = 50.0
    # Fixed-mode weights
    fixed_w_lidar: float = 0.6
    fixed_w_camera: float = 0.2
    fixed_w_odom: float = 0.15
    fixed_w_imu: float = 0.05


def bounded_r_scale(confidence: float, cfg: FusionConfig) -> float:
    """Map confidence to a bounded measurement covariance scale.

    High confidence -> scale near r_scale_min (trust measurement).
    Low confidence -> scale near r_scale_max (down-weight measurement).
    Numerically stable; prevents covariance explosion.
    """
    c = max(cfg.conf_floor, min(cfg.conf_ceil, confidence))
    # Inverse relationship with soft clamp
    raw = 1.0 / c
    return max(cfg.r_scale_min, min(cfg.r_scale_max, raw))


@dataclass
class Measurement2D:
    x: float
    y: float
    yaw: float | None
    source: str  # lidar | camera | odom | imu


class AdaptiveEKF:
    """2D pose EKF with optional adaptive measurement weighting."""

    def __init__(self, cfg: FusionConfig | None = None) -> None:
        self.cfg = cfg or FusionConfig()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.yaw_rate = 0.0
        self.Pxx = 1.0
        self.Pyy = 1.0
        self.Pyaw = 0.5
        self._last_update = Timestamp.now()
        self._updates = 0
        self._window_start = self._last_update.mono_ns
        self.last_innovation: dict[str, float] = {}
        self.last_weights: dict[str, float] = {
            "lidar": 0.0,
            "camera": 0.0,
            "odom": 0.0,
            "imu": 0.0,
        }

    def predict(self, dt: float) -> None:
        dt = max(1e-4, min(1.0, dt))
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.yaw = wrap_angle(self.yaw + self.yaw_rate * dt)
        self.Pxx += self.cfg.q_pos * dt
        self.Pyy += self.cfg.q_pos * dt
        self.Pyaw += self.cfg.q_yaw * dt

    def _weight_for(self, source: str, conf: SensorConfidence) -> float:
        mode = self.cfg.mode
        if mode == FusionMode.LIDAR_ONLY:
            return 1.0 if source == "lidar" else 0.0
        if mode == FusionMode.CAMERA_ONLY:
            return 1.0 if source == "camera" else 0.0
        if mode == FusionMode.FIXED_FUSION:
            return {
                "lidar": self.cfg.fixed_w_lidar,
                "camera": self.cfg.fixed_w_camera,
                "odom": self.cfg.fixed_w_odom,
                "imu": self.cfg.fixed_w_imu,
            }.get(source, 0.0)
        # Adaptive modes
        return {
            "lidar": conf.lidar,
            "camera": conf.camera,
            "odom": conf.odom,
            "imu": conf.imu,
        }.get(source, 0.0)

    def _base_r(self, source: str) -> float:
        return {
            "lidar": self.cfg.base_r_lidar,
            "camera": self.cfg.base_r_camera,
            "odom": self.cfg.base_r_odom,
            "imu": self.cfg.base_r_imu_yaw,
        }.get(source, 0.1)

    def update(self, meas: Measurement2D, conf: SensorConfidence) -> None:
        w = self._weight_for(meas.source, conf)
        self.last_weights[meas.source] = w
        if w <= 1e-6:
            return

        adaptive = self.cfg.mode in {
            FusionMode.ADAPTIVE_FUSION,
            FusionMode.ADAPTIVE_FUSION_DYNFILTER,
        }
        scale = bounded_r_scale(w, self.cfg) if adaptive else 1.0 / max(w, self.cfg.conf_floor)
        scale = max(self.cfg.r_scale_min, min(self.cfg.r_scale_max, scale))
        R = self._base_r(meas.source) * scale

        # Scalar Kalman updates for x, y (and yaw if present)
        for state_attr, meas_val, p_attr in (
            ("x", meas.x, "Pxx"),
            ("y", meas.y, "Pyy"),
        ):
            x = getattr(self, state_attr)
            P = getattr(self, p_attr)
            innov = meas_val - x
            S = P + R
            if S <= 1e-12:
                continue
            K = P / S
            # Sanity: clamp gain
            K = max(0.0, min(1.0, K))
            setattr(self, state_attr, x + K * innov)
            setattr(self, p_attr, max(1e-6, (1.0 - K) * P))
            self.last_innovation[f"{meas.source}_{state_attr}"] = innov

        if meas.yaw is not None:
            innov = wrap_angle(meas.yaw - self.yaw)
            S = self.Pyaw + R
            if S > 1e-12:
                K = max(0.0, min(1.0, self.Pyaw / S))
                self.yaw = wrap_angle(self.yaw + K * innov)
                self.Pyaw = max(1e-6, (1.0 - K) * self.Pyaw)
                self.last_innovation[f"{meas.source}_yaw"] = innov

        self._updates += 1
        self._last_update = Timestamp.now()

    def set_velocity(self, vx: float, vy: float, yaw_rate: float) -> None:
        self.vx = vx
        self.vy = vy
        self.yaw_rate = yaw_rate

    def state(self) -> FusionState:
        now = Timestamp.now()
        elapsed = (now.mono_ns - self._window_start) / 1e9
        rate = self._updates / elapsed if elapsed > 0.2 else 0.0
        if elapsed > 2.0:
            self._updates = 0
            self._window_start = now.mono_ns
        return FusionState(
            pose=Pose2D(self.x, self.y, self.yaw),
            twist=Twist2D(math.hypot(self.vx, self.vy), self.yaw_rate),
            covariance=Covariance2D(self.Pxx, self.Pyy, self.Pyaw),
            sensor_weights=dict(self.last_weights),
            innovation=dict(self.last_innovation),
            update_rate_hz=rate,
            mode=self.cfg.mode,
            timestamp=now,
        )
