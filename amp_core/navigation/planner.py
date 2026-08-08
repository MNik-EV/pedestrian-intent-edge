"""Simple local navigation helpers (occupancy-free reactive planner)."""

from __future__ import annotations

import math
from dataclasses import dataclass

from amp_core.common.types import Pose2D, SectorDistances, Twist2D
from amp_core.calibration.transforms import wrap_angle


@dataclass
class NavigationConfig:
    max_linear_speed: float = 0.30
    max_angular_speed: float = 0.70
    goal_tolerance_m: float = 0.15
    goal_yaw_tolerance_rad: float = 0.25
    obstacle_inflate_m: float = 0.20


class SimpleNavigator:
    """Go-to-pose with sector-aware reactive avoidance (research baseline)."""

    def __init__(self, cfg: NavigationConfig | None = None) -> None:
        self.cfg = cfg or NavigationConfig()
        self.goal: Pose2D | None = None
        self.active = False

    def set_goal(self, goal: Pose2D) -> None:
        self.goal = goal
        self.active = True

    def stop(self) -> None:
        self.active = False
        self.goal = None

    def compute(self, pose: Pose2D, sectors: SectorDistances | None = None) -> Twist2D:
        if not self.active or self.goal is None:
            return Twist2D(0.0, 0.0)

        dx = self.goal.x - pose.x
        dy = self.goal.y - pose.y
        dist = math.hypot(dx, dy)
        if dist < self.cfg.goal_tolerance_m:
            yaw_err = wrap_angle(self.goal.yaw - pose.yaw)
            if abs(yaw_err) < self.cfg.goal_yaw_tolerance_rad:
                self.active = False
                return Twist2D(0.0, 0.0)
            return Twist2D(
                0.0,
                max(
                    -self.cfg.max_angular_speed,
                    min(self.cfg.max_angular_speed, 1.5 * yaw_err),
                ),
            )

        desired_yaw = math.atan2(dy, dx)
        yaw_err = wrap_angle(desired_yaw - pose.yaw)
        angular = max(
            -self.cfg.max_angular_speed,
            min(self.cfg.max_angular_speed, 2.0 * yaw_err),
        )
        linear = self.cfg.max_linear_speed * max(0.0, 1.0 - abs(yaw_err) / math.pi)

        if sectors is not None:
            if sectors.front < 0.8:
                linear *= 0.3
            if sectors.front_left < 0.6:
                angular -= 0.4
            if sectors.front_right < 0.6:
                angular += 0.4
            if sectors.front < self.cfg.obstacle_inflate_m + 0.15:
                linear = 0.0

        return Twist2D(
            max(-self.cfg.max_linear_speed, min(self.cfg.max_linear_speed, linear)),
            max(-self.cfg.max_angular_speed, min(self.cfg.max_angular_speed, angular)),
        )
