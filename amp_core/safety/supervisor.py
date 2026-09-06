"""Independent local safety supervisor (must not depend on dashboard/Wi-Fi/AI)."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import time

from amp_core.common.types import SectorDistances, Timestamp, Twist2D


class SafetyAction(str, Enum):
    ALLOW = "ALLOW"
    SLOW = "SLOW"
    STOP = "STOP"
    ESTOP = "ESTOP"


@dataclass
class SafetyConfig:
    max_linear_speed: float = 0.35
    max_angular_speed: float = 0.8
    min_obstacle_distance: float = 0.35
    emergency_stop_distance: float = 0.25
    warning_distance: float = 0.60
    heartbeat_timeout_s: float = 0.5
    lidar_timeout_s: float = 0.4
    enable_soft_slowdown: bool = True


@dataclass
class SafetyStatus:
    action: SafetyAction
    reason: str
    commanded: Twist2D
    limited: Twist2D
    timestamp: Timestamp
    faults: list[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "action": self.action.value,
            "reason": self.reason,
            "commanded": self.commanded.to_dict(),
            "limited": self.limited.to_dict(),
            "timestamp": self.timestamp.to_dict(),
            "faults": self.faults,
        }


class SafetySupervisor:
    """Hard safety layer between planners/teleop and motor commands."""

    def __init__(self, cfg: SafetyConfig | None = None) -> None:
        self.cfg = cfg or SafetyConfig()
        self._last_heartbeat_mono = time.monotonic()
        self._last_lidar_mono = time.monotonic()
        self._estop_latched = False
        self._faults: list[str] = []

    def heartbeat(self) -> None:
        self._last_heartbeat_mono = time.monotonic()

    def note_lidar(self) -> None:
        self._last_lidar_mono = time.monotonic()

    def clear_estop(self) -> None:
        self._estop_latched = False

    def force_estop(self, reason: str = "manual") -> None:
        self._estop_latched = True
        if reason not in self._faults:
            self._faults.append(reason)

    def filter_command(
        self,
        cmd: Twist2D,
        sectors: SectorDistances | None = None,
        lidar_healthy: bool = True,
        motor_healthy: bool = True,
    ) -> SafetyStatus:
        now = Timestamp.now()
        faults: list[str] = []
        mono = time.monotonic()

        if self._estop_latched:
            return SafetyStatus(
                SafetyAction.ESTOP,
                "latched_estop",
                cmd,
                Twist2D(0.0, 0.0),
                now,
                list(self._faults) or ["latched_estop"],
            )

        if mono - self._last_heartbeat_mono > self.cfg.heartbeat_timeout_s:
            faults.append("heartbeat_timeout")
        if mono - self._last_lidar_mono > self.cfg.lidar_timeout_s:
            faults.append("lidar_timeout")
        if not lidar_healthy:
            faults.append("lidar_unavailable")
        if not motor_healthy:
            faults.append("motor_comm_failure")

        min_dist = sectors.minimum() if sectors is not None else float("inf")
        if min_dist <= self.cfg.emergency_stop_distance:
            faults.append("emergency_obstacle")

        if faults:
            self._faults = faults
            if any(
                f in faults
                for f in (
                    "emergency_obstacle",
                    "lidar_unavailable",
                    "lidar_timeout",
                    "motor_comm_failure",
                    "heartbeat_timeout",
                )
            ):
                self._estop_latched = True
                return SafetyStatus(
                    SafetyAction.ESTOP,
                    faults[0],
                    cmd,
                    Twist2D(0.0, 0.0),
                    now,
                    faults,
                )

        # Speed caps
        lin = max(
            -self.cfg.max_linear_speed, min(self.cfg.max_linear_speed, cmd.linear)
        )
        ang = max(
            -self.cfg.max_angular_speed, min(self.cfg.max_angular_speed, cmd.angular)
        )

        action = SafetyAction.ALLOW
        reason = "ok"
        if sectors is not None and min_dist < self.cfg.warning_distance:
            if self.cfg.enable_soft_slowdown:
                scale = max(
                    0.0,
                    (min_dist - self.cfg.emergency_stop_distance)
                    / max(
                        1e-3,
                        self.cfg.warning_distance - self.cfg.emergency_stop_distance,
                    ),
                )
                lin *= scale
                action = SafetyAction.SLOW
                reason = "warning_zone"
            if min_dist < self.cfg.min_obstacle_distance:
                lin = 0.0
                action = SafetyAction.STOP
                reason = "min_obstacle_distance"

        return SafetyStatus(action, reason, cmd, Twist2D(lin, ang), now, faults)
