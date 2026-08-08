"""Modular SLAM backend interface + lightweight occupancy mapper for mocks."""

from __future__ import annotations

import json
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path

from amp_core.common.types import LidarScan, Pose2D


@dataclass
class SlamConfig:
    backend: str = "occupancy_mock"  # occupancy_mock | slam_toolbox | cartographer
    resolution: float = 0.05
    width_m: float = 20.0
    height_m: float = 20.0
    dynamic_filtering: bool = False
    map_path: str = ""


class SlamBackend(ABC):
    @abstractmethod
    def update(self, scan: LidarScan, pose_hint: Pose2D | None = None) -> Pose2D:
        raise NotImplementedError

    @abstractmethod
    def get_pose(self) -> Pose2D:
        raise NotImplementedError

    @abstractmethod
    def get_occupancy(self) -> dict:
        raise NotImplementedError

    @abstractmethod
    def save_map(self, path: str | Path) -> None:
        raise NotImplementedError

    @abstractmethod
    def load_map(self, path: str | Path) -> None:
        raise NotImplementedError


class OccupancyMockSlam(SlamBackend):
    """Research-friendly occupancy grid for PC mock / integration tests."""

    def __init__(self, cfg: SlamConfig | None = None) -> None:
        self.cfg = cfg or SlamConfig()
        w = int(self.cfg.width_m / self.cfg.resolution)
        h = int(self.cfg.height_m / self.cfg.resolution)
        self.width = w
        self.height = h
        self.origin_x = -self.cfg.width_m / 2
        self.origin_y = -self.cfg.height_m / 2
        self.grid = [0] * (w * h)  # 0 unknown, 1 free, 2 occupied
        self.pose = Pose2D(0.0, 0.0, 0.0)
        self.trajectory: list[tuple[float, float]] = []

    def _world_to_grid(self, x: float, y: float) -> tuple[int, int] | None:
        gx = int((x - self.origin_x) / self.cfg.resolution)
        gy = int((y - self.origin_y) / self.cfg.resolution)
        if gx < 0 or gy < 0 or gx >= self.width or gy >= self.height:
            return None
        return gx, gy

    def update(self, scan: LidarScan, pose_hint: Pose2D | None = None) -> Pose2D:
        if pose_hint is not None:
            self.pose = Pose2D(pose_hint.x, pose_hint.y, pose_hint.yaw)
        self.trajectory.append((self.pose.x, self.pose.y))
        for r, a in zip(scan.ranges, scan.angles):
            if r != r or r <= 0:
                continue
            yaw = self.pose.yaw + a
            ox = self.pose.x + r * math.cos(yaw)
            oy = self.pose.y + r * math.sin(yaw)
            cell = self._world_to_grid(ox, oy)
            if cell:
                gx, gy = cell
                self.grid[gy * self.width + gx] = 2
            # mark free along ray coarsely
            steps = max(1, int(r / self.cfg.resolution))
            for s in range(steps):
                fx = self.pose.x + (r * s / steps) * math.cos(yaw)
                fy = self.pose.y + (r * s / steps) * math.sin(yaw)
                fcell = self._world_to_grid(fx, fy)
                if fcell:
                    fgx, fgy = fcell
                    idx = fgy * self.width + fgx
                    if self.grid[idx] == 0:
                        self.grid[idx] = 1
        return self.get_pose()

    def get_pose(self) -> Pose2D:
        return Pose2D(self.pose.x, self.pose.y, self.pose.yaw)

    def get_occupancy(self) -> dict:
        # Downsample for dashboard (max 100x100 preview)
        step = max(1, self.width // 100)
        preview = []
        for y in range(0, self.height, step):
            row = []
            for x in range(0, self.width, step):
                row.append(self.grid[y * self.width + x])
            preview.append(row)
        return {
            "resolution": self.cfg.resolution * step,
            "width": len(preview[0]) if preview else 0,
            "height": len(preview),
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "data": preview,
            "trajectory": self.trajectory[-500:],
            "pose": self.pose.to_dict(),
        }

    def save_map(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "width": self.width,
            "height": self.height,
            "resolution": self.cfg.resolution,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "grid": self.grid,
        }
        path.write_text(json.dumps(payload), encoding="utf-8")

    def load_map(self, path: str | Path) -> None:
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        self.width = payload["width"]
        self.height = payload["height"]
        self.cfg.resolution = payload["resolution"]
        self.origin_x = payload["origin_x"]
        self.origin_y = payload["origin_y"]
        self.grid = payload["grid"]


def create_slam(cfg: SlamConfig | None = None) -> SlamBackend:
    cfg = cfg or SlamConfig()
    if cfg.backend in {"occupancy_mock", "mock"}:
        return OccupancyMockSlam(cfg)
    # External backends are launched via ROS2 on Pi; fall back to mock on PC
    return OccupancyMockSlam(cfg)
