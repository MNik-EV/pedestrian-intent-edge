"""Mock sensors for PC development without physical hardware."""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass

from amp_core.common.types import ImageFrame, LidarScan, Pose2D, Timestamp, Twist2D


@dataclass
class MockWorld:
    pose: Pose2D
    obstacles: list[tuple[float, float]]  # world x,y static points
    pedestrians: list[tuple[float, float, float, float]]  # x,y,vx,vy


class MockLidar:
    def __init__(self, num_beams: int = 360, max_range: float = 8.0) -> None:
        self.num_beams = num_beams
        self.max_range = max_range
        self._fps_t0 = time.monotonic()
        self._frames = 0
        self.fps = 0.0

    def sense(self, world: MockWorld) -> LidarScan:
        ranges: list[float] = []
        angles: list[float] = []
        for i in range(self.num_beams):
            a = -math.pi + (2 * math.pi) * i / self.num_beams
            yaw = world.pose.yaw + a
            # Ray cast against point obstacles (coarse)
            best = self.max_range
            for ox, oy in world.obstacles:
                dx = ox - world.pose.x
                dy = oy - world.pose.y
                # project onto ray
                proj = dx * math.cos(yaw) + dy * math.sin(yaw)
                if proj <= 0:
                    continue
                cross = abs(-dx * math.sin(yaw) + dy * math.cos(yaw))
                if cross < 0.15:
                    best = min(best, proj)
            for px, py, _, _ in world.pedestrians:
                dx = px - world.pose.x
                dy = py - world.pose.y
                proj = dx * math.cos(yaw) + dy * math.sin(yaw)
                if proj <= 0:
                    continue
                cross = abs(-dx * math.sin(yaw) + dy * math.cos(yaw))
                if cross < 0.25:
                    best = min(best, proj)
            ranges.append(best)
            angles.append(a)
        self._frames += 1
        now = time.monotonic()
        if now - self._fps_t0 >= 1.0:
            self.fps = self._frames / (now - self._fps_t0)
            self._frames = 0
            self._fps_t0 = now
        return LidarScan(ranges=ranges, angles=angles, timestamp=Timestamp.now())


class MockCamera:
    def __init__(self, width: int = 640, height: int = 480) -> None:
        self.width = width
        self.height = height
        self._t0 = time.monotonic()
        self._frames = 0
        self.fps = 0.0

    def capture(self, world: MockWorld) -> ImageFrame:
        # Synthetic grayscale: floor gradient + blobs for pedestrians
        buf = bytearray(self.width * self.height)
        for y in range(self.height):
            for x in range(self.width):
                buf[y * self.width + x] = 40 + (x + y) % 50
        # Project pedestrians roughly to image center band
        for i, (px, py, _, _) in enumerate(world.pedestrians):
            dx = px - world.pose.x
            dy = py - world.pose.y
            # body frame
            bx = math.cos(-world.pose.yaw) * dx - math.sin(-world.pose.yaw) * dy
            by = math.sin(-world.pose.yaw) * dx + math.cos(-world.pose.yaw) * dy
            if bx <= 0.2:
                continue
            u = int(self.width / 2 + (by / bx) * 320)
            v = int(self.height * 0.55)
            for yy in range(max(0, v - 40), min(self.height, v + 40)):
                for xx in range(max(0, u - 20), min(self.width, u + 20)):
                    buf[yy * self.width + xx] = 200
        self._frames += 1
        now = time.monotonic()
        if now - self._t0 >= 1.0:
            self.fps = self._frames / (now - self._t0)
            self._frames = 0
            self._t0 = now
        return ImageFrame(
            width=self.width,
            height=self.height,
            channels=1,
            data=bytes(buf),
            encoding="mono8",
            timestamp=Timestamp.now(),
            fps=self.fps,
        )


class MockRobot:
    """Integrates twist commands into a simple unicycle model."""

    def __init__(self) -> None:
        self.world = MockWorld(
            pose=Pose2D(0.0, 0.0, 0.0),
            obstacles=[(2.0, 0.3), (2.5, -1.0), (-1.5, 1.2), (1.0, 2.0)],
            pedestrians=[(1.5, 0.0, 0.05, 0.0)],
        )
        self.lidar = MockLidar()
        self.camera = MockCamera()
        self.cmd = Twist2D(0.0, 0.0)

    def step(self, dt: float = 0.1) -> None:
        p = self.world.pose
        p.x += self.cmd.linear * math.cos(p.yaw) * dt
        p.y += self.cmd.linear * math.sin(p.yaw) * dt
        p.yaw += self.cmd.angular * dt
        # Move pedestrians
        updated = []
        for x, y, vx, vy in self.world.pedestrians:
            updated.append((x + vx * dt, y + vy * dt, vx, vy))
        self.world.pedestrians = updated
