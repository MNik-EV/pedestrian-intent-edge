"""Geometric transforms and LiDAR–camera projection utilities."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    dist_coeffs: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)

    def project(self, X: float, Y: float, Z: float) -> tuple[float, float] | None:
        """Project a 3D camera-frame point to pixel coordinates."""
        if Z <= 1e-6:
            return None
        u = self.fx * (X / Z) + self.cx
        v = self.fy * (Y / Z) + self.cy
        if u < 0 or v < 0 or u >= self.width or v >= self.height:
            return None
        return u, v


@dataclass
class ExtrinsicTransform:
    """Rigid transform from source frame to target frame (rotation + translation)."""

    # Row-major 3x3 rotation
    r00: float = 1.0
    r01: float = 0.0
    r02: float = 0.0
    r10: float = 0.0
    r11: float = 1.0
    r12: float = 0.0
    r20: float = 0.0
    r21: float = 0.0
    r22: float = 1.0
    tx: float = 0.0
    ty: float = 0.0
    tz: float = 0.0

    def transform_point(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        X = self.r00 * x + self.r01 * y + self.r02 * z + self.tx
        Y = self.r10 * x + self.r11 * y + self.r12 * z + self.ty
        Z = self.r20 * x + self.r21 * y + self.r22 * z + self.tz
        return X, Y, Z

    @classmethod
    def from_xyz_rpy(
        cls, x: float, y: float, z: float, roll: float, pitch: float, yaw: float
    ) -> "ExtrinsicTransform":
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        # ZYX yaw-pitch-roll
        r00 = cy * cp
        r01 = cy * sp * sr - sy * cr
        r02 = cy * sp * cr + sy * sr
        r10 = sy * cp
        r11 = sy * sp * sr + cy * cr
        r12 = sy * sp * cr - cy * sr
        r20 = -sp
        r21 = cp * sr
        r22 = cp * cr
        return cls(r00, r01, r02, r10, r11, r12, r20, r21, r22, x, y, z)


def wrap_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    a = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return a


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def lidar_polar_to_camera(
    ranges: Sequence[float],
    angles: Sequence[float],
    lidar_to_camera: ExtrinsicTransform,
    intrinsics: CameraIntrinsics,
    z_plane: float = 0.0,
) -> list[tuple[float, float, float]]:
    """Project LiDAR polar measurements into image pixels.

    Returns list of (u, v, range) for points that land in the image.
    Assumes 2D LiDAR in horizontal plane; z_plane is height of beam in lidar frame.
    """
    projected: list[tuple[float, float, float]] = []
    for r, a in zip(ranges, angles):
        if r != r or r <= 0.0:
            continue
        lx = r * math.cos(a)
        ly = r * math.sin(a)
        lz = z_plane
        X, Y, Z = lidar_to_camera.transform_point(lx, ly, lz)
        pix = intrinsics.project(X, Y, Z)
        if pix is not None:
            projected.append((pix[0], pix[1], r))
    return projected


def bearing_from_bbox_center(
    cx: float, image_width: int, fx: float, principal_x: float | None = None
) -> float:
    """Approximate horizontal bearing (radians) from bbox center."""
    cx0 = principal_x if principal_x is not None else 0.5 * image_width
    return math.atan2((cx - cx0), fx)


def associate_detection_with_lidar(
    bbox_cx: float,
    bbox_cy: float,
    projected_points: Sequence[tuple[float, float, float]],
    pixel_radius: float = 40.0,
) -> float | None:
    """Median range of projected LiDAR points near a detection center."""
    nearby = [
        r
        for u, v, r in projected_points
        if (u - bbox_cx) ** 2 + (v - bbox_cy) ** 2 <= pixel_radius**2
    ]
    if not nearby:
        return None
    nearby_sorted = sorted(nearby)
    return nearby_sorted[len(nearby_sorted) // 2]
