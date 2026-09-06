"""Trajectory error metrics (ATE / RPE) — no fabricated results."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class PoseSample:
    t: float
    x: float
    y: float
    yaw: float


def align_umeyama_2d(
    est: list[PoseSample], gt: list[PoseSample]
) -> tuple[float, float, float]:
    """Similarity alignment (yaw, tx, ty) for ATE. Returns (yaw, tx, ty)."""
    n = min(len(est), len(gt))
    if n < 2:
        return 0.0, 0.0, 0.0
    ex = sum(p.x for p in est[:n]) / n
    ey = sum(p.y for p in est[:n]) / n
    gx = sum(p.x for p in gt[:n]) / n
    gy = sum(p.y for p in gt[:n]) / n
    # Cross-covariance for rotation
    sxx = syy = sxy = syx = 0.0
    for i in range(n):
        e_x = est[i].x - ex
        e_y = est[i].y - ey
        g_x = gt[i].x - gx
        g_y = gt[i].y - gy
        sxx += e_x * g_x
        sxy += e_x * g_y
        syx += e_y * g_x
        syy += e_y * g_y
    yaw = math.atan2(sxy - syx, sxx + syy)
    c, s = math.cos(yaw), math.sin(yaw)
    tx = gx - (c * ex - s * ey)
    ty = gy - (s * ex + c * ey)
    return yaw, tx, ty


def absolute_trajectory_error(
    est: list[PoseSample], gt: list[PoseSample]
) -> dict[str, float]:
    n = min(len(est), len(gt))
    if n == 0:
        return {"ate_rmse": float("nan"), "ate_mean": float("nan"), "n": 0.0}
    yaw, tx, ty = align_umeyama_2d(est, gt)
    c, s = math.cos(yaw), math.sin(yaw)
    errs = []
    for i in range(n):
        ax = c * est[i].x - s * est[i].y + tx
        ay = s * est[i].x + c * est[i].y + ty
        errs.append(math.hypot(ax - gt[i].x, ay - gt[i].y))
    mean = sum(errs) / n
    rmse = math.sqrt(sum(e * e for e in errs) / n)
    return {"ate_rmse": rmse, "ate_mean": mean, "n": float(n)}


def relative_pose_error(
    est: list[PoseSample], gt: list[PoseSample], delta: int = 1
) -> dict[str, float]:
    n = min(len(est), len(gt))
    if n <= delta:
        return {"rpe_rmse": float("nan"), "rpe_mean": float("nan"), "n": 0.0}
    errs = []
    for i in range(n - delta):
        de_x = est[i + delta].x - est[i].x
        de_y = est[i + delta].y - est[i].y
        dg_x = gt[i + delta].x - gt[i].x
        dg_y = gt[i + delta].y - gt[i].y
        errs.append(math.hypot(de_x - dg_x, de_y - dg_y))
    mean = sum(errs) / len(errs)
    rmse = math.sqrt(sum(e * e for e in errs) / len(errs))
    return {"rpe_rmse": rmse, "rpe_mean": mean, "n": float(len(errs))}
