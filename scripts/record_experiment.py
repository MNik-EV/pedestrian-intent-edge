#!/usr/bin/env python3
"""Record experiment (cross-platform)."""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from amp_core.common.types import FusionMode
from amp_core.logging_io.experiment import (
    ExperimentLogger,
    ExperimentMetadata,
    config_hash,
    new_experiment_id,
)
from amp_core.pipeline import AmpPipeline, PipelineConfig


def git_commit() -> str:
    try:
        return (
            subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT)
            .decode()
            .strip()
        )
    except Exception:
        return "unknown"


def main() -> int:
    ap = argparse.ArgumentParser(description="Record AMP experiment")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--mode", default="adaptive_fusion")
    ap.add_argument("--scenario", default="normal")
    args = ap.parse_args()

    cfg = {"fusion_mode": args.mode, "scenario": args.scenario}
    exp_id = new_experiment_id()
    meta = ExperimentMetadata(
        experiment_id=exp_id,
        start_time=datetime.now(timezone.utc).isoformat(),
        end_time=None,
        configuration=cfg,
        git_commit=git_commit(),
        hardware={"source": "mock_or_runtime"},
        software_version="0.1.0",
        ros2_version="jazzy",
        model_version="stub",
        config_hash=config_hash(cfg),
    )
    logger = ExperimentLogger(ROOT / "experiments", meta)
    pipe = AmpPipeline(PipelineConfig(fusion_mode=FusionMode(args.mode)))
    t_end = time.monotonic() + args.seconds
    n = 0
    while time.monotonic() < t_end:
        snap = pipe.step()
        logger.log_sample("telemetry", snap.to_dict())
        logger.log_metric("pose_x", snap.pose["x"], method=args.mode, scenario=args.scenario)
        logger.log_metric(
            "lidar_confidence",
            float(snap.confidence["lidar"]),
            method=args.mode,
            scenario=args.scenario,
        )
        n += 1
        time.sleep(0.1)
    logger.close()
    print(f"Recorded {n} samples -> {ROOT / 'experiments' / exp_id}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
