"""Integration test: mock stack produces coherent telemetry."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from amp_core.common.types import FusionMode
from amp_core.logging_io.experiment import (
    ExperimentLogger,
    ExperimentMetadata,
    config_hash,
    new_experiment_id,
)
from amp_core.pipeline import AmpPipeline, PipelineConfig


def test_experiment_logging_and_pipeline() -> None:
    pipe = AmpPipeline(PipelineConfig(fusion_mode=FusionMode.FIXED_FUSION))
    exp_id = new_experiment_id()
    cfg = {"mode": "fixed_fusion"}
    meta = ExperimentMetadata(
        experiment_id=exp_id,
        start_time="now",
        end_time=None,
        configuration=cfg,
        git_commit="unknown",
        hardware={"mock": True},
        software_version="0.1.0",
        ros2_version="jazzy",
        model_version="stub",
        config_hash=config_hash(cfg),
    )
    logger = ExperimentLogger(ROOT / "experiments", meta)
    for _ in range(5):
        snap = pipe.step()
        logger.log_sample("telemetry", snap.to_dict())
        logger.log_metric("pose_x", snap.pose["x"], method="fixed_fusion", scenario="mock")
    logger.close()
    assert (ROOT / "experiments" / exp_id / "metadata.json").exists()
    assert (ROOT / "experiments" / exp_id / "telemetry.sqlite").exists()
