"""Reproducibility metadata tests for live experiment logging."""

from __future__ import annotations

import json

from demo.logger import DemoLogger


def test_demo_logger_persists_reproducibility_context(tmp_path) -> None:
    logger = DemoLogger(root=tmp_path, prefix="TEST")
    logger.update_context(camera_source="synthetic-test", lidar_port="none")
    logger.log({"objects": []})
    logger.close()

    metadata = json.loads((logger.dir / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["git_commit"]
    assert metadata["software"]["amp_version"]
    assert metadata["runtime"]["camera_source"] == "synthetic-test"
    assert metadata["num_records"] == 1
    assert any(
        item["path"] == "config/demo_hardware.yaml" for item in metadata["artifacts"]
    )
