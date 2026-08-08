"""Pi runtime entry: keep safety heartbeat alive (ROS2-optional)."""

from __future__ import annotations

import logging
import time

from amp_core.pipeline import AmpPipeline, PipelineConfig
from amp_core.common.types import FusionMode

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("amp.runtime_core")


def main() -> None:
    pipe = AmpPipeline(PipelineConfig(fusion_mode=FusionMode.ADAPTIVE_FUSION, enable_navigation=True))
    log.info("AMP core runtime started")
    while True:
        snap = pipe.step()
        pipe.safety.heartbeat()
        if snap.safety.get("action") == "ESTOP":
            log.warning("ESTOP: %s", snap.safety.get("reason"))
        time.sleep(0.05)


if __name__ == "__main__":
    main()
