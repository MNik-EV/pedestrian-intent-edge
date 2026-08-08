"""Background structured logger service."""

from __future__ import annotations

import logging
import time
from datetime import datetime, timezone

from amp_core.logging_io.experiment import JsonlWriter
from pathlib import Path

logging.basicConfig(level=logging.INFO)
log = logging.getLogger("amp.logger")


def main() -> None:
    path = Path("logs/jsonl/runtime_events.jsonl")
    writer = JsonlWriter(path)
    log.info("Logger writing to %s", path)
    while True:
        writer.write(
            {
                "type": "heartbeat",
                "ts": datetime.now(timezone.utc).isoformat(),
            }
        )
        time.sleep(5.0)


if __name__ == "__main__":
    main()
