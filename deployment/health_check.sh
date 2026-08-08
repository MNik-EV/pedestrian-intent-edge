#!/usr/bin/env bash
set -euo pipefail
echo "AMP health check"
python3 - <<'PY'
import json, pathlib, sys
sys.path.insert(0, "/opt/amp-robot/app")
try:
    from amp_core import __version__
    print("amp_core", __version__)
except Exception as e:
    print("amp_core import failed", e)
    sys.exit(1)
print("OK")
PY
