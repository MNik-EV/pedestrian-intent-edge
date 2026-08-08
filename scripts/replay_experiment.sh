#!/usr/bin/env bash
# Replay experiment telemetry offline (no hardware).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EXP="${1:?usage: replay_experiment.sh experiments/EXP_xxx}"
export PYTHONPATH="$ROOT:${PYTHONPATH:-}"
python3 - <<PY
import json, sqlite3, sys
from pathlib import Path
exp = Path(r"""$EXP""")
db = exp / "telemetry.sqlite"
if not db.exists():
    print("No telemetry.sqlite in", exp)
    sys.exit(1)
conn = sqlite3.connect(db)
rows = conn.execute("SELECT wall_ns, topic, payload FROM samples ORDER BY id").fetchall()
print(f"Replaying {len(rows)} samples from {exp.name}")
for wall_ns, topic, payload in rows[:5]:
    data = json.loads(payload)
    print(wall_ns, topic, list(data.keys())[:8])
print("...")
print("Offline replay OK. Use evaluation/run_experiment.py for metrics.")
PY
