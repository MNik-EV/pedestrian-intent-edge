#!/usr/bin/env bash
# Record experiment (Linux / Pi). On Windows use: python scripts/record_experiment.py
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export PYTHONPATH="$ROOT:${PYTHONPATH:-}"
python3 "$ROOT/scripts/record_experiment.py" "$@"
