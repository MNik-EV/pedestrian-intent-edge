#!/usr/bin/env bash
# PC development setup (Ubuntu recommended for full ROS2; Windows uses mock stack).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
python3 -m pip install -U pip
python3 -m pip install -r "$ROOT/requirements.txt"
python3 -m pip install -e "$ROOT"
echo "Dev setup complete. Run: python scripts/run_mock_stack.py"
echo "For ROS2 Jazzy (Ubuntu 24.04 / Pi): follow docs/INSTALLATION.md"
