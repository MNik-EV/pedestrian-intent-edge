#!/usr/bin/env bash
# Produce dist/robot_release_<version>.tar.gz for Raspberry Pi deployment.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
VERSION="${AMP_VERSION:-0.1.0}"
OUT="$ROOT/dist"
STAGE="$OUT/robot_release_$VERSION"
rm -rf "$STAGE"
mkdir -p "$STAGE"

copy_tree() {
  local src="$1" dst="$2"
  mkdir -p "$dst"
  cp -a "$src/." "$dst/"
}

copy_tree "$ROOT/amp_core" "$STAGE/amp_core"
copy_tree "$ROOT/config" "$STAGE/config"
copy_tree "$ROOT/web_dashboard" "$STAGE/web_dashboard"
copy_tree "$ROOT/scripts" "$STAGE/scripts"
copy_tree "$ROOT/deployment" "$STAGE/deployment"
copy_tree "$ROOT/docs" "$STAGE/docs"
copy_tree "$ROOT/calibration" "$STAGE/calibration"
copy_tree "$ROOT/models" "$STAGE/models"
copy_tree "$ROOT/ros2_ws/src" "$STAGE/ros2_ws/src"
copy_tree "$ROOT/evaluation" "$STAGE/evaluation"
copy_tree "$ROOT/paper" "$STAGE/paper"

cp "$ROOT/requirements.txt" "$STAGE/"
cp "$ROOT/pyproject.toml" "$STAGE/"
cp "$ROOT/README.md" "$STAGE/"
cp "$ROOT/LICENSE" "$STAGE/" 2>/dev/null || true
cp "$ROOT/FINAL_SYSTEM_REPORT.md" "$STAGE/" 2>/dev/null || true

# Exclude caches / datasets
find "$STAGE" -type d -name "__pycache__" -prune -exec rm -rf {} + 2>/dev/null || true
find "$STAGE" -type d -name "node_modules" -prune -exec rm -rf {} + 2>/dev/null || true

mkdir -p "$OUT"
tar -C "$OUT" -czf "$OUT/robot_release_${VERSION}.tar.gz" "robot_release_$VERSION"
echo "Created $OUT/robot_release_${VERSION}.tar.gz"
