#!/usr/bin/env bash
# Raspberry Pi installation entrypoint (run from extracted release).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
if [[ "$EUID" -ne 0 ]]; then
  echo "Run with sudo: sudo ./install.sh"
  exit 1
fi

apt-get update
apt-get install -y python3-pip python3-venv python3-opencv || true

python3 -m venv /opt/amp-robot/venv
/opt/amp-robot/venv/bin/pip install -U pip
/opt/amp-robot/venv/bin/pip install -r "$ROOT/requirements.txt"
/opt/amp-robot/venv/bin/pip install -e "$ROOT"

mkdir -p /opt/amp-robot
rsync -a --delete "$ROOT/" /opt/amp-robot/app/ || cp -a "$ROOT/." /opt/amp-robot/app/

# systemd units
cp "$ROOT/deployment/systemd/"*.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable robot.service robot_web.service robot_logger.service robot_monitor.service

echo "Install complete. Run: sudo ./configure.sh && sudo systemctl start robot.service"
