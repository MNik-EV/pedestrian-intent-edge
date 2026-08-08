#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
APP=/opt/amp-robot/app
echo "Interactive configuration placeholders:"
echo "  - Edit $APP/config/*.yaml for ports, fusion mode, safety limits"
echo "  - export AMP_CONTROL_TOKEN in /etc/amp-robot.env"
install -m 600 /dev/null /etc/amp-robot.env || true
if ! grep -q AMP_CONTROL_TOKEN /etc/amp-robot.env 2>/dev/null; then
  TOKEN=$(openssl rand -hex 16 2>/dev/null || echo "change-me")
  echo "AMP_CONTROL_TOKEN=$TOKEN" >> /etc/amp-robot.env
  echo "AMP_HOST=0.0.0.0" >> /etc/amp-robot.env
  echo "AMP_PORT=8000" >> /etc/amp-robot.env
  echo "Wrote /etc/amp-robot.env"
fi
"$ROOT/health_check.sh" || true
