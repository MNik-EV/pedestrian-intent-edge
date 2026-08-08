#!/usr/bin/env bash
set -euo pipefail
sudo systemctl start robot.service
systemctl --no-pager status robot.service robot_core.service robot_web.service || true
echo "Dashboard: http://$(hostname -I | awk '{print $1}'):8000"
