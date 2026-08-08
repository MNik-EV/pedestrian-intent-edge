# Deployment

## Release build (PC / CI)

```bash
./scripts/build_release.sh
# → dist/robot_release_<version>.tar.gz
```

## Pi install order

1. `sudo ./install.sh` — venv, deps, copy app, enable systemd  
2. `sudo ./configure.sh` — generate `/etc/amp-robot.env` token  
3. `./health_check.sh`  
4. `sudo systemctl start robot.service`  
5. `systemctl status robot_core robot_web robot_logger robot_monitor`

## Services

| Unit | Role |
|------|------|
| `robot.service` | Aggregate |
| `robot_core.service` | Perception / fusion / safety loop |
| `robot_web.service` | Dashboard + API |
| `robot_logger.service` | Structured logs |
| `robot_monitor.service` | CPU/RAM/temp |

Restarts use `Restart=on-failure` with `StartLimitBurst` to avoid restart loops.

## Security

- Control endpoints require `Authorization: Bearer <AMP_CONTROL_TOKEN>`
- Token stored in `/etc/amp-robot.env` (mode 600), never in git
- Telemetry read APIs remain separable from control
