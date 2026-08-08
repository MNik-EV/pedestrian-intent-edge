# API

Base URL: `http://<host>:8000`

## Read (no auth required by default)

- `GET /api/status`
- `GET /api/robot`
- `GET /api/sensors`
- `GET /api/objects`
- `GET /api/map`
- `GET /api/logs`
- `GET /api/experiments`

## Control (Bearer token required)

Header: `Authorization: Bearer <AMP_CONTROL_TOKEN>`

- `POST /api/navigation/goal` `{ "x", "y", "yaw" }`
- `POST /api/navigation/stop`
- `POST /api/navigation/clear_estop`
- `POST /api/teleop` `{ "linear", "angular" }` — **always safety-filtered**
- `POST /api/experiment/start`
- `POST /api/experiment/stop`

## WebSocket

- `/ws/telemetry` — full live snapshot (~10 Hz)
- `/ws/events` — event stream

Rate limiting applies to teleop/goal endpoints.
