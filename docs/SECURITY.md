# Security

- Do not commit passwords or `AMP_CONTROL_TOKEN`.
- Control APIs require Bearer auth; telemetry can remain read-only.
- Validate teleop magnitudes server-side; safety supervisor is authoritative.
- Rate-limit control endpoints.
- Prefer binding dashboard to LAN; use firewall rules on Pi.
- Keep `/etc/amp-robot.env` mode `600`.
