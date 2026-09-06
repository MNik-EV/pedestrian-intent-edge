"""Camera source factory: Pi Zero 2W network stream (default) or USB fallback.

Both branches return an object exposing start()/stop()/read() where read()
yields something with .bgr and .fps — demo/camera_live.py's LiveCamera and
amp_core/vision/network_camera.py's NetworkCameraCapture already share that
shape, so callers (demo/perception.py) don't need to know which is active.
"""

from __future__ import annotations

from typing import Any

from amp_core.vision.network_camera import NetworkCameraCapture
from demo.camera_live import LiveCamera
from demo.defaults import load_demo_hardware


def open_camera_source(
    camera_index: int | None = None,
    camera_url: str | None = None,
    width: int = 640,
    height: int = 480,
) -> Any:
    """Build the configured camera source.

    Explicit args (camera_index / camera_url) override config/demo_hardware.yaml,
    letting app.py/demo_show.py CLI flags force one mode or the other for
    quick bench testing without editing the config file. width/height only
    apply to the USB path — the network path's resolution is whatever
    edge/camera_streamer.py was started with on the Pi, which should be kept
    in sync with the calibrated intrinsics' image_width/image_height.
    """
    cfg = load_demo_hardware()
    cam_cfg = cfg.get("camera", {}) or {}
    mode = cam_cfg.get("mode", "network")

    if camera_url:
        mode = "network"
    elif camera_index is not None:
        mode = "usb"

    mode = str(mode).strip().lower()
    if mode not in {"network", "usb"}:
        raise ValueError(
            f"Unsupported camera.mode={mode!r}; expected 'network' or 'usb' in "
            "config/demo_hardware.yaml"
        )

    if mode == "network":
        url = camera_url or (cam_cfg.get("network", {}) or {}).get("url")
        if not url:
            raise RuntimeError(
                "camera.mode is 'network' but no camera.network.url is set in "
                "config/demo_hardware.yaml (expected e.g. http://raspberrypi.local:8000/stream.mjpg)"
            )
        timeout = float((cam_cfg.get("network", {}) or {}).get("reconnect_timeout_s", 3.0))
        return NetworkCameraCapture(url=url, reconnect_timeout_s=timeout)

    return LiveCamera(index=camera_index, width=width, height=height)
