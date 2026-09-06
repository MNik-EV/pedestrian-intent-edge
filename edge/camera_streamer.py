#!/usr/bin/env python3
"""MJPEG-over-HTTP streamer for the IMX219-120 CSI camera on Raspberry Pi Zero 2W.

Runs entirely on the Pi. Captures with picamera2/libcamera (CSI, not USB — the
IMX219-120 has no UVC interface) and serves a low-overhead multipart MJPEG
stream that the laptop reads with plain OpenCV/urllib (see
amp_core/vision/network_camera.py on the laptop side). No AI, no fusion, no
LiDAR here: the Pi Zero 2W is a camera edge relay only — everything else runs
on the laptop.

Usage (on the Pi):
    python3 edge/camera_streamer.py --width 640 --height 480 --fps 15 --port 8000

Then from the laptop / a browser: http://<pi-hostname-or-ip>:8000/
"""

from __future__ import annotations

import argparse
import io
import socket
import threading
from http import server


PAGE = """<!DOCTYPE html>
<html>
<head><title>AMP Camera Edge — Pi Zero 2W</title></head>
<body style="background:#0b1014;color:#e8eef3;font-family:sans-serif;text-align:center">
<h2>AMP Camera Edge (IMX219-120)</h2>
<img src="stream.mjpg" style="max-width:100%;border:1px solid #24303a" />
<p>Raw stream: <a style="color:#2eb7c9" href="stream.mjpg">/stream.mjpg</a></p>
</body>
</html>
"""


class StreamingOutput(io.BufferedIOBase):
    """Holds the latest JPEG frame; wakes up any waiting client on each write."""

    def __init__(self) -> None:
        self.frame: bytes | None = None
        self.condition = threading.Condition()

    def write(self, buf: bytes) -> int:
        with self.condition:
            self.frame = buf
            self.condition.notify_all()
        return len(buf)


class StreamingHandler(server.BaseHTTPRequestHandler):
    output: StreamingOutput  # set on the class before the server starts

    def log_message(self, fmt: str, *args) -> None:  # quieter default logging
        pass

    def do_GET(self) -> None:  # noqa: N802 (stdlib method name)
        if self.path in ("/", "/index.html"):
            body = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=FRAME"
            )
            self.end_headers()
            try:
                while True:
                    with self.output.condition:
                        self.output.condition.wait()
                        frame = self.output.frame
                    if frame is None:
                        continue
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        self.send_error(404)


class StreamingServer(server.ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def main() -> None:
    try:
        from picamera2 import Picamera2
        from picamera2.encoders import MJPEGEncoder, Quality
        from picamera2.outputs import FileOutput
    except ImportError as exc:  # pragma: no cover - only available on a Pi
        raise SystemExit(
            "picamera2 is not installed. On Raspberry Pi OS install it via apt "
            "(not pip): sudo apt install -y python3-picamera2 --no-install-recommends"
        ) from exc

    parser = argparse.ArgumentParser(description="Pi Zero 2W IMX219-120 MJPEG streamer")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15, help="target capture fps")
    parser.add_argument("--quality", type=int, default=70, help="JPEG quality 1-95")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    picam2 = Picamera2()
    frame_duration_us = int(1_000_000 / max(1, args.fps))
    config = picam2.create_video_configuration(
        # Keep Picamera2's encoder-compatible default pixel format, matching
        # the official hardware-MJPEG server example.
        main={"size": (args.width, args.height)},
        controls={"FrameDurationLimits": (frame_duration_us, frame_duration_us)},
    )
    picam2.configure(config)

    output = StreamingOutput()
    quality = {
        95: Quality.VERY_HIGH,
        85: Quality.HIGH,
        70: Quality.MEDIUM,
        50: Quality.LOW,
    }.get(min([95, 85, 70, 50], key=lambda q: abs(q - args.quality)), Quality.MEDIUM)
    picam2.start_recording(MJPEGEncoder(), FileOutput(output), quality=quality)

    StreamingHandler.output = output
    hostname = socket.gethostname()
    print(
        f"AMP camera edge streaming: http://{hostname}.local:{args.port}/  (or the Pi's IP)"
    )
    print(
        f"Capture: {args.width}x{args.height} @ ~{args.fps}fps, quality={args.quality}"
    )

    address = (args.bind, args.port)
    srv = StreamingServer(address, StreamingHandler)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop_recording()


if __name__ == "__main__":
    main()
