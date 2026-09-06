"""Unit tests for the MJPEG-over-HTTP network camera client (no real Pi needed).

Spins up a tiny local http.server that speaks the same multipart/JPEG
protocol as edge/camera_streamer.py, then verifies
amp_core.vision.network_camera.NetworkCameraCapture can connect, decode
frames, and fail cleanly (not crash) against an unreachable host.
"""

from __future__ import annotations

import http.server
import socket
import threading
import time

import cv2
import numpy as np
import pytest

from amp_core.vision.network_camera import NetworkCameraCapture


def _make_jpeg(value: int) -> bytes:
    img = np.full((4, 4, 3), value, dtype=np.uint8)
    ok, buf = cv2.imencode(".jpg", img)
    assert ok
    return buf.tobytes()


_FRAMES = [_make_jpeg(10), _make_jpeg(200)]


class _MjpegHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args) -> None:  # quiet test output
        pass

    def do_GET(self) -> None:  # noqa: N802
        if self.path != "/stream.mjpg":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()
        try:
            for _ in range(50):
                for frame in _FRAMES:
                    self.wfile.write(b"--FRAME\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    time.sleep(0.02)
        except (BrokenPipeError, ConnectionResetError):
            pass


@pytest.fixture()
def mjpeg_server():
    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), _MjpegHandler)
    server.daemon_threads = True
    port = server.server_address[1]
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{port}/stream.mjpg"
    finally:
        server.shutdown()
        thread.join(timeout=2.0)


def test_network_camera_receives_and_decodes_frames(mjpeg_server):
    cam = NetworkCameraCapture(mjpeg_server, reconnect_timeout_s=0.2)
    cam.start()
    try:
        frame = cam.read()
        assert frame.bgr is not None
        assert frame.bgr.shape == (4, 4, 3)
        deadline = time.monotonic() + 2.0
        while cam.fps <= 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        assert cam.connected is True
        assert cam.last_error is None
        assert cam.fps > 0
    finally:
        cam.stop()
    assert cam.connected is False


def test_network_camera_unreachable_raises_without_crashing():
    # Reserve a free port but don't bind a server to it.
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()

    cam = NetworkCameraCapture(
        f"http://127.0.0.1:{port}/stream.mjpg",
        reconnect_timeout_s=0.2,
        connect_timeout_s=0.3,
        first_frame_timeout_s=0.6,
    )
    with pytest.raises(RuntimeError):
        cam.start()
    assert cam.connected is False
    assert cam._running is False
    cam.stop()
