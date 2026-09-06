"""Camera-source configuration tests that do not require physical hardware."""

from __future__ import annotations

import pytest

from demo import camera_source


def test_network_mode_builds_network_capture(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        camera_source,
        "load_demo_hardware",
        lambda: {
            "camera": {
                "mode": "network",
                "network": {"url": "http://pi.test/stream.mjpg", "reconnect_timeout_s": 1.5},
            }
        },
    )
    source = camera_source.open_camera_source()
    assert source.url == "http://pi.test/stream.mjpg"
    assert source.reconnect_timeout_s == 1.5


def test_invalid_camera_mode_is_rejected(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        camera_source,
        "load_demo_hardware",
        lambda: {"camera": {"mode": "bluetooth"}},
    )
    with pytest.raises(ValueError, match="camera.mode"):
        camera_source.open_camera_source()
