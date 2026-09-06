#!/usr/bin/env bash
# One-time, repeatable setup for Raspberry Pi Zero 2W + IMX219-120.
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="${HOME}/amp_edge"
SERVICE_DIR="${HOME}/.config/systemd/user"
SERVICE_NAME="amp-camera-stream.service"

if [[ ! -r /proc/device-tree/model ]]; then
  echo "ERROR: this installer must run on Raspberry Pi OS." >&2
  exit 1
fi

MODEL="$(tr -d '\0' </proc/device-tree/model)"
echo "Detected: ${MODEL}"
echo "Installing Raspberry Pi camera packages..."
sudo apt-get update
sudo apt-get install -y --no-install-recommends python3-picamera2 curl

# Desktop images already ship the full rpicam-apps package. Install the Lite variant only
# when no camera CLI exists, avoiding an unnecessary package replacement.
if ! command -v rpicam-hello >/dev/null 2>&1; then
  sudo apt-get install -y --no-install-recommends rpicam-apps-lite
fi

if ! command -v rpicam-hello >/dev/null 2>&1; then
  echo "ERROR: rpicam-hello was not installed." >&2
  exit 1
fi

echo "Checking CSI camera discovery..."
CAMERA_LIST="$(rpicam-hello --list-cameras 2>&1)"
printf '%s\n' "${CAMERA_LIST}"
if grep -qi "no cameras available" <<<"${CAMERA_LIST}"; then
  echo "ERROR: no CSI camera was detected. Power off and reseat both ends of the ribbon cable." >&2
  exit 2
fi

TEST_IMAGE="${HOME}/imx219-camera-test.jpg"
echo "Capturing a headless test image at ${TEST_IMAGE}..."
rpicam-jpeg --nopreview --timeout 2000 --output "${TEST_IMAGE}"
test -s "${TEST_IMAGE}"

echo "Installing AMP edge streamer in ${INSTALL_DIR}..."
install -d -m 755 "${INSTALL_DIR}" "${SERVICE_DIR}"
SOURCE_STREAMER="$(realpath -m "${SCRIPT_DIR}/camera_streamer.py")"
TARGET_STREAMER="$(realpath -m "${INSTALL_DIR}/camera_streamer.py")"
if [[ "${SOURCE_STREAMER}" != "${TARGET_STREAMER}" ]]; then
  install -m 755 "${SOURCE_STREAMER}" "${TARGET_STREAMER}"
else
  chmod 755 "${TARGET_STREAMER}"
fi
install -m 644 "${SCRIPT_DIR}/systemd/${SERVICE_NAME}" "${SERVICE_DIR}/${SERVICE_NAME}"

# Linger lets the per-user service start at boot without an interactive login.
sudo loginctl enable-linger "${USER}"
systemctl --user daemon-reload
systemctl --user enable --now "${SERVICE_NAME}"

echo "Waiting for the MJPEG endpoint..."
for _ in {1..20}; do
  if curl --fail --silent --show-error --max-time 2 http://127.0.0.1:8000/ >/dev/null; then
    break
  fi
  sleep 1
done

if ! curl --fail --silent --show-error --max-time 2 http://127.0.0.1:8000/ >/dev/null; then
  echo "ERROR: camera service did not become healthy." >&2
  systemctl --user --no-pager --full status "${SERVICE_NAME}" || true
  journalctl --user -u "${SERVICE_NAME}" --no-pager -n 80 || true
  exit 3
fi

HOSTNAME_VALUE="$(hostname)"
IP_VALUES="$(hostname -I | xargs)"
echo
echo "Camera setup completed successfully."
echo "Preview: http://${HOSTNAME_VALUE}.local:8000/"
echo "IP address(es): ${IP_VALUES}"
echo "Test image: ${TEST_IMAGE}"
systemctl --user --no-pager --full status "${SERVICE_NAME}"
