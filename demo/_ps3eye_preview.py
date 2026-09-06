"""Quick live preview of the USB PS3 Eye (default index from demo_hardware.yaml)."""

from __future__ import annotations

import sys
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from demo.defaults import default_camera_index

idx = default_camera_index()
cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
if not cap.isOpened():
    raise SystemExit(f"Cannot open USB camera index {idx}")
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 60)
for _ in range(20):
    cap.read()
win = f"USB PS3 Eye LIVE (index {idx}) - press Q to close"
cv2.namedWindow(win, cv2.WINDOW_NORMAL)
cv2.resizeWindow(win, 960, 720)
n = 0
last = time.time()
fps = 0.0
print(f"USB camera preview index={idx}", flush=True)
while True:
    ok, frame = cap.read()
    if not ok:
        continue
    n += 1
    now = time.time()
    if now - last >= 1.0:
        fps = n / (now - last)
        n = 0
        last = now
    cv2.putText(
        frame,
        f"USB PS3 Eye  idx={idx}  {fps:.0f} FPS",
        (12, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 255, 0),
        2,
    )
    cv2.imshow(win, frame)
    if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
        break
cap.release()
cv2.destroyAllWindows()
