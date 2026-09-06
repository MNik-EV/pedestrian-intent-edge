# Models

## PC (default)
- **YOLOv8n** (`yolov8n.pt`) via Ultralytics — auto-selected by `detector.backend: auto`
- First run downloads weights (~6 MB)

## Fallbacks
- OpenCV MobileNet-SSD (+ hard filters) if YOLO cannot load
- Do **not** use Face+HOG+DNN combined mode on webcam (duplicate person boxes)

## Pi
Prefer quantized ONNX/TFLite after on-device benchmarking. Keep YOLO for PC research.
