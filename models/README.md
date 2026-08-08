# Models directory

Place detector exports here, e.g. `detector.onnx`.

Pi real-time: quantized / small input (e.g. 320) ONNX or TFLite.  
PC research: heavier checkpoints for offline experiments.

Benchmark on the actual Raspberry Pi 5 before locking a model:

```bash
python -c "from amp_core.detection.backends import *; print(benchmark_detector(create_detector(DetectorConfig(backend='stub'))))"
```

Do not claim FPS numbers without measuring on device.
