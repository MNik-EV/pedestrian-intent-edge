# Calibration

## Camera intrinsics

1. Collect chessboard images with the PS3 Eye at operating resolution.
2. Run OpenCV calibrateCamera (or ROS `camera_calibration`).
3. Save to `calibration/camera_intrinsics.yaml` (`fx, fy, cx, cy`, distortion).
4. Point `config/camera.yaml` → `calibration_file`.

**Do not use generic internet intrinsics for quantitative papers.**

## Camera–LiDAR extrinsics

1. Observe a shared calibration target visible to both sensors.
2. Estimate `ExtrinsicTransform` (xyz + rpy) from LiDAR frame to camera frame.
3. Save under `calibration/lidar_camera_extrinsics.yaml`.
4. Verify with live LiDAR overlay on the dashboard.

## Timestamp offset

If systematic lag appears, measure cross-correlation between motion events and store `time_offset_ms` in calibration metadata.

## Wheel odometry / IMU

Calibrate wheel radius, track width, and IMU mounting yaw when hardware is enabled.
