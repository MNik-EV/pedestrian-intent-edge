# Methodology

## Reliability features

**LiDAR:** valid_ratio, point_density, range_jump_rate, temporal_consistency, (optional) scan-match residual when available.

**Camera:** feature_count, feature_spread, brightness quality, sharpness, optical_flow_consistency, reprojection error, motion blur proxy.

**IMU / odom (optional):** saturation, noise, slip indicators, disagreement with LiDAR odometry.

## Adaptive fusion

Measurement covariance scale is a **bounded** function of confidence (see `amp_core.fusion.ekf.bounded_r_scale`) to avoid numerical explosion.

## Dynamic filtering

Associate LiDAR returns with tracks labeled dynamic via temporal motion evidence (not class priors alone). Optional for SLAM input.
