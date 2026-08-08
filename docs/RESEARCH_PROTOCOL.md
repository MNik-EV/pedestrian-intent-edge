# Research Protocol

## Contribution statement (to be validated experimentally)

Adaptive multimodal reliability weighting combined with dynamic-object filtering improves robustness of low-cost indoor localization/navigation under controlled sensor degradation relative to fixed fusion and single-sensor baselines.

## Required comparisons

A. LiDAR-only  
B. Camera-only  
C. Fixed fusion  
D. Adaptive fusion  
E. Adaptive fusion + dynamic filtering  

## Scenarios

1. Normal indoor  
2. Low light  
3. Texture-poor  
4. Dynamic pedestrians  
5. Multiple movers  
6. Glass / reflective  
7. Fast motion  
8. Partial degradation  
9. LiDAR degradation  
10. Camera degradation  
11. Combined degradation  

Use `amp_core.degradation` for **labeled synthetic** failure injection only.

## Metrics

ATE, RPE, position/orientation RMSE, detection P/R, tracking quality, dynamic detection accuracy, localization failure rate, navigation success, collision rate, latency, CPU, RAM, FPS.

Until measured: report **NOT YET MEASURED**.

## Reproducibility

Each `experiment_id` stores git commit, config hash, model hash, software/ROS versions, hardware report, timestamps.
