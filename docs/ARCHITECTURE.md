# Architecture

## Title

Adaptive Multimodal Perception and Sensor Fusion for Robust Low-Cost Autonomous Indoor Robots

## Split compute model

```
                 RASPBERRY PI 5                         PC (Research)
                       │                                      │
        ┌──────────────┼──────────────┐                       │
        │              │              │                       │
      LD19          PS3 Eye          IMU                      │
        │              │              │                       │
        └──────────────┼──────────────┘                       │
                       ↓                                      │
              MULTIMODAL PERCEPTION                           │
                       ↓                                      │
        ┌──────────────┼──────────────┐                       │
        ↓              ↓              ↓                       │
     LiDAR          Vision        Odometry                    │
        │              │              │                       │
        └──────────────┼──────────────┘                       │
                       ↓                                      │
             ADAPTIVE SENSOR FUSION                           │
                       ↓                                      │
               DYNAMIC FILTERING                              │
                       ↓                                      │
                   SLAM / MAP                                 │
                       ↓                                      │
              NAVIGATION / SAFETY                             │
                       ↓                                      │
                    MOTORS                                    │
                       │                                      │
                     Wi-Fi  ◄──────── dashboard / DDS / REST ─┘
                       ▼
              WEB DASHBOARD (on Pi)
                  ↙           ↘
               PHONE          LAPTOP
```

## Software layers

1. **`amp_core`** — ROS-agnostic algorithms (testable on Windows PC).
2. **`ros2_ws`** — thin ROS2 Jazzy nodes / launch files for Pi & Ubuntu.
3. **`web_dashboard`** — FastAPI + WebSocket + control-center UI.
4. **`evaluation` / `experiments`** — reproducible research loop.

## Fusion modes

| Mode | Description |
|------|-------------|
| `lidar_only` | Baseline |
| `camera_only` | Baseline |
| `fixed_fusion` | Constant sensor weights |
| `adaptive_fusion` | Confidence → bounded measurement covariance |
| `adaptive_fusion_dynfilter` | Adaptive + dynamic obstacle removal for SLAM |

## Safety

`SafetySupervisor` runs locally on the Pi. It does **not** depend on Wi-Fi, dashboard, or detector models. Web teleop commands are always filtered.

## Data flow (topics / logical buses)

- LiDAR → filter → sectors / obstacles / confidence features  
- Camera → preprocess → features / detections → tracks → distances  
- Reliability estimator → SensorConfidence  
- Adaptive EKF → pose / covariance / weights  
- Dynamic filter → static scan → SLAM  
- Navigator → SafetySupervisor → motors  
- Telemetry aggregator → WebSocket + experiment logger  
