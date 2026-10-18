# Architecture (proposed)

**Status:** early design; only RGB ingestion has been demonstrated in custom GuideDog code.

## Data flow

```text
Gazebo Go2 / later physical Go2
  ├── RGB camera ──> /rgb_image ──> guidedog_perception ──> [detections: planned]
  ├── 4D LiDAR ────> /unitree_lidar/points ──> [geometry: planned]
  └── IMU / odometry ─────────────────────────> [state estimation: planned]
                                              │
                           [world model / sensor fusion: planned]
                                              │
                          [human-aware local navigation: planned]
                                              │
                           [independent safety gate: planned]
                                              │
                                  [Go2 command adapter: planned]
```

Square brackets denote **unimplemented** components; topic names for their outputs are not frozen.

## Existing software

- `guidedog_perception`: ROS 2 `ament_python` package, local `rgb_node.py` logging RGB frame metadata, pending upload.
- Sensor diagnostic helpers `~/sensor_probe.py` and `~/capture_rgb.py` were tested locally; they are not yet tracked in this repository.
- Simulator from `khaledgabr77/unitree_go2_ros2`: external third-party ROS 2/Gazebo dependency, not part of this codebase.

## ROS 2 interface baseline (simulation)

| Topic | Type | Status |
| --- | --- | --- |
| `/rgb_image` | `sensor_msgs/msg/Image` | Verified, 640×480 `rgb8`, frame `camera_link` |
| `/unitree_lidar/points` | `sensor_msgs/msg/PointCloud2` | Verified publishing, frame `lidar_l1_link` |
| `/imu/data` | IMU messages | Discovered; end-to-end GuideDog integration not tested |

Simulator bridge publishers used `RELIABLE` QoS during local tests. Consumers must negotiate compatible QoS settings.

## Design rules

1. Keep perception, geometry, world model, planning, safety, and robot-specific adapters independently testable.
2. Propagate timestamps, coordinate frames, confidence, and freshness; never silently treat stale data as current.
3. A safety supervisor must be capable of vetoing movement, independent of the planner.
4. Do not equate visual object detection with traversability or safe guidance.
5. Prefer simulator-independent ROS 2 message contracts; agree on messages before parallel implementation.

## Open design decisions

- Detection and fused-obstacle message schemas.
- Camera/LiDAR calibration, time synchronization, and frame transforms.
- Path planning representation and human-relative speed limits.
- How human intent and physical contact/handle input will be measured.
- Fault injection, emergency STOP acceptance criteria, and hardware deployment gates.
