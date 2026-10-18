# GuideDog-Go2 architecture v1.1 (proposed)

**Status:** Early design. Only simulated Go2 teleoperation, RGB/LiDAR topic discovery and project-owned RGB ingestion have been demonstrated. No autonomous navigation, safe stair handling, physical deployment or user guidance validated.

## Selected platform and compute boundary

- **Minimum hardware target:** Unitree Go2 **Air**, with stock RGB camera and L1-style LiDAR. IMU/odometry availability and usable interfaces must be confirmed on purchased hardware and firmware.
- **Development:** Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic under UTM on Mac. External simulator pinned separately, not vendored.
- **Temporary physical prototype:** Go2 Air + **laptop running the complete GuideDog intelligence stack**, communicating over a **local Wi-Fi network**. **No internet or cloud inference** required.
- Never assume unrestricted access to Air internal compute, EDU-only features, a Jetson, added depth camera, or a particular stair gait. Prove compatible local ROS 2 adapter and motion command/stop behavior on actual Air before autonomous physical trials.
- Long-term compute placement is undecided; keep perception, geometry, planner and safety code hardware-neutral wherever practicable.

## Conceptual data flow

```text
Gazebo (now) OR Go2 Air local hardware adapter (later)
  RGB ---------> Visual perception / tracking ------+
  LiDAR -------> 3D geometry + terrain analysis ----+--> Rolling Local World Model
  IMU/Odom ----> State estimation, TF, timing -------+      occupancy/free/unknown
                                                          elevation/slope/edges
  Handler state (simulated first; physical sensor TBD) --> handler-relative pose/uncertainty
                                                          3D head/shoulder clearance
                                                          confidence/freshness
                                                               |
                                              Human-aware classical local planner
                                                               |
                                        Stair guidance state machine / local goal
                                              approach, evaluate, confirm,
                                              ascend, descend, exit, abort
                                                               |
                                            Independent safety supervisor / veto
                                                     direct critical sensing
                                                     watchdog / command freshness
                                                     stair stability handling
                                                               |
                                               Bounded Go2 command adapter
                                                               |
                                             Stock low-level Go2 locomotion
```

Boxes above are **planned**, not implemented. Safety does not depend solely on the planner's output. Local communication and robot-side loss-of-command stop must be validated; a laptop-only watchdog cannot stop a robot after the radio link fails.

## Spatial representation

Use a **rolling robot-centered local map**, initially benchmark ~10 × 10 m; map size and resolution are parameters, not guaranteed safety values. Retain short-lived obstacles after temporary occlusion, decay stale observations, preserve unknown regions as unknown, and move the map incrementally with odometry. Use a 2.5D elevation/traversability layer for efficient planning plus 3D obstacle/overhead clearance where projection would lose relevant geometry.

Terrain-specific classification includes upward/downward curbs, drop-offs, holes, ramps, **ascending and descending stairs**. Estimate stair boundaries, step geometry, traversable width, landings and confidence. Do not infer a safe drop-off from lack of LiDAR points or call semantic segmentation equivalent to traversability.

Planning must separately evaluate robot and handler envelopes. The handler is simulated initially. Physical handle/relative positioning measurement and human interaction are unsolved and cannot be inferred from following a virtual point.

## Stairs as a functional MVP requirement

Controlled ascending **and** descending straight stairs with a virtual handler in specified simulation scenarios are required for the **simulation functional MVP**. This includes approach, pre-entry stop, user confirmation, movement synchronized to handler pace, landing/exit, interruption and stability/recovery checks.

Physical stair ability of **Go2 Air** is an explicit feasibility gate, not an assumed capability. Hardware trials require independent robot-only ascent and descent testing, actual command interface/access, demonstrated safe stopping/recovery in stair poses and verified link-loss behavior. A successful robot-only demo is not proof of safe human accompaniment. An unavailable/unsafe Air gait blocks the physical stair-capable MVP rather than authorizing risky operation.

## ROS 2 baseline (simulation only)

| Topic | Type | Status |
| --- | --- | --- |
| `/rgb_image` | `sensor_msgs/msg/Image` | Verified 640×480 `rgb8`; frame `camera_link` |
| `/unitree_lidar/points` | `sensor_msgs/msg/PointCloud2` | Verified publishing; frame `lidar_l1_link` |
| `/imu/data` | `sensor_msgs/msg/Imu` | Discovered; end-to-end GuideDog integration not tested |

Define new ROS 2 output messages only after agreeing units, frames, timestamps, freshness/validity, confidence, QoS and expected rates. Simulated topic names and rates do **not** establish compatible hardware interfaces.

## Engineering principles

1. Independently test perception, geometry, mapping, planning, stair guidance, safety and robot-specific adapters.
2. Keep critical safety logic independent of semantics/ML; reject stale, contradictory, unknown or low-confidence geometry.
3. Use measured end-to-end latency and robot dynamics to set motion limits and stop margins. Mid-stair safe stop means a stable posture/recovery, not an unconditional freeze.
4. Prefer a reproducible classical planner baseline; do not introduce RL/VLM or global SLAM without a demonstrated need.
5. Log raw sensors, commands, TF, latency, failures, CPU/RAM, and separate robot/handler contact results.
6. No user-facing guide-dog safety claim from simulation or unsupervised tests.

## Open decisions and blockers

- `PERF-001`: ROS 2 subscriber RGB ~2–3 Hz and LiDAR ~3–4.5 Hz against Gazebo ~9 Hz on UTM. Diagnose before piling on inference.
- Actual Air firmware/interface, LiDAR-camera extrinsics, timestamp synchronization, and robot-side command timeout.
- Human/handle measurement, interaction/confirmation channel, and required specialist review.
- Tested stair height/depth and stair-descent capability; benchmark-based acceptance thresholds, fall/recovery safety and network failure.
- Real-world deployment beyond carefully supervised, robot-only controlled testing is **not authorized** by this design.

See [roadmap](roadmap.md) and [limitations](limitations.md).
