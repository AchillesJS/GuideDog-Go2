# Limitations, assumptions, and safety

**Prototype status:** GuideDog-Go2 has only demonstrated simulation setup and RGB frame ingestion. It is **not** an assistive mobility product.

## Current limitations

- No object detector, fused world model, independent safety supervisor, autonomous route planning, or human-guidance interface has been implemented.
- No verification on a physical Unitree Go2.
- Simulation's L1-like 4D LiDAR may differ from physical Go2 hardware/sensor revisions. Stock camera and LiDAR availability, calibration, coverage, and timing must be checked on the target robot.
- The initial environment was Ubuntu ARM64 in UTM. ROS 2 RGB reception was roughly 2–3 Hz and LiDAR reception roughly 3–4.5 Hz while source Gazebo topics were near 9 Hz. Delivery timing, jitter, frame drops, and bridge performance remain under investigation.
- ROS 2 topic existence and single-frame image capture do not demonstrate correct localization, depth estimation, obstacle avoidance, or accessibility.

## Safety principles for future development

- Stop/hold safely on missing, stale, contradictory, or low-confidence perception and localization.
- Treat curbs, stairs, roads, drop-offs, reflective/transparent obstacles, and moving people as separate hazards requiring specific validation.
- Evaluate end-to-end latency, detection false negatives, command timeout, contact forces, and user interaction before physical experiments.
- Use supervised and controlled testing only; never present unvalidated behavior as safe navigation for visually impaired users.
- Obtain appropriate specialist and end-user input before defining human-assistance requirements.

## Data and intellectual property

No employer/customer confidential code, models, or datasets may be included. Avoid committing recorded images containing personal information without authorization.
