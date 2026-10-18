# Limitations, assumptions, and safety

**Prototype status:** GuideDog-Go2 has demonstrated only simulation setup, teleoperation, topic discovery and project-owned RGB frame ingestion. No safe autonomous navigation, stair assistance or physical guide-dog functionality has been validated.

## Hardware and operating boundaries

- **Target:** stock Go2 Air sensors; actual firmware, interfaces and available motion capabilities remain unknown until the physical robot is obtained.
- **Temporary deployment:** Go2 Air + laptop over a local Wi-Fi link; internet is not required. This is not onboard autonomy; link failure is a safety-critical hazard.
- **Critical physical gate:** prove robot-side timeout or equivalent independent stop/recovery on command loss before autonomous tests. A stop command from the disconnected laptop cannot guarantee a stop.
- Air must not be assumed to support developer access, EDU expansion compute, stair gait modes or all stair geometries.
- Simulated L1-like sensor is not a validated twin of a physical Air sensor revision.
- On Ubuntu ARM64/UTM, ROS 2 RGB was ~2–3 Hz and LiDAR ~3–4.5 Hz versus ~9 Hz Gazebo. `PERF-001` remains open; rates, jitter and full command latency must be measured.

## Stair safety (required functional MVP, not yet implemented)

- Detect ascending **and** descending stairs, boundaries, positive/negative curbs, holes and drop-offs, with explicit uncertainty and stale data.
- In simulation, demonstrate controlled approach, stop, confirmation, ascent/descent, landing, exit, handler synchronization and fault recovery; report both successful and failed trials.
- Robot-only real stair ascent/descent must be demonstrated under controlled, supervised conditions, including stable mid-stair stop/recovery and emergency procedures, **before any physical handler study**.
- Physical step traversability and descending stability may exceed what stock Go2 Air can do. If infeasible, do not call the hardware MVP complete or bypass limits.
- Unknown ground is **not** free ground; blind spots or insufficient downward LiDAR returns require conservative handling.
- Robot stability is not equal to human safety; account separately for head clearance, human path, pace, fall hazards, handles and contact forces.

## Safety principles

- An independent supervisor must veto unsafe commands based on critical geometry, timestamps, confidence and system state, with tested watchdogs and physical loss-of-link behavior.
- Stopping on stairs requires a validated stable posture or recovery. Abrupt command zeroing alone is not a validated safe response.
- Validate curbs, stairs, roads, drop-offs, transparent/reflective hazards, moving people and sensor failures separately.
- Measure false negatives, false stops, end-to-end latency, stop distance, command timeout, falls, collisions, contact forces and user interface before physical human experiments.
- No blind/low-vision person may rely on the device as a mobility aid under the current milestones. Obtain domain-expert and end-user input, appropriate oversight and separate human-use validation.

## Data, licensing and privacy

No confidential employer/customer code, models, datasets, or other internal resources may be copied into this independent project. Check third-party licenses and protect recorded images or other personal data.
