# GuideDog-Go2 Roadmap v1.1 — Air baseline, local world model, stairs in MVP

**Updated:** 2026-10-19  
**Status:** proposed implementation plan; all capabilities require acceptance tests, not just a demo.

## Product and deployment decisions

- **Target hardware baseline:** stock Unitree **Go2 Air** sensors and locomotion. Do not assume EDU-only compute, interfaces, locomotion modes, or developer access.
- **Temporary physical prototype:** **Go2 Air + nearby laptop**, connected over a **local network with no internet requirement**. The laptop runs GuideDog Intelligence; Go2 supplies sensor streams and performs low-level locomotion. Local network availability, command-path access, firmware compatibility, and the robot-side loss-of-link stop must be proven before physical autonomous tests.
- **Development:** Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic in UTM on Mac; real-hardware connection is a separate adapter. Simulator is not a validated hardware digital twin.
- **Spatial intelligence:** a rolling, robot-centered **local world model**; no global SLAM map is required for the first MVP. Start by evaluating a ~10 × 10 m region with a 2.5D elevation/traversability representation plus 3D obstacles, head-clearance checks, confidence, and data age. These dimensions are tunable hypotheses, not safety specifications.
- **Compute strategy:** measure CPU, RAM, latency, and actual sensor throughput before choosing optional onboard hardware. No mandatory Jetson, cloud inference, RL, VLM, or heavy segmentation network in the initial baseline.
- **Human-aware design:** validate robot and handler clearance separately. A virtual handler is initially used in simulation; do not claim real-person safety from it.
- **Stairs and curbs are part of the REQUIRED functional MVP**, not a deferred research extra: detect both ascending and descending edges, stop/communicate before entry, and demonstrate controlled stair ascent/descent with handler-aware behavior. The platform's physical capability remains a **hard feasibility gate**; a blocked gate is a reported limitation, never a reason to waive safety criteria.

## Distinct deliverables and no-go rules

1. **Simulation functional MVP (M0–M4):** tested perception, local terrain/obstacle model, planning, independent safety, and controlled stair ascent **and** descent with a simulated handler in defined test scenarios.
2. **Physical feasibility prototype (M5):** validate the Air's accessible sensors, command interfaces, connectivity, safe command timeout, and locomotion on representative stairs **without a person** first. Physical stairs remain a no-go until the robot can recover or stop safely in each tested stair state.
3. **Human-use validation (later, separate approval):** a stair-capable robot alone is **not** a validated guide dog. Physical handler interaction, fall prevention, emergency procedures, and supervised studies with relevant accessibility expertise are separate prerequisites. No unsupervised guiding of a blind or low-vision person is authorized by M5.

If stock Go2 Air cannot meet validated step-height, stair-descent, stability, and fail-safe requirements, **do not mark stair-capable physical MVP as complete**. Document the failure and revisit the platform / locomotion assumptions without silently relaxing the product requirement.

## M0 — Reproducible simulation and timing (current)

- [x] Simulated Go2 launches and responds to keyboard teleoperation.
- [x] Gazebo and ROS 2 camera and LiDAR topics discovered.
- [x] Local ROS 2 subscriber receives `640×480` `rgb8` frames.
- [x] Local OpenCV screenshot captured.
- [x] Public repository created; RGB package committed.
- [x] External simulator pinned to `55ff151632a0220bf3ae0c0772392615d86da7a8`.
- [x] Apache-2.0 selected for original project code.
- [ ] Agree two-developer ownership, review and message contract process.
- [ ] Reproduce launch, teleoperation, RGB, and LiDAR independently on second Mac/UTM setup.
- [ ] Diagnose and benchmark **PERF-001** (Gazebo ~9 Hz versus ROS 2 subscribers ~2–4 Hz): source-to-receiver rates, QoS, bridge, timestamps, drops, CPU, and jitter.
- [ ] Verify TF and timestamp integrity for camera, LiDAR, IMU/odometry and robot base; record what is simulated rather than assumed.

**Exit gate:** both developers can reproduce the baseline and have a recorded performance profile; any remaining sensor bottleneck is understood and documented before heavier inference is added.

## M1 — Visual perception (planned)

- [ ] Agree minimal, simulator-independent detection messages with class, score, frame ID, timestamp, latency, and validity.
- [ ] Run one lightweight object detector on RGB frames; initially choose the smallest defensible runtime/model.
- [ ] Publish ROS 2 detections; instrument inference latency and throughput.
- [ ] Test pedestrians, poles and other relevant obstacles; distinguish unrecognized objects from geometrically free space.
- [ ] Add stale-data semantics and simple detector regression scenes.

**Exit gate:** measurable detector quality and timings, repeatable ROS 2 output, no claim that semantic detections alone prove traversability.

## M2 — Geometry, terrain and rolling local world model (planned)

- [ ] Parse LiDAR `PointCloud2` and validate coordinate transforms, ground alignment, time alignment, and odometry updates.
- [ ] Implement a rolling local representation: occupied/free/**unknown**, elevation, surface slope/roughness, vertical clearance, confidence and last observation time. Benchmark map memory and incremental update cost.
- [ ] Detect upward/downward curbs, positive and negative elevation transitions, holes, drop-offs, and ramps.
- [ ] **Stair geometry:** detect ascending and descending stairs, estimate first/last step, step height/depth, orientation, usable width, and confidence wherever sensor coverage supports it.
- [ ] Preserve uncertainty at descending edges: missing LiDAR returns must **not** be interpreted as safe ground.
- [ ] Integrate vision with geometry only where alignment/calibration allows defensible association.
- [ ] Test partial occlusions, reflective/transparent surfaces, coarse resolution, false flats, and motion updates; include raw point-cloud checks where grid resolution would erase small edges.

**Exit gate:** reproducible terrain/stair detection benchmarks, reported false negatives and boundary uncertainty; map follows the robot without requiring a global map.

## M3 — Local navigation and independent safety (simulation only, planned)

- [ ] Build a **classical local planner** as baseline before any optional RL/MPC study.
- [ ] Distinguish robot-body, human-body, and shared-path clearance. Represent traversability restrictions conservatively.
- [ ] Provide stair approach and **pre-entry STOP** logic; never enter unverified stairs or a drop-off.
- [ ] Implement stair-specific decision modes: normal ground, approach, evaluate/confirm, ascend, descend, exit, abort/recover. Initially lock ascent/descent modes behind simulator and safety checks.
- [ ] Implement an independent safety supervisor capable of command veto, stale-sensor stop, command age limits, and fault injection. Safety should also have direct access to critical raw/derived geometry.
- [ ] On stairs, define a **stable stop/recovery**, not merely “send zero velocity”; validate no destabilizing freeze on a step.
- [ ] Quantify obstacles, missed detections, boundary errors, stop distance, watchdog behavior, and command latency.

**Exit gate:** the robot passes predefined collision, drop-off, command-timeout, and stair-entry safety tests in simulation; unverified stair states remain blocked.

## M4 — Human-aware simulation and stairs (planned)

- [ ] Model a virtual handler with relative position, body/shoulder/head clearance, connection/handle assumptions, gait lag, and stopping response.
- [ ] Test moving pedestrians, occlusions, narrow routes and shared safe corridor.
- [ ] **Ascending:** controlled straight-stair traversal in simulation with virtual handler; account for the handler remaining on lower steps during robot ascent.
- [ ] **Descending:** separately validate step edges, foot/robot stability, relative pace and handler clearance; do not infer descending performance from ascending.
- [ ] Detect entrance, landing, and exit transitions; communicate stair direction and wait for simulated user acknowledgement before proceeding.
- [ ] Inject stair faults: missed edge, partial point cloud, unexpectedly halted handler, unsafe pose, network/command delay, slipping/unavailable locomotion.
- [ ] Record task success, robot collisions, **handler** collisions, near misses, emergency interventions, per-step geometry errors, CPU/RAM and end-to-end latency.
- [ ] Test both stair directions across multiple geometries/seeds; define pass thresholds based on measured stopping/recovery capability rather than inventing a universal step height.

**Simulation functional MVP exit gate:** controlled ascent and descent with a virtual handler succeed in the defined benchmark; no unresolved critical safety failure. This **does not** demonstrate safe real-world human guidance.

## M5 — Go2 Air + laptop integration and hardware feasibility (future; not authorized by simulation success alone)

- [ ] Obtain and identify the specific Air hardware and firmware; verify RGB, LiDAR, IMU/odometry accessibility and command semantics using a documented local-only interface.
- [ ] Benchmark Wi-Fi connection loss, latency/jitter, data freshness, CPU/RAM and the complete laptop → robot motion-command path.
- [ ] **Verify on-robot loss-of-command stop / timeout / emergency behavior independently of laptop software.** If unavailable or unreliable, do not perform autonomous physical experiments.
- [ ] Validate physical sensor calibration, edge visibility, robot footprint, permitted gaits, mechanical stability, and actual stair ascent/descent limits for the exact Air revision.
- [ ] Progress in gates: sensor-only tests → non-autonomous flat-ground control → obstacle/STOP tests → stair trials **with robot only**, containment/spotter and emergency provisions → integration of handler mechanics only after independent review.
- [ ] Measure real step traversal in **both** directions including controlled interruption and recovery. Failure to meet stair requirements blocks the physical functional MVP.

**No-go:** no unsupervised operation as a mobility aid; no blind/low-vision user on real stairs under this milestone.

## Test and measurement policy

Maintain a scenario matrix, replayable logs, tested Git SHAs, environment/firmware IDs and acceptance thresholds for:

- positive vs negative obstacles, curbs, stairs up/down, landings, ramps, pits and overhead obstacles;
- robot collisions **and handler** collisions (separately), near misses, false safe classifications and false stops;
- stair-edge and step-geometry estimation error, missed step detection, safe response to unknown regions;
- measured network/sensor-to-command latency, jitter, frame drops, compute/memory budget;
- loss of sensor, planner hang, lost connection, timeout, emergency stop, and mid-stair stability/recovery.

Do not extrapolate safety claims from a few successful runs. A safety gate must be demonstrated, with a documented failure policy, before proceeding.

## Work split (initial proposal; confirm together)

- **Developer A:** LiDAR/TF, terrain/elevation and local world model, fusion, safety supervisor and acceptance criteria.
- **Developer B:** visual detector/tracking, classical local planner and stair guidance state-machine integration.
- **Both:** handler assumptions, simulator stair scenarios, performance profiling, integration, hardware gates, design reviews, documentation and PR review.

## Schedule (indicative, not a shipping commitment)

| Milestone | Indicative window |
| --- | --- |
| M0 | Oct 19–25, 2026 |
| M1 | Oct 26–Nov 8, 2026 |
| M2 | Nov 9–29, 2026 |
| M3 | Nov 30–Dec 20, 2026 |
| M4 | Dec 21, 2026–Jan 3, 2027 |
| M5 | Only after M4 pass + physical device access and safety readiness |

Acceptance gates take precedence over dates.

## Existing timing blocker

**PERF-001:** camera and LiDAR were generated around 9 Hz in Gazebo, but ROS 2 Python subscribers on Ubuntu ARM64/UTM observed only around 2–4 Hz. Cause not yet established; do not treat the slower rate as intrinsic hardware performance or proceed as though end-to-end timing has been validated.

## Boundaries

- No proprietary employer/client code, models, data, or other internal resources may be transferred into this independent project.
- The third-party simulator is an external pinned dependency; respect its license and do not silently upgrade.
- No real-world safety or accessibility capability is established by the milestones already completed.
