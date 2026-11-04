# Clearance and LiDAR delivery validation — 5 November 2026

The expanded simulation series passed: five box approaches, rejection of an
unsupported low speed, and the existing free/side/front, rearm and data-loss
scenarios. The same-corridor final reference clearance was **0.816–0.852 m**
in the five approaches. This validates the defined flat-world experiment,
not arbitrary obstacles, gaits or a physical Go2.

Environment: Ubuntu 24.04.5 ARM64 in UTM, 4 GiB RAM / four vCPUs, ROS 2 Jazzy,
Gazebo Sim 8.15.0. Project baseline `183eb236f0777ad3f228461caf1a67e0ba09ed96`,
change branch `feat/clearance-validation`. External simulator remains clean at
`55ff151632a0220bf3ae0c0772392615d86da7a8`.

## What the distance means

The configured forward corridor is `x >= 0.45 m`, `abs(y) <= 0.40 m` in a
horizontal robot-heading frame. Clearance is the smallest obstacle X inside
that corridor, minus the configured front extent of 0.45 m. It is a corridor
metric, not the minimum Euclidean distance between every part of the robot
and every part of an obstacle.

The earlier report's `ground_truth_clearance_m` projected the **whole box**
onto the robot's forward axis. As the robot yawed, the nearest projected corner
could lie outside the observed corridor. Comparing that value with the LiDAR's
corridor-only value mixed two different metrics. For example, the final
straight-case detection cloud at simulation time 33.9 s yielded:

| Reference for the same source time | Clearance |
| --- | ---: |
| Production TF and LiDAR processing | 0.797 m |
| Same cloud, actual Gazebo body attitude and position | 0.810 m |
| Analytic box clipped to the same corridor | 0.824 m |
| Legacy projection of the entire box | 0.683 m |

The new observer transforms each actual stamped cloud into `base_link`, then
uses the full ground-truth quaternion/position to remove body roll and pitch.
It compares that result with production `base_footprint` TF at the **same stamp**.
Ground-truth poses must bracket the source time with at most a 60 ms gap;
there is no pose extrapolation. An analytic oriented rectangle is clipped
against the same corridor before measuring its gap.

Gazebo ground truth is used only by the test observer. It never provides data
or commands to the motion veto. Its frame assumptions were checked against the
pinned simulator's odometry configuration and
[Gazebo 8 OdometryPublisher implementation](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/odometry_publisher/OdometryPublisher.cc).
The test does not independently calibrate the simulator's static LiDAR joint
or prove the physical robot's body dimensions.

Version 2 reports store the clipped value in `ground_truth_clearance_m`, label
`clearance_metric_version: 2`, and preserve `legacy_projected_clearance_m`.
Old raw reports are unchanged. Their previously reported 0.61–0.74 m values
must be read as whole-box longitudinal projections. The previous runs cannot
be exactly recalculated without their missing full pose history.

## Ground support correction

The first audit stopped on `unknown_forward_ground` near the box. Both the
production and independent reference processing failed to fit the plane:
the 0.8 m LiDAR minimum range and the box left a visible ground patch shorter
than the fitter's required 0.40 m longitudinal extent.

The fitter now includes nearby side support out to `abs(y) <= 1.0 m`, while
still requiring at least eight observed ground points **inside the forward
corridor in the same fresh cloud**. Side points alone cannot authorize motion.
No past ground plane is held over a missing observation. Tests cover the
short front patch, side-only points and a raised forward surface.

## Predetermined matrix and results

The matrix requires final same-corridor clearance >= 0.70 m; absolute frame
and cloud-to-analytic errors at the 95th percentile <= 0.05 m; forward travel
after observed zero <= 0.03 m; paired reference fraction >= 80%; actual forward
progress >= 0.02 m; and no robot/box contact. At least eight paired clouds and
the exact detection cloud must be available. These limits were defined before
the final run, not fitted to its results.

The 0.5 m deep, 1.0 m high box is recreated for each case. A dropped probe
positively verifies the new box's contact sensor before that approach.
Angles are relative to the robot's heading when placing the box. The robot
continues through the series without teleporting its pose.

| Case | Width | Angle | Final corridor gap | Frame error p95 | Cloud/box error p95 | Post-zero forward travel |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Straight | 1.0 m | 0° | 0.823 m | 10.8 mm | 24.4 mm | 0.3 mm |
| Angled left | 1.0 m | +15° | 0.816 m | 12.2 mm | 23.5 mm | 5.8 mm |
| Angled right | 1.0 m | −15° | 0.840 m | 21.2 mm | 21.0 mm | 0.0 mm |
| Narrow | 0.4 m | 0° | 0.852 m | 10.7 mm | 26.5 mm | 0.0 mm |
| Wide | 1.4 m | 0° | 0.822 m | 18.7 mm | 22.7 mm | 0.3 mm |

All five passed with 100% paired references in their measured approach phases
and no contacts. Observed zero-command delays after obstacle status receipt
were 6.8–10.1 ms. These are observer wall-time measurements, not synchronized
sensor-to-actuator latency or hardware braking figures.

The accepted moving command is 0.05 m/s. A prior 0.03 m/s trial accumulated
about 33° heading excursion and left the corridor; it failed and is retained.
The default profile therefore rejects positive requests below 0.05 m/s,
requires neutral rearming, and caps larger requests at 0.05 m/s. This narrows
the supported experiment rather than claiming the slow gait was fixed.
The sixth matrix case verified that 0.03 m/s produces only zero commands;
absolute forward displacement was 0.2 mm over the observation window.
Actual body speed is not assumed equal to the requested speed.

The separate full regression scenario passed with a 0.826 m final corridor gap,
zero robot/box contacts, and no automatic restart when the obstacle was removed.
Withholding the real LiDAR relay produced final zero output after 1.556 wall
seconds; stopping request publication produced zero after 0.491 seconds.
Maximum forward travel after observed zero across those fault cases was
12.5 mm. The independent watchdog's node-loss behavior is also covered by ROS
integration tests.

## LiDAR delivery

`bridge_profile.py` independently subscribes to native Gazebo headers and to
ROS outputs. It records stamps and receipt times, without payloads. Two
20-second stationary windows compared the **same original bridge publisher**
using simultaneous best-effort and reliable subscriptions. The second window
also added an experimental one-way bridge on a separate ROS topic.

| Window | Native unique frames | Original BEST_EFFORT | Original RELIABLE depth 1 | Added one-way BEST_EFFORT |
| --- | ---: | ---: | ---: | ---: |
| Original bridge | 182 / 9.10 Hz | 131 / 6.55 Hz | 182 / 9.10 Hz | — |
| Parallel bridge experiment | 180 / 9.00 Hz | 100 / 5.00 Hz | 180 / 9.00 Hz | 83 / 4.15 Hz |

Within the shared source-stamp ranges, reliable matched **100%** of native
stamps in both windows; original best effort matched 73.2% and 56.3%.
The percentages use shared stamp ranges, not a ratio of unrelated rate windows.
Native stamps had no duplicates. The maximum reliable receipt gap was 0.144 s;
the one-way best-effort gap reached 1.109 s. Changing bridge direction did
not solve the observed delivery difference.

This isolates a LiDAR delivery problem in this VM's ROS/DDS best-effort path.
UDP receive-buffer error counters also increased during profiling, but no
per-packet trace was captured, so the precise transport-level cause remains
an inference. The
[Fast DDS large-data guidance](https://fast-dds.docs.eprosima.com/en/v2.6.11/fastdds/use_cases/large_data/large_data.html)
describes relevant fragmentation/buffering effects. No system buffer settings,
external bridge source or sensor update rate were changed.

The launch and stop node now default to reliable LiDAR delivery with depth
one; the test relay uses compatible QoS. Source timestamp, wall-clock freshness,
TF and command deadlines remain enforced. A reliable queue does not make old
scans fresh. `lidar_reliability:=best_effort` remains an explicit comparison
option. Both subscription policies have ROS integration coverage with
compatible publishers.

This resolves the measured LiDAR component sufficiently for this experiment.
**RGB PERF-001 remains open**; the stop profile disables RGB. The comparison
does not prove physical Wi-Fi throughput or universal superiority of one QoS.

## Reproduce and inspect evidence

From an unlocked Ubuntu desktop, close previous simulation sessions, build
the package, and run the experiments sequentially:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/guidedog_ws
colcon build --packages-select guidedog_perception --symlink-install
~/guidedog_ws/src/GuideDog-Go2/scripts/test_obstacle_demo.sh --clearance-matrix
~/guidedog_ws/src/GuideDog-Go2/scripts/test_obstacle_demo.sh --audit-clearance
~/guidedog_ws/src/GuideDog-Go2/scripts/test_obstacle_demo.sh --bridge-profile
colcon test --packages-select guidedog_perception --event-handlers console_direct+
colcon test-result --verbose
```

Reports are printed under `~/.ros/guidedog/reports/`. Relevant saved runs:

- `clearance-matrix4-20261105`: final passing matrix; `benchmark.json`,
  `clearance-samples.jsonl`, `launch.log`, `session/` sensor and stop reports.
- `stop-audit2-20261105`: full passing fault/rearm regression and matched scan audit.
- `bridge-profile2-20261105`: reliable/best-effort comparison; `bridge-stamps.json`
  contains the actual receipt-time/source-stamp pairs.
- `clearance-audit1-20261105`: retained initial ground-fit failure.
- `clearance-matrix3-20261105`: retained four passing approaches, wide-case
  stale-scan stop, and failed 0.03 m/s drift trial before the final profile changes.
- `bridge-profile1-20261105`: retained original/one-way comparison before adding
  the simultaneous reliable observer.
- `clearance-matrix1-20261105`: failed startup while the desktop was locked.
  Matrix attempt 2 did not start because of an inhibitor invocation error.
  An initial attempt 4 stopped at lint before creating a scene; import order
  was corrected before the final validation. These are not successful runs.

The final package suite reports **44 passed, 1 skipped, 0 failures/errors**.
The skipped copyright check was already disabled in the baseline. Four warnings
concern the lint tool's fork in a multithreaded process.

The series is a bounded repeatability check, not a statistical reliability
estimate. RGB timing, more starting poses, gait stabilization, sloped/uneven
ground, low/overhead/negative obstacles and colleague reproduction remain
separate work. The next geometry feature is a local map that preserves
occupied, free and unknown space with explicit age and uncertainty.
