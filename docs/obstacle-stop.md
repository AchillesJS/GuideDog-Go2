# Flat-ground obstacle-stop demo

This is a simulation-only, forward-only experiment, built on the diagnostics
baseline. It adds geometry filtering, a command veto and a separate output
watchdog. It does not implement route planning, turning, reverse, stairs,
drop-off detection, human guidance or a physical Go2 adapter.

See the [initial Gazebo validation](obstacle-stop-validation.md) and the
[clearance and delivery investigation](clearance-validation.md). The latter
corrects the interpretation of the earlier clearance metric and records the
expanded test series.

## Build and run

```bash
source /opt/ros/jazzy/setup.bash
cd ~/guidedog_ws
colcon build --packages-select guidedog_perception --symlink-install
~/guidedog_ws/src/GuideDog-Go2/scripts/run_obstacle_demo.sh
```

Wait for the robot to stand and both controllers to activate (typically at
least 30 seconds). In a second Ubuntu terminal:

```bash
~/guidedog_ws/src/GuideDog-Go2/scripts/drive_guarded.sh
```

- Hold `i` to request forward motion at 0.05 m/s. This is the only moving speed
  validated by the current profile; larger requests are capped and positive
  requests below 0.05 m/s are rejected. A 0.03 m/s experiment drifted sideways.
- `k` or space sends a neutral request. Release `i` to let the keyboard dead-man
  timeout stop the request after 0.6 wall seconds.
- After an obstacle or fault, clear the condition, send neutral, then press `i`
  again. A held forward request cannot automatically resume after a stop.
- Ctrl+C closes the keyboard node; the downstream request timeout also stops motion.
- Other keyboard motions are deliberately unavailable in this first demo.

Use these scripts together: they select ROS domain **42** and Gazebo partition
`guidedog_stop` by default. The earlier `run_sim.sh` remains the original
unprotected teleoperation/diagnostics launch. Ordinary teleop in domain 0 does
not control this demo. On the 4 GiB VM, close other Gazebo sessions before testing.
`GUIDEDOG_ROS_DOMAIN_ID` and `GUIDEDOG_GZ_PARTITION` can override the demo isolation.

The wrapper owns its ROS/Gazebo process tree and stops detached server/GUI
children when the launch exits. This prevents leftover worlds from mixing
sensor timestamps or consuming the VM's CPU after a test. Cleanup identifies
owned descendants by PID and process start time; unrelated processes are excluded.

The stop-demo profile disables RGB rendering by default to leave the 4 GiB VM's
CPU budget for LiDAR and motion; its sensor report labels RGB `DISABLED`.
`enable_rgb:=true` enables it for profiling. The original diagnostic launch
still enables RGB by default. Motion test durations use simulation time while
all watchdog deadlines continue to use monotonic wall time.

## Command path

```text
guarded keyboard / scenario requests
  /guidedog/cmd_vel_request          Twist
       -> LiDAR veto
  /guidedog/cmd_vel_candidate        Twist
       -> separate wall-clock watchdog
  /guidedog/cmd_vel_checked          Twist
       -> pinned CHAMP controller
```

A scoped ROS launch remap overrides the controller's normal `cmd_vel/smooth`
subscription. The external simulator's source is unchanged. The automated
scenarios verify that the controller subscribes to the checked topic and has
no `/cmd_vel` bypass before sending motion requests.

The veto publishes zero at 20 Hz when scans, exact-time TF, clock progress,
forward ground support or commands are unavailable. The separate process
continues publishing zero if the veto stops sending its heartbeat for 0.25
wall seconds. CHAMP itself has no command-age timeout in the pinned revision;
simultaneous loss of both observer/watchdog processes is not solved by this demo.

## Geometry and test settings

- XYZ decoding validates point count, field types/offsets, row padding, endian
  order and buffer length. Nonfinite points are discarded; insufficient data stops.
- Points are transformed into `base_footprint` at the source message timestamp.
  A bounded queue of at most eight clouds waits up to 0.15 wall seconds for
  exact-time TF without blocking callbacks. The newest transformable cloud is
  processed; newer pending clouds cannot evict all delayed scans. The previous
  accepted observation is never refreshed by that wait.
- The tested forward corridor has front extent 0.45 m and half width 0.40 m.
  Points between 0.08 and 0.85 m above observed support are candidate obstacles. Body/rear
  returns and off-corridor returns are excluded.
- Clear reports require a support plane fitted independently in each fresh scan,
  with at least eight inliers within 0.025 m and spatial extents of at least
  0.40 x 0.20 m in the forward three-metre region, using nearby support up to
  one metre either side of the centreline. At least eight ground points must
  still be observed inside the narrower forward corridor in that same scan;
  side support alone cannot authorize motion. Deterministic bounded RANSAC
  handles the observed footprint height error while keeping tilt below 10 degrees
  and intercept between -0.15 and +0.055 m. Larger errors and uniformly elevated
  surfaces do not count as ground. Reports include the fitted plane and raw
  forward-height percentiles. This is not continuous ground coverage or hole detection.
- Default stop clearance is 0.80 m from the configured front extent. Maximum
  requested speed is 0.05 m/s, scan timeout 1.50 wall seconds and request timeout
  0.40 wall seconds. Replayed timestamps cannot renew scan freshness.
- The initial VM measurement included a 1.06-second LiDAR receipt gap. The slower
  profile accommodates it while reducing commanded blind travel from the
  initial 0.10 x 0.80 = 0.08 m to 0.05 x 1.50 = 0.075 m. Configuration rejects
  combinations exceeding the 0.08 m commanded-travel budget. This calculation
  does not replace measurement of actual body motion, slip or braking.
- LiDAR subscriptions now default to `RELIABLE`, `KEEP_LAST(1)` after the
  [shared-stamp delivery experiment](clearance-validation.md#lidar-delivery).
  `lidar_reliability:=best_effort` is available for comparison or a publisher
  that only offers best effort. QoS must match the actual publisher. This does
  not extend the scan, TF, command or clock deadlines; old data still stops motion.
- These are bounded simulation experiment settings. Stopping distance and
  observer overhead must be measured; they are not physical safety limits.

The pinned simulated LiDAR has a **0.8 m minimum sensor range**. Its blind zone,
height cutoff, partial coverage, calibration and the body envelope restrict
which objects this demonstration can detect. Do not infer low-curb, overhead,
stair or negative-obstacle performance from the box scenario.

## Reports and automatic scenarios

Manual runs save the existing sensor reports plus:

- `stop_report.json`: current reason, output request, scan age, clearance and parameters.
- `stop_events.jsonl`: state transitions, including obstacle and timeout stops.
- `/guidedog/stop_status`: the same state as a ROS JSON string for the scenario driver.

Run the repeatable Gazebo scenarios from an Ubuntu terminal:

```bash
~/guidedog_ws/src/GuideDog-Go2/scripts/test_obstacle_demo.sh
```

This starts an owned scene, observes the real LiDAR and ground-truth odometry,
and checks: free path, side obstacle, front obstacle, explicit rearm, scan loss
and request loss. Scan loss is injected by withholding a real-stream relay;
the test does not replace geometry with invented points. A box contact sensor
provides a separate contact check. A probe dropped onto the distant box verifies
the contact sensor and bridge before motion, then is removed. Only that named
probe is excluded from collision failures. Gazebo publishes contact messages
only on contact, so silence alone cannot validate the sensor
([Gazebo Contact implementation](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/contact/Contact.cc)).
The driver aborts before the configured
body envelope reaches the box if the stop does not trigger.

The test uses ROS domain **43** and a unique partition with prefix
`guidedog_stop_test` by default,
with `GUIDEDOG_TEST_DOMAIN_ID` / `GUIDEDOG_TEST_PARTITION` overrides. It stops
its own simulation on completion. Each test directory contains `benchmark.json`,
`launch.log`, and a `session/` subdirectory with the sensor/stop reports.

Reported timing is wall time measured at observer receipt. Post-zero travel
is the maximum forward ground-truth displacement during the following two
wall seconds; it is not a hardware braking certification. The final pose
spread is checked separately. Native ROS and Gazebo performance still needs
the remaining RGB investigation documented in the diagnostics guide.

Additional bounded experiments (each creates its own scene and report directory):

```bash
~/guidedog_ws/src/GuideDog-Go2/scripts/test_obstacle_demo.sh --audit-clearance
~/guidedog_ws/src/GuideDog-Go2/scripts/test_obstacle_demo.sh --clearance-matrix
~/guidedog_ws/src/GuideDog-Go2/scripts/test_obstacle_demo.sh --bridge-profile
~/guidedog_ws/src/GuideDog-Go2/scripts/test_obstacle_demo.sh --map-benchmark
~/guidedog_ws/src/GuideDog-Go2/scripts/test_obstacle_demo.sh --with-local-map
```

The audit compares the same stamped cloud with an independent Gazebo pose
reference. The matrix tests five box variants and rejection of an unsupported
low speed. The stationary bridge profile compares native and ROS header stamps
in shared windows and temporarily adds a separate one-way bridge. It requires
the Gazebo Harmonic Python bindings `gz.transport13` and `gz.msgs10`, available
in the tested sourced Jazzy vendor environment. See the
[measurement definitions and acceptance limits](clearance-validation.md).
The [local-map guide](local-map.md) describes the optional observation-only grid
and its separate rolling/blackout scenarios. `--with-local-map` runs the original
stop scenarios with that extra observer enabled.

## Automated checks

```bash
source /opt/ros/jazzy/setup.bash
source ~/guidedog_ws/install/setup.bash
cd ~/guidedog_ws
colcon test --packages-select guidedog_perception --event-handlers console_direct+
colcon test-result --verbose
```

Geometry/policy tests include padded/big-endian clouds, invalid values, frame
transforms, ground and side filtering, unknown space, speed limits, unsupported
motion, replayed stamps and rearming. A ROS integration test in isolated domain
78 also removes the veto from its executor and verifies that the separate
watchdog continues with zero output. A second ROS test in domain 79 reproduces
frequent scans with delayed dynamic TF and checks that processing continues.
