# One-command simulation and sensor diagnostics

This adds measurements for PERF-001. It does not assume a cause or change the
pinned external simulator, bridge directions, sensor rates, QoS, or motion control.

## Build once after updating the source

From the ROS workspace (this machine uses `~/guidedog_ws`):

```bash
source /opt/ros/jazzy/setup.bash
cd ~/guidedog_ws
colcon build --packages-select guidedog_perception --symlink-install
```

The simulator must already be installed in this workspace. Runtime dependencies
are declared in `guidedog_perception/package.xml`; use rosdep for a fresh setup.

## Run

```bash
~/guidedog_ws/src/GuideDog-Go2/scripts/run_sim.sh
```

The script locates its workspace, sources Jazzy and the workspace, then starts:

- The existing Go2 simulator with RGB and L1-like LiDAR enabled, D455/Velodyne/RViz disabled.
- A lightweight observer for RGB, LiDAR, IMU, odometry, `/clock`, and TF.
- Two native `gz topic -f` probes with a default 60-second measurement window
  and up to five seconds of startup allowance before bounded cleanup.

The Gazebo GUI uses the current Ubuntu desktop. Run this from its terminal.
The upstream `gui` launch argument is unused by the pinned simulator, so this
wrapper deliberately does not advertise a headless switch.

Ctrl+C stops the launch and its probes. It saves a final sensor snapshot.
The observer sends no motion commands and is not a safety supervisor.

If the simulator is already running, avoid launching a second copy:

```bash
~/guidedog_ws/src/GuideDog-Go2/scripts/run_sim.sh start_simulator:=false
```

## Reports

Each run prints a new directory under `~/.ros/guidedog/reports/`:

| File | Contents |
| --- | --- |
| `report.md` | Latest readable sensor summary |
| `report.json` | Latest snapshot with parameters, revisions, clock, resources, sensor statistics and issues |
| `samples.jsonl` | One snapshot every five wall seconds, plus the final snapshot |
| `gazebo-rgb.log`, `gazebo-lidar.log` | Native Gazebo frequency output, for comparison with ROS receipt rates |
| `gazebo-probes.json` | Probe commands, exit codes, elapsed time and interruption status |

The readable report also includes whole-VM CPU/RAM and the latest native GZ
rates. Native and ROS windows differ; JSON records native log age so an old
probe result is not mistaken for a current measurement after the probe ends.

No image or point-cloud payloads are saved. Statistics retain bounded timestamp
windows (at most 2048 observations per stream). JSONL grows by one small record per
report interval; keep runs bounded when disk space is limited.

Settings can be passed through the launch command, for example:

```bash
~/guidedog_ws/src/GuideDog-Go2/scripts/run_sim.sh \
  duration_sec:=90.0 probe_duration_sec:=90.0 report_period_sec:=5.0
```

`duration_sec` stops only the observer after its next report interval; the
simulation remains running until Ctrl+C. Zero means continuous observation.
Use a new empty `report_dir` for each run to avoid mixing sessions.
`record_gazebo:=false` disables the two extra native subscribers when measuring
their overhead. `world:=/absolute/path/world.sdf` selects a different test scene.

## Interpretation

- `received_hz_wall` counts received messages per real second. It is not the
  publisher's production rate.
- `received_hz_source_time` counts received messages per second of header time.
  With simulated sensors, this is simulation time. Never compare these two rate
  columns as evidence of dropped messages.
- `/clock`'s `source_time_per_wall_second` estimates simulation speed over the
  recent window. Receipt frequency and simulation speed describe different things.
- Age compares a header timestamp with ROS time, only after a simulation clock
  has been received. Negative ages over 50 ms are flagged, not clamped to zero.
  This is ROS-clock data age, not a synchronized physical sensor-to-actuator latency.
- Freshness and timers use a monotonic wall clock. Pausing Gazebo therefore
  produces stale warnings even though simulated time no longer advances.
- TF is checked from the received frame to `base_link` at the message timestamp.
  A frame string alone is not sufficient. RGB and LiDAR expected frame IDs are
  `camera_link` and `lidar_l1_link`. This checks TF availability, not calibration
  accuracy, camera optical conventions, or complete sensor fusion.
- `WARN` means inspect the listed issues: startup, missing/stale data, unavailable
  TF, wrong frames, old/future stamps or a clock reset. The default 2-second
  freshness threshold is a diagnostic setting, not a validated stopping limit.
- CPU busy percentage and available memory describe the whole Ubuntu VM, not
  the Mac host. CPU percentage needs two samples; the first value is null.

To investigate PERF-001, compare the native Gazebo logs, ROS wall rates and
simulation speed during the same steady period after spawning/controllers finish.
Repeat the same scene and resource allocation, and compare with probes disabled.
Do not claim loss percentages or a fixed performance improvement from rate ratios.

## Checks

```bash
source /opt/ros/jazzy/setup.bash
cd ~/guidedog_ws
colcon test --packages-select guidedog_perception --event-handlers console_direct+
colcon test-result --verbose
```

Timing tests cover wall/source clock separation, window expiry, bounded storage,
clock resets, invalid frames, future timestamps and stale streams. A ROS integration
test publishes real messages and static TF in isolated domain 77, then freezes
the clock and stops sensors to verify stale detection and report files.

References: [ROS topic-rate interpretation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#ros2-topic-hz),
[TF lookup time](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Tf2.html).

The [recorded UTM baseline](perf-001-baseline.md) contains the first measured
results and the remaining PERF-001 investigation limits.
