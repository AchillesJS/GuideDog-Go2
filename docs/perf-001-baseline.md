# PERF-001: instrumented UTM baseline, 2026-11-04

**Status: reproduced, not resolved.** Sensor diagnostics and one-command bringup
are implemented on `feat/sensor-diagnostics`. No transport, bridge direction,
sensor frequency, controller or third-party simulator changes were made.

## Environment and procedure

- Ubuntu 24.04 ARM64/aarch64 in UTM; about 3890 MiB guest RAM.
- ROS 2 Jazzy, Python 3.12.3, Gazebo Sim 8.15.0.
- External simulator: `55ff151632a0220bf3ae0c0772392615d86da7a8`, unchanged.
- GuideDog base: `ad49960` with the uncommitted sensor-diagnostics feature.
  Reports explicitly record the source working tree as dirty.
- Default upstream world and Go2 model; RGB and L1-like LiDAR enabled;
  D455, Velodyne and RViz disabled. GUI enabled, no motion commands sent.

After building the package, the final run used:

```bash
~/guidedog_ws/src/GuideDog-Go2/scripts/run_sim.sh \
  duration_sec:=45.0 probe_duration_sec:=45.0
```

The test harness stopped the remaining simulation with SIGINT after 60 wall
seconds. Its timeout exit code 124 was expected. Native probes exited with code
0; the observer saved its final report at about 45 seconds. All test processes
were subsequently confirmed stopped. The two controllers activated successfully.

## Observations

ROS values below are medians of rolling-window snapshots at/after 20 seconds,
excluding the duplicate final snapshot. Gazebo values are the latest native
CLI window in the final report. **These windows differ**; their ratio is not a
measured message-loss percentage.

| Stream | ROS receipt rate, wall Hz | Latest Gazebo native receipt rate, wall Hz | TF to base at latest message stamp |
| --- | --- | --- | --- |
| RGB | 3.17 | 9.09 | Available |
| L1-like LiDAR | 5.20 | 9.08 | Available |
| IMU | 90.84 | Not probed | Available |
| Odometry | 42.45 | Not probed | Available |

The final clock snapshot estimated simulation/wall speed at 0.87. CPU busy was
85.9% over a roughly five-second sampling interval, and available guest memory
was about 1377 MiB. These describe this VM/run only. Source timestamps had no
observed regressions and all four streams had no final timing/TF issues.

An earlier exploratory 75-second run also reproduced lower ROS receipt rates
(RGB median 3.44 Hz, LiDAR 5.33 Hz). Its older observer version resampled CPU
immediately during shutdown; that final CPU value must not be used. The final
version reuses the last valid CPU sample for subsecond report updates.

The Gazebo log contained EGL/renderer fallback warnings. This is an observation,
not an established explanation for PERF-001. The transport/subscriber timing
difference remains open and needs controlled profiling, including a run with
native probes disabled to quantify observer overhead.

## Validation

- Package build succeeded.
- `colcon test --packages-select guidedog_perception`: **12 passed, 1 skipped**.
- The skipped copyright-header test was already disabled in the repository.
- Checks include an isolated ROS test that publishes sensors and static TF,
  then freezes the clock/stops sensors and verifies stale warnings and report files.
- `git diff --check` passed; external simulator working tree remained clean.
- Full launch, both native probes, final JSON/Markdown/JSONL reports and shutdown
  were exercised in the installed Ubuntu environment.

This records one machine's timing baseline. It does not close M0's two-developer
reproduction gate, prove calibration accuracy, or implement autonomous stopping.

See [diagnostics usage and metric definitions](sensor-diagnostics.md).
