# Obstacle-stop validation — 2026-11-04

The forward-only flat-ground simulation demo passed two complete Gazebo runs
on the existing Ubuntu ARM64 UTM VM. This validates the bounded box scenarios
below; it does not complete the planned safety supervisor or hardware gates.

## Reproduction

- Ubuntu 24.04.5, aarch64, ROS 2 Jazzy, Gazebo Sim 8.15.0, Python 3.12.3.
- Approximately 4 GiB VM RAM, four virtual CPUs; only one Gazebo session active.
- GuideDog base commit: `ad49960aeae83899a9e2d52aea4c30af1343711f`.
- Local development branch: `feat/lidar-stop`; changes are uncommitted.
- External simulator unchanged at `55ff151632a0220bf3ae0c0772392615d86da7a8`.
- Command cap 0.05 m/s; scan timeout 1.50 wall seconds; request timeout 0.40 s.
- Estimated stop clearance 0.80 m; configured front extent 0.45 m, half width 0.40 m.
- RGB disabled in this profile; native Gazebo rate probing disabled.
- Test domain 43, unique Gazebo partition per run. Manual demo uses domain 42.

```bash
~/guidedog_ws/src/GuideDog-Go2/scripts/test_obstacle_demo.sh
```

Each run starts and stops its own scene. The test relays actual simulated LiDAR
messages, records `/odom/ground_truth`, checks the controller's final input,
and injects faults by withholding data. It does not generate replacement clouds.

## Results

Reports in Ubuntu:

```text
~/.ros/guidedog/reports/stop-benchmark10-20261104/
~/.ros/guidedog/reports/stop-benchmark11-20261104/
```

| Measurement | Run 10 | Run 11 |
| --- | ---: | ---: |
| Overall | PASS | PASS |
| Free-path forward progress | 0.153 m | 0.093 m |
| Side-obstacle forward progress | 0.122 m | 0.107 m |
| Front-obstacle forward progress | 0.172 m | 0.177 m |
| Front stop: ground-truth envelope clearance | 0.735 m | 0.611 m |
| Front stop: first LiDAR estimated clearance | 0.797 m | 0.785 m |
| Front stop: observed final zero after status receipt | 0.025 s | 0.037 s |
| Scan-loss injection to observed final zero | 1.443 s | 1.404 s |
| Command-loss injection to observed final zero | 0.437 s | 0.443 s |
| Maximum post-zero forward travel across all cases | 0.0178 m | 0.0102 m |
| Robot/box contacts | 0 | 0 |
| Positive-control probe contacts | 147 | 179 |

Run 11 adds explicit report fields for the verified command route and physical
motion before both fault injections (0.042 m before scan loss; 0.055 m before
command loss). During the held-forward rearm check, every observed final command
remained zero after obstacle removal; measured forward displacement was below
0.000003 m. A neutral request was required to resume.

The contact monitor was positively tested by dropping a named probe onto the
distant box and then removing it before robot motion. Silence from a contact
topic alone was not accepted as proof that the sensor worked.

The last diagnostic windows showed LiDAR receipt rates of 6.14 and 5.47 Hz,
simulation/wall factors of 0.95 and 0.97, CPU busy values of 72.68% and 76.62%,
and available RAM around 1764–1766 MiB. These profile observations do not close
PERF-001 or establish publisher frequency/message loss.

## Automated validation

`colcon build --packages-select guidedog_perception --symlink-install` passed.
`colcon test` and `colcon test-result --verbose` reported **34 passed, one
pre-existing skipped copyright check, zero errors/failures**. Four warnings
from the lint runner concern forking in a multithreaded Python process.

Coverage includes point-cloud layout/endian/padding, bounded ground-plane
fitting and rejection, obstacle and side filtering, invalid commands,
timestamp/clock faults, explicit rearm, delayed exact-time dynamic TF, and
the independent output watchdog when the veto executor stops. Process-tree
cleanup is tested with a detached child and an unrelated process.

`git diff --check` passed. The final process audit found no remaining project
ROS/Gazebo processes; the external simulator working tree was clean.

## Interpretation and limits

Timing is observer wall-clock receipt timing. Post-zero travel is the maximum
forward ground-truth displacement over the next two wall seconds; final pose
spread is checked separately. It is not total path length or hardware braking.

LiDAR-derived and ground-truth clearances differ because the prototype relies
on an estimated footprint frame and a configured body envelope. The 0.80 m
threshold did **not** produce a guaranteed physical 0.80 m clearance: observed
ground-truth clearances were 0.61–0.74 m in these two runs. Two runs do not
establish a statistical reliability bound.

This demo supports slow forward commands on observed flat support and a large
box. Turning, reverse, low/overhead objects, cliffs, stairs, route planning,
human guidance, hardware deployment, and simultaneous loss of both veto and
watchdog remain outside the validated scope. See [the demo guide](obstacle-stop.md).
