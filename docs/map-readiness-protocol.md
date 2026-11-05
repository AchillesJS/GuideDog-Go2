# Map readiness protocol — declared before the expanded Gazebo series

Baseline: `62cb4919676af870d836b6cfa90ca6e8ff25f0a9`.
Branch: `feat/map-validation`. Date: 5 November 2026.

This series evaluates the existing sparse flat-ground map as possible input
to a future planner. It does not enable turns, route execution, ground filling,
or a new motion policy. Actual real-cloud mapping, the direct LiDAR veto and
watchdog stay active. Box truth is used only by the independent test observer.

## Fixed scenarios

All boxes are static, 1 m high, 0.5 m deep, with individually verified contact
sensors. Positions below are in the initial robot-heading test frame. The same
initial truth-to-odom alignment is used through the series; it is not refitted
at later checkpoints.

| Scene | Box centres (forward, side), metres | Widths | Purpose |
| --- | --- | --- | --- |
| Two boxes | (2.0, −0.9), (2.7, +0.9) | 0.5, 0.7 m | Detect/localize more than one object; second box yaw +10° |
| Wide passage | (2.0, −1.25), (2.0, +1.25) | 0.5 m | Geometric gap 2.0 m |
| Narrow passage | (2.0, −0.55), (2.0, +0.55) | 0.5 m | Geometric gap 0.6 m |
| Blocked passage | (2.0, −0.55), (2.0, 0), (2.0, +0.55) | 0.5 m | Two 0.05 m gaps |
| Longer straight motion | (1.0, −1.2), (2.0, +1.2), (3.0, −1.2) | 0.5 m | Attempt actual 2.0 m forward progress with three side landmarks |

The first four scenes are stationary. At least three distinct fresh map stamps
are required per scene after 5.6 wall seconds for old occupied observations to
expire. Fixtures are created/probed far away and placed using Gazebo services;
only the benchmark's named boxes are removed. The original test box is moved
away by the existing preparation routine. No synthetic point clouds replace
the actual sensor stream in these Gazebo scenarios.

## Fixed geometric and timing criteria

- Each reference object: at least three associated occupied cells, boundary
  error p95 <= 0.20 m and maximum <= 0.30 m.
- Occupied cells are assigned to the nearest known rectangle boundary once,
  with a 0.50 m association gate. Report unassigned occupied cells separately;
  conditional association errors are not global map error or full object recall.
- Zero free cells wholly inside the known box footprints. To exclude uncertain
  edge cells, the cell-centre signed distance must be less than minus half a
  cell diagonal. Unknown interior cells are reported, not counted as free.
- Source-time ground-truth pose brackets <= 0.06 source seconds, no extrapolation.
  Map and status must match source time, frame, origin, resolution, dimensions
  and actual cell-content digest. Status receipt delay contributes to freshness.
- Odom-position error <= 0.20 m, mapper processing p95 <= 100 ms and persistent
  cell arrays <= 1 MiB. These have the same bounded interpretation as the
  previous map report; processing cost excludes report rendering and TF waits.

## Connectivity assessment

Unknown, occupied, out-of-map and stale data are unavailable. Connectivity uses
four neighbours; diagonal corner cutting is forbidden. No nearest-free start
is substituted for an unknown actual robot position.

For this diagnostic, enclose the configured front/rear 0.45 m and half-width
0.40 m rectangle in a rotation-invariant circle, then add a proposed assessment
margin of 0.10 m: radius approximately 0.702 m. Include grid cells whose squares
may intersect that circle. This is an explicit assessment profile, not a
validated physical body envelope, gait envelope or motion safety specification.

Report both point-free and whole-envelope component sizes, start/goal class,
reachable observed cells and the exact reason for rejected readiness. A 2.0 m
geometric gap is not proof that its full supporting floor was observed. The
sensor blind zone and sparse ground sampling remain unknown. The future goal
is three metres forward in the fixed initial test frame; no route is executed.

## Longer-motion limits

Requested speed stays 0.05 m/s. Target is 2.0 m **measured actual forward motion**,
not commanded distance. Limits: 0.30 m lateral displacement, 15° heading
excursion, body Z between 0.15 and 0.40 m, 180 source seconds or 600 wall seconds.
Any veto stop, contact, exceeded envelope or deadline ends the attempt with
zero requests and a failed case. Do not silently resume a failed attempt or
teleport/reorient the robot to claim the target.

Evaluate geometry about every five wall seconds, requiring at least eight fresh
paired evaluations and >=80% paired reference availability for a passing long
case. Save cell snapshots around each 0.25 m checkpoint and at the final stop.
Retain failed scenes and early stops in the report. Check contacts independently
and record actual post-zero motion using the existing settled-stop observer.

## Outcome and reproduction

The report separates geometric scenario results, the 2 m attempt and planning
readiness. Readiness requires the wide/two-object scenes to have a genuine
observed connection from the real start, plus passing validation cases. An
unknown start or disconnected observed floor is a blocking result even when
box localization is good. Do not relabel space to obtain a successful route.

```bash
~/guidedog_ws/src/GuideDog-Go2/scripts/test_obstacle_demo.sh --map-validation
```

For automated runs without a desktop, add `--headless`. This explicitly selects
Gazebo's server-only EGL rendering (`-s --headless-rendering`), retaining the
same world, sensor models, bridges, controllers, and criteria. The project
adapter changes only the one upstream Gazebo include's arguments and fails if
that upstream layout is ambiguous. The external pinned repository is unchanged.
Record `render_profile` with each run; desktop and server resource timings are
different profiles and must not be pooled. Actual scans, time-aligned truth,
controllers and positive contact controls remain mandatory in either profile.
The flags are documented in [Gazebo's headless rendering guide](https://gazebosim.org/api/sim/8/headless_rendering.html).

Run one simulation at a time (an unlocked Ubuntu desktop is needed for GUI runs). The benchmark
owns its scene and exits nonzero when readiness is not established. Its
`benchmark.json`, cell snapshots and motion trace preserve the findings. Failed
readiness is a measured limitation to resolve before planner integration.
