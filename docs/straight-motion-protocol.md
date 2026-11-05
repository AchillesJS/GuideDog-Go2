# Straight-motion diagnosis and acceptance protocol — 5 November 2026

Baseline: `8f022e04cac9896e0a00f15085aaaa7dfc40b4e1`, branch
`feat/straight-motion`. The previous map-readiness limits remain unchanged.

## Diagnosis

Record all six components on requested, candidate and checked command topics;
stamped Gazebo body truth, IMU and estimated odometry; joint-trajectory targets;
the effort controller's reference, feedback, tracking error and effort output.
These are observers only. Truth is not a controller input. A nonzero angular
request is not introduced to conceal gait drift. Preserve failed runs.

The initial instrumented stock run reproduces physical yaw drift despite zero
angular commands. Joint errors are small compared with the body motion, while
roll and yaw oscillate. This motivates a controlled gait-trajectory experiment;
it does not establish a single root cause or validate hardware behaviour.

## First controlled candidate

The explicit `low_lift` simulator profile changes only CHAMP's `gait.swing_height`
from 0.040 m to 0.015 m. It keeps the effort interface, PID values, physics,
friction, model, clock, other gait parameters and guard limits. It is selected
at construction, with actual parameter values recorded through ROS services.
The project owns its launch adapter; the pinned external repository stays clean.

Start with `--motion-only --headless --gait-profile low_lift --audit-motion`.
Each run starts a new owned simulation; do not resume an interrupted attempt
or move/reset the robot to count a successful distance.

## Fixed motion acceptance

Actual progress >=2.0 m, request 0.05 m/s, lateral displacement <=0.30 m,
heading excursion <=15 degrees, body Z within 0.15–0.40 m, no robot/obstacle
contact, no LiDAR veto, <=180 source seconds and <=600 wall seconds. On a
violation, request zero and measure the actual settled stop. A failure is not
fixed by increasing these bounds or removing the independent command watchdog.

Require three independent passing 2 m attempts before declaring the selected
simulator profile repeatable under these scenarios. Passing motion alone is
separate from passing map geometry, odometry error and reference availability.
The original long-map criteria still require >=8 paired observations, >=80%
reference availability, odom position error <=0.20 m and the predeclared box
boundary/free-cell/resource bounds. Re-run the original STOP and short-map
regressions for any profile made available for ordinary operation.

Map connectivity remains a separate gate. Do not fill unknown cells, replace
the actual unknown start with a nearby free seed or use truth to supply floor.
Stable motion is not a completed navigation or physical-robot result.

## Calibration following the first low-lift run (before hold-out runs)

The first low-lift attempt achieved 2 m, but estimated about 1.13 m. A fixed
scalar was fitted offline to 2310 source-time-paired odom/truth positions from
that calibration run, with brackets <=0.06 s. The fitted odom scalar is 1.59914;
select **1.60** for the explicit `low_lift_calibrated` profile. Hold it fixed
through independent validation; do not count the calibration run as a hold-out.
In-sample residual p95 is 0.0831 m, maximum 0.1006 m: these are calibration
residuals, not validation accuracy. The estimator remains the pinned kinematic
algorithm. This empirical scale applies only to this simulated gait/surface
profile and is not a hardware calibration or a fix for all contact-estimation
defects. Runtime estimation receives no truth data.

Motion envelope evaluation includes all received body poses through the settled
stop, not only the five-second progress snapshots.
