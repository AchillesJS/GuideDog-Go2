# Rolling local evidence map — first flat-ground increment

The new `local_mapper` observes real LiDAR and production TF without publishing
velocity or subscribing to Gazebo ground truth. It adds an odom-aligned rolling
8 × 8 m grid with 0.10 m cells. The existing direct-LiDAR veto and watchdog
remain the command path. This map is not yet consumed by a planner.

## Run and view

Build once from Ubuntu after updating the package:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/guidedog_ws
colcon build --packages-select guidedog_perception --symlink-install
~/guidedog_ws/src/GuideDog-Go2/scripts/run_local_map.sh
```

This starts the protected stop demo plus the mapper. RGB stays disabled in
this measured profile. The usual second terminal command still drives it:

```bash
~/guidedog_ws/src/GuideDog-Go2/scripts/drive_guarded.sh
```

`i` requests the verified 0.05 m/s profile; `k` or space sends neutral.
Clear the fault and send neutral before requesting movement again.
The scripts use the existing ROS domain 42 / Gazebo partition isolation.

The launch prints its new report directory under `~/.ros/guidedog/reports/`.
Open **`mapping/local_map.svg`** inside that directory to view the most recent
map. Reload the image after movement; the file updates every two wall seconds.
Green is observed floor, red is obstacle evidence, gray is unknown. Blue marks
the last observed robot pose and a white line its heading.

Files in `mapping/`:

| File | Meaning |
| --- | --- |
| `local_map.svg` | View of the actual occupancy cells |
| `local_map_report.json` | Health, last source stamp, age, cell counts, parameters and update cost |
| `local_map.npz` | Occupancy, per-cell age, evidence score, ground height and scatter arrays |

The ordinary `run_obstacle_demo.sh` retains mapping disabled by default.
`enable_local_map:=true` enables it explicitly. `local_mapper` can also run
independently against an existing simulator, with `use_sim_time:=true`.
Use one simulator at a time on the 4 GiB VM.
The wrappers select X11/XCB for Qt and reject a second launch in the same
ROS/Gazebo channels. Ctrl+C, terminal hangup and Gazebo exit clean up that
launch. See the [GUI startup fix and validation](gui-startup.md).

## Message and time contract

- `/guidedog/local_map`: `nav_msgs/msg/OccupancyGrid`, frame `odom`, identity
  origin orientation, row-major cells, published at at most 5 wall Hz.
  Values are **-1 unknown, 0 observed floor, 100 occupied**. They are classes,
  not calibrated occupancy probabilities. Resolution is metres/cell; the origin
  is the lower-left cell corner in odom metres.
- `header.stamp` is the last accepted **source cloud timestamp**, including
  when publishing a changed expiry snapshot. It is never advanced merely
  because a publication timer fired. A reset clears it to zero.
- Map publisher: reliable, transient local, keep last one. A compatible late
  subscriber receives the latest snapshot. Input LiDAR uses reliable depth one.
  A best-effort-only publisher requires a separately configured adapter.
- `/guidedog/local_map_status`: JSON in `std_msgs/msg/String`, reliable depth
  one. `healthy` requires advancing ROS time, accepted data younger than 0.5
  wall seconds, and a valid observation reason. Consumers must inspect this
  status and age; a retained occupied cell is not evidence of current visibility.
- `snapshot_digest` pairs the status with the exact grid cells, source stamp,
  origin, float32 ROS resolution and dimensions. Ageing can change cells without
  changing the source stamp, so matching the stamp alone is insufficient.
  The benchmark also requires frame agreement and adds elapsed consumer cache
  time to the publisher's reported receipt age. This does not measure transport
  latency or establish a synchronized end-to-end deadline.
- Cloud -> `base_footprint` -> `odom` transforms use the cloud's exact stamp.
  A queue of eight waits at most 0.25 wall seconds for TF. The newest available
  transformable cloud is processed, with at least 0.20 wall seconds between
  updates. Cloud ROS-clock age must be between -0.05 and +0.50 seconds.
- Duplicate/out-of-order **received** source stamps are rejected before queuing,
  including a duplicate of a frame that never completed TF processing.
- ROS clock reset clears every layer, pending cloud and stamp watermark.
  An odometry translation jump over 1 m between accepted updates discards spatial
  history. Cells shift by whole resolution steps, never rotate with yaw, and
  newly exposed strips start unknown. No global map is required.

For RViz, set Fixed Frame `odom` and add a Map display on `/guidedog/local_map`
with reliable/transient-local QoS. The lightweight saved SVG avoids an extra
RViz process in the benchmark.

## Evidence rules and limits

The current flat-world support plane is fitted from the same fresh scan using
the existing bounded floor fitter. There is no cached support plane. Self/body
returns inside `abs(x) <= 0.45`, `abs(y) <= 0.40` do not establish free ground.

At least three ground returns within 0.025 m of support are required to mark a
cell observed free. At least two returns 0.08–1.50 m above support establish an
occupied cell. Any nonfloor return blocks fresh free classification, including
a single sparse obstacle, a low step or an unsupported height. Ambiguous cells
are unknown unless valid remembered occupied evidence remains.

The implementation does **not ray-clear ground between the sensor and a
return**. A ray passing above a hole cannot establish supporting ground.
Cells with no relevant returns remain unknown. Therefore the map is deliberately
sparse; a large gray area is an expected representation of missing evidence.

Occupied evidence persists for up to five wall seconds through occlusion;
free ground expires after two. Expired observations become unknown. Three
consecutive distinct accepted scans of observed ground can clear a remembered
obstacle sooner. A missing/ambiguous scan interrupts that clearing streak.
No timestamp replay renews expiry. Occupied evidence dominates fresh floor
until it is explicitly cleared or expires.

`local_map.npz` layers share the grid indices:

- `occupancy`: int8 classes, as published.
- `age_sec`: wall seconds since the evidence establishing the current class;
  unknown is NaN.
- `evidence_score`: supporting scan count divided by three, capped at one.
  This is an evidence indicator, not a calibrated confidence probability.
- `ground_z_odom_m`: mean observed ground-return Z in odom metres.
- `ground_rms_m`: within-cell standard deviation of those Z returns, in metres.
  This includes point noise and pose error; it is not a certified roughness or
  traversability measure. Height/scatter expire with observed floor support.
- `origin_odom`, `resolution`, `source_stamp_ns`: snapshot coordinates/time.

Ground alignment still relies on production state estimation and the flat
support model. The first increment does not provide terrain slope layers,
validated holes/edges/stairs, continuous ground coverage, human clearance,
navigation or hardware operation. Age and evidence scores do not calibrate
odometry uncertainty. Turning a sparse map into a traversability decision
requires a separately tested planner contract.

## Reproduce tests

```bash
~/guidedog_ws/src/GuideDog-Go2/scripts/test_obstacle_demo.sh --map-benchmark
~/guidedog_ws/src/GuideDog-Go2/scripts/test_obstacle_demo.sh --with-local-map
```

Run sequentially from an unlocked Ubuntu desktop. The first creates real-cloud
scenarios for localization, motion/rolling, occluded floor, blackout, expiry,
recovery and box removal. The second repeats the original free/side/front,
rearm and scan/command-loss cases with mapping running. Both own and stop their
simulation processes and refuse a reused nonempty report directory.

The mapper uses only TF and LiDAR. The benchmark independently aligns ground
truth to odom once while stationary, then uses that same reference after motion.
Ground truth and box dimensions are acceptance measurements, not mapper inputs.
Saved `map-*.json` / `map-*.svg` preserve each phase rather than only the final
expired map.

The matrix's predefined limits are box-boundary p95 <= 0.20 m, at least three
occupied cells near the box, >= 80% unknown cells in its defined ground shadow,
actual motion >= 0.05 m, cell storage <= 1 MiB and processing p95 <= 100 ms.
Processing cost covers cloud decoding, projection, floor fitting and cell update;
it excludes TF waiting, ROS serialization and report rendering. Persistent cell
storage is not total node or VM memory.

Unit/ROS checks add observed-only free cells, mixed/sparse/unsupported returns,
explicit clearing, fixed memory, four roll directions, reset/jump behavior,
height/age layers, late map delivery, delayed/missing TF, clock freeze/reset and
replay. See the dated [validation record](local-map-validation.md) for measured
results and retained failed test attempts.
