# Roadmap

This roadmap describes **proposed work**; milestones require demonstrated acceptance tests.

## M0 — Project bootstrap (current)

- [x] Simulated Go2 launched and keyboard controlled locally.
- [x] Camera and LiDAR topics discovered in Gazebo and ROS 2.
- [x] Local RGB subscriber received `640×480` `rgb8` frames.
- [x] Local OpenCV screenshot captured and visually inspected.
- [x] Public GitHub repository created.
- [ ] Commit the working local ROS 2 package and pin external simulator revision.
- [ ] Agree module ownership, review process, and a license.
- [ ] Confirm setup independently on the second developer's machine.

## M1 — Perception foundation

- [ ] Establish data/message contracts for detections.
- [ ] Run a minimal object detector on camera frames.
- [ ] Publish ROS 2 detections with timestamps, confidence, and frame identifiers.
- [ ] Reproduce baseline performance measurements and record latency.

## M2 — Geometry and world model

- [ ] Parse LiDAR PointCloud2 and verify valid coordinate transforms.
- [ ] Design/test local obstacle representation.
- [ ] Match visual detections to geometry where defensible.
- [ ] Establish explicit stale-data and low-confidence behavior.

## M3 — Navigation proof of concept (simulation only)

- [ ] Define traversability and safe local motion envelopes.
- [ ] Demonstrate obstacle-aware local planning.
- [ ] Add an independent STOP/veto mechanism with fault injection.
- [ ] Evaluate on repeatable scenes with documented failures.

## M4 — Human-aware simulation scenarios

- [ ] Test moving pedestrians, occlusions, narrow passages, and unexpected obstacles.
- [ ] Define human accompaniment interaction assumptions.
- [ ] Record task completion, collision/near-miss, response latency, and failure cases.

## Beyond simulation

Physical deployments require separate hardware-specific validation, verified sensor correspondence, emergency stop and human factors work, and suitable oversight. **No timeline or authorization for real-world guidance is implied.**

## Known performance issue

`PERF-001`: camera and LiDAR generated ~9 Hz in Gazebo but ROS 2 Python subscribers received ~2–4 Hz in UTM. Root cause unresolved. Keep this issue visible; measure again when integrating perception.
