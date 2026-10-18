# GuideDog-Go2

**An experimental robotic guide-dog prototype built around Unitree Go2, ROS 2, and computer vision.**

GuideDog-Go2 explores whether a quadruped robot can perceive obstacles, represent nearby space, plan human-aware routes, and eventually assist a visually impaired person. The initial objective is a **simulation-only proof of concept**—not a deployable mobility aid.

> [!WARNING]
> **Research/development prototype only.** No autonomous guidance, safe human-leading behavior, or real-world accessibility performance has been validated. Do not use this project as a mobility aid or rely on it for personal safety.

## Current status

**Milestone M0 — Project bootstrap (in progress).**

The following have been demonstrated **locally** in an Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic environment:

- Simulated Unitree Go2 launches and accepts keyboard teleoperation.
- Simulated RGB camera and 4D LiDAR publish Gazebo and ROS 2 messages.
- A custom ROS 2 Python RGB subscriber receives `640 × 480` `rgb8` images on `/rgb_image`.
- A diagnostic Python/OpenCV script successfully saved a camera frame as PNG.

**Not yet implemented:** YOLO inference, object tracking, sensor fusion, world model, route planning, safety supervisor, human guidance, or hardware deployment. The initial `guidedog_perception` ROS 2 package is available in this repository.

## Intended capabilities (roadmap, not current features)

1. **Perception** — detect and track people, obstacles, and potentially traversable space using the robot's built-in camera.
2. **Geometry** — use the built-in 4D LiDAR, IMU, and odometry for spatial understanding.
3. **World model** — fuse semantic detections with local obstacle geometry.
4. **Human-aware navigation** — generate paths and motion commands appropriate for an accompanied pedestrian.
5. **Independent safety layer** — conservatively stop when perception, localization, or system state is unreliable.
6. **Validation** — test scenarios in simulation before considering supervised hardware experiments.

**Hardware constraint:** the project currently targets **stock Go2 sensors**; added cameras, LiDARs, and external compute hardware are not part of the initial baseline. Simulation of the L1-style LiDAR is **not** a validated digital twin of every physical Go2 sensor variant.

## Stack

- Ubuntu 24.04 (currently tested in an ARM64 UTM virtual machine)
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3, NumPy, OpenCV
- External Go2 simulator: [khaledgabr77/unitree_go2_ros2](https://github.com/khaledgabr77/unitree_go2_ros2)

YOLO and an inference runtime have **not** been integrated yet.

## Repository layout

```text
GuideDog-Go2/
├── README.md
├── .gitignore
├── CONTRIBUTING.md
├── docs/
│   ├── architecture.md
│   ├── setup.md
│   ├── roadmap.md
│   ├── team.md
│   └── limitations.md
├── guidedog_perception/     # ROS 2 RGB subscriber (implemented)
└── LICENSE
```

The third-party simulator is an **external dependency** and is not vendored into this repository. Planned subsystems are documented, not represented as completed packages.

## Getting started

Start with the step-by-step [partner onboarding guide](docs/partner-onboarding.md), or see [docs/setup.md](docs/setup.md) for the tested baseline and commands to launch the RGB subscriber.

## Architecture and plans

- [System architecture](docs/architecture.md)
- [Partner onboarding (start here)](docs/partner-onboarding.md)
- [Setup and reproducibility](docs/setup.md)
- [Development roadmap](docs/roadmap.md)
- [Team workflow](docs/team.md)
- [Known limitations and safety](docs/limitations.md)
- [Contributing](CONTRIBUTING.md)

## Collaboration

A two-person team is building the project. Module ownership and interfaces will be agreed jointly; see [docs/team.md](docs/team.md). Work is tracked with GitHub Issues and reviewed via pull requests.

## Third-party components and licensing

This repository does **not** include proprietary client/employer materials, third-party model weights, or the source code of the external Go2 simulator. Those dependencies retain their own licenses.

Original GuideDog-Go2 code is published under the [Apache License 2.0](LICENSE). Third-party components remain subject to their own licenses. Contributors should confirm they have the right to submit their work.
