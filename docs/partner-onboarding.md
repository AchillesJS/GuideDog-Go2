# Partner onboarding — GuideDog-Go2

**Audience:** the second developer joining the project.  
**Goal:** reproduce the current simulation baseline, run the GuideDog RGB subscriber, and open a first pull request.

> **Status:** this is a working *setup recipe* assembled from the first developer's successful Ubuntu ARM64 session and the upstream Go2 simulator instructions. It has **not yet been independently validated** on a second machine. Please record discrepancies in a GitHub Issue.
>
> **Safety:** this is a simulation-only prototype, **not** a validated mobility aid.

## 0. What you are joining

- Project: https://github.com/AchillesJS/GuideDog-Go2
- Project-owned code: `guidedog_perception` (ROS 2 Python package).
- External dependency: [khaledgabr77/unitree_go2_ros2](https://github.com/khaledgabr77/unitree_go2_ros2) (Go2 simulator, robot description, CHAMP controllers, Gazebo bridges).
- Runtime: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, Python 3.
- **Implemented:** teleoperated simulated Go2, ROS 2 camera + LiDAR streams, RGB frame metadata subscriber.
- **Not implemented:** YOLO, tracking, LiDAR fusion, world model, autonomous navigation, human-guidance behavior.

**Do not** copy external Go2 packages into the GuideDog Git repository. Each developer installs those packages alongside GuideDog in the same ROS 2 workspace.

## 1. Machine and Ubuntu

Use **Ubuntu 24.04 LTS** (native installation preferred if available). Either `amd64` or `arm64` can work with ROS 2 Jazzy, but only **arm64 in UTM** has been exercised by our team so far.

On macOS, use an Ubuntu Linux VM. A working reference configuration used about **6 virtual CPUs, 8 GiB RAM, and a 120 GiB virtual disk**, but these values are *observations*, not guaranteed requirements. Gazebo graphics performance depends on GPU/VM drivers.

Verify OS:

```bash
lsb_release -a
uname -m
```

Expected Ubuntu release: `24.04`. Architecture: `x86_64` or `aarch64`.

## 2. Install ROS 2 Jazzy and Gazebo Harmonic

On a **fresh** Ubuntu install, follow the official ROS 2 Jazzy Debian/apt instructions to enable the ROS apt repository, then install the desktop environment:

- [Official ROS 2 Jazzy Ubuntu installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Official Gazebo / ROS compatibility and installation](https://gazebosim.org/docs/latest/ros_installation/)

After the repository is configured:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-dev-tools \
  git \
  python3-opencv \
  python3-numpy
```

Install common dependencies from the external simulator's setup guidance:

```bash
sudo apt install -y \
  ros-jazzy-xacro \
  ros-jazzy-robot-localization \
  ros-jazzy-ros2-controllers \
  ros-jazzy-ros2-control \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-velodyne \
  ros-jazzy-velodyne-description
```

The upstream simulator also lists `ros-jazzy-gazebo-ros2-control`, a naming combination associated with a different Gazebo integration. Our known working setup uses **Gazebo Harmonic** and `gz_ros2_control`. If apt cannot find a package, stop and record the exact error rather than swapping Gazebo versions.

Sanity check:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
gz sim --version
python3 -c 'import rclpy, cv2, numpy; print("ROS2/OpenCV/NumPy: OK")'
```

**Do not install Ultralytics, PyTorch or ONNX Runtime for milestone M0.** Object inference is not part of this baseline.

## 3. Clone the two repositories into one workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# External simulator (keep as separate repository)
git clone https://github.com/khaledgabr77/unitree_go2_ros2.git

# Our code
git clone https://github.com/AchillesJS/GuideDog-Go2.git
```

Expected layout:

```text
~/ros2_ws/
└── src/
    ├── unitree_go2_ros2/       # third-party simulator + CHAMP
    └── GuideDog-Go2/
        ├── docs/
        └── guidedog_perception/
```

**Reproducibility gap:** the exact upstream simulator commit from the first developer's machine still needs recording. Until then, both developers should record their revisions before building:

```bash
git -C ~/ros2_ws/src/unitree_go2_ros2 rev-parse HEAD
git -C ~/ros2_ws/src/GuideDog-Go2 rev-parse HEAD
```

If a pinned simulator SHA is added to this document later, run `git checkout <SHA>` **inside the simulator repository** before building.

## 4. Resolve dependencies and build

```bash
source /opt/ros/jazzy/setup.bash

# Only initialize rosdep if not initialized on this Ubuntu installation
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

If `rosdep` reports unresolved dependency keys, capture the names and attach the output to an Issue. This third-party simulator may need manual dependency resolution; we have not verified a clean install on a second machine.

Check both packages:

```bash
ros2 pkg prefix unitree_go2_sim
ros2 pkg prefix guidedog_perception
```

Both should resolve to locations under `~/ros2_ws/install/`.

**Important:** source ROS 2 and the workspace in **each new terminal**:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 5. Start the simulated Go2

**Terminal A:**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch unitree_go2_sim unitree_go2_launch.py \
  rviz:=false \
  disable_d455:=true \
  disable_velodyne_lidar:=true \
  disable_camera:=false \
  disable_lidar_l1:=false
```

Wait for Gazebo to load the world, spawn Go2, and activate its controllers. This baseline deliberately enables only the simulated mono RGB camera and L1-style LiDAR while disabling the additional D455 and Velodyne.

**Terminal B — optional keyboard control:**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Focus the keyboard-control terminal while driving. Test only in simulation.

## 6. Verify sensors before running GuideDog

**Terminal C:**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 topic info /rgb_image
ros2 topic echo /rgb_image --once --field header
ros2 topic info /unitree_lidar/points
```

Expected topics and types:

| Topic | ROS 2 type | First developer's local observation |
| --- | --- | --- |
| `/rgb_image` | `sensor_msgs/msg/Image` | 640×480, `rgb8`, frame `camera_link` |
| `/unitree_lidar/points` | `sensor_msgs/msg/PointCloud2` | frame `lidar_l1_link` |
| `/imu/data` | `sensor_msgs/msg/Imu` | bridge exists; no GuideDog IMU consumer yet |

If more detail is needed:

```bash
ros2 topic info -v /rgb_image
gz topic -f -t /rgb_image
gz topic -f -t /unitree_lidar/points
```

On the original UTM machine, Gazebo produced about **9 Hz** for both RGB and LiDAR, while ROS 2 Python subscribers received roughly **2–3 Hz** RGB and **3–4.5 Hz** LiDAR. This is a **known unresolved performance issue**, not a target specification. Do not change bridge direction or report end-to-end real-time performance based on Gazebo's source rate alone.

## 7. Run the project-owned ROS 2 node

While the simulation is running, in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run guidedog_perception rgb_node
```

Expected pattern (timestamps and frequency vary):

```text
[INFO] [guidedog_rgb]: GuideDog RGB subscriber started
[INFO] [guidedog_rgb]: Frame #5 | 640x480 | encoding=rgb8 | frame=camera_link
[INFO] [guidedog_rgb]: Frame #10 | 640x480 | encoding=rgb8 | frame=camera_link
```

**M0 onboarding PASS** means all of the following are independently confirmed:

- [ ] Go2 appears in Gazebo.
- [ ] Robot responds to keyboard teleoperation.
- [ ] ROS 2 sees `/rgb_image` and `/unitree_lidar/points`.
- [ ] `ros2 run guidedog_perception rgb_node` receives and logs frames.
- [ ] Simulator and GuideDog commit SHAs, OS/CPU architecture, and observed FPS are recorded.

Do **not** infer YOLO or safe autonomy from these checks.

## 8. Git workflow — first contribution

Ask the repo owner to invite you as a GitHub collaborator before pushing a branch. [GitHub collaborator instructions](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/repository-access-and-collaboration/inviting-collaborators-to-a-personal-repository).

```bash
cd ~/ros2_ws/src/GuideDog-Go2
git pull --ff-only origin main
git switch -c docs/partner-setup-feedback

# Edit documentation to record what worked or broke on your machine.
git status
git add docs/
git commit -m "docs: record second-machine setup findings"
git push -u origin docs/partner-setup-feedback
```

Then open a Pull Request targeting `main`. Do not commit `build/`, `install/`, `log/`, model weights, private credentials, personal images or proprietary materials.

If using HTTPS, GitHub account passwords are **not** accepted by Git. You can use `gh auth login --web --git-protocol https` and `gh auth setup-git` to authenticate.

## If something fails

Capture:

```bash
lsb_release -a
uname -m
ros2 doctor --report
git -C ~/ros2_ws/src/unitree_go2_ros2 rev-parse HEAD
git -C ~/ros2_ws/src/GuideDog-Go2 rev-parse HEAD
```

Also include the exact failing command and its console output. Use a GitHub Issue rather than changing multiple dependency versions at once.

## References

- [GuideDog architecture](architecture.md)
- [GuideDog roadmap](roadmap.md)
- [Known limitations](limitations.md)
- [Upstream Go2 simulator README](https://github.com/khaledgabr77/unitree_go2_ros2)
- [ROS 2 Jazzy Ubuntu installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Gazebo / ROS integration installation](https://gazebosim.org/docs/latest/ros_installation/)
