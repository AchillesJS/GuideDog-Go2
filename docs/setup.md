# Setup — local simulation baseline

**Current verified platform:** Ubuntu 24.04.5 LTS ARM64/aarch64 in UTM on an Apple Silicon Mac; ROS 2 Jazzy; Gazebo Harmonic through `ros-jazzy-ros-gz`. These steps describe the tested setup, not a portable automated installer.

## Prerequisites

Install ROS 2 Jazzy and Gazebo Harmonic using the official distribution instructions for your platform, plus a working colcon environment. Verify:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
gz sim --version
```

The project currently uses a third-party simulator:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/khaledgabr77/unitree_go2_ros2.git
```

Follow that simulator's own README to resolve dependencies, then build and source the workspace. Simulator dependency packages observed locally include `champ`, `champ_base`, `champ_msgs`, `unitree_application`, `unitree_go2_description`, `unitree_go2_sim`. The tested upstream commit has **not yet been pinned**; before making a reproducibility claim record `git -C ~/ros2_ws/src/unitree_go2_ros2 rev-parse HEAD`.

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Launch the simulated robot

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

**Why these flags?** The tested baseline deliberately disables the extra D455 and Velodyne models. The simulated RGB camera and L1-like 4D LiDAR are enabled. This is not a guarantee that the simulated sensor characteristics match physical Go2 hardware.

For keyboard teleoperation, run in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Bring the GuideDog code into the workspace

**Do this only after the `guidedog_perception` directory has been committed to the GitHub repository.** Do not clone over an existing local package.

```bash
cd ~/ros2_ws/src
git clone https://github.com/AchillesJS/GuideDog-Go2.git
cd ~/ros2_ws
colcon build --packages-select guidedog_perception --symlink-install
source install/setup.bash
ros2 run guidedog_perception rgb_node
```

If you already developed `~/ros2_ws/src/guidedog_perception`, move that directory into the cloned `GuideDog-Go2/` and delete the old location **only after checking the move succeeded**; never keep two copies of a ROS 2 package with the same name in `src`.

## Sensor checks

```bash
ros2 topic info /rgb_image
ros2 topic echo /rgb_image --once --field header
ros2 topic info /unitree_lidar/points
gz topic -f -t /rgb_image
gz topic -f -t /unitree_lidar/points
```

Local observations: Gazebo generated approximately 9–9.5 Hz for camera and LiDAR. Receiving via ROS 2 was slower (roughly 2–3 Hz for RGB and 3–4.5 Hz for LiDAR). The bottleneck is **not yet identified**; low reported rates are not acceptable evidence of safety.

## Troubleshooting

- No topic? Check the Gazebo source topic and verify bridge nodes are running.
- RGB topic visible but no callbacks? Check `ros2 topic info -v /rgb_image`, subscriber QoS, bridge logs, and sensor publishing.
- Do not change bridge direction or sensor update rates as a performance workaround without controlled before/after measurements.
- A one-time PNG capture via OpenCV is a useful sanity check; it does not validate object detection or autonomy.
