# UR5e Cube Detection and Pointing - AIS2105

ROS2 (Jazzy) system for a UR5e robot arm that uses a wrist-mounted USB camera to detect red, green and blue cubes on a table and point at each in sequence.

---

## System overview

```
USB camera → camera_node → detection_node → transform_node → coordinator_node → motion_node → UR5e
```

| Package | Node | Responsibility |
|---|---|---|
| `bringup_pkg` | launch files, `publish_table` | Starts the full system; publishes table collision object to MoveIt2 |
| `camera_pkg` | `camera_node` | Opens USB camera, publishes raw images and CameraInfo |
| `camera_pkg` | `transform_node` | Converts pixel detections to 3D positions via ray-plane intersection |
| `detection_pkg` | `detection_node` | HSV colour detection, blob detection, publishes 2D centroids |
| `robot_pkg` | `coordinator_node` | State machine orchestrating the full task sequence |
| `robot_pkg` | `motion_node` | IK solving via MoveIt2 `/compute_ik`, trajectory execution via `FollowJointTrajectory` action |

---

## Hardware

- UR5e robot arm
- Logitech C930e (or compatible) USB camera mounted on the tool flange
- Surface board running `ur_robot_driver` and MoveIt2
- This PC running camera, detection and coordination

---

## Prerequisites

**Both machines:**
```bash
sudo apt install ros-jazzy-ur-robot-driver ros-jazzy-ur-moveit-config
```

**This PC only:**
```bash
sudo apt install ros-jazzy-cv-bridge ros-jazzy-image-transport python3-opencv
```

**CycloneDDS** — edit `~/cyclone_dds.xml` and set the network interface:
- `lo` — simulation / single-PC only
- `wlp0s20f3` — WiFi to surface board
- `enx782d7e140822` — USB-Ethernet direct to robot

Both machines must use the same `ROS_DOMAIN_ID` (set in `~/.bashrc`):
```bash
export ROS_DOMAIN_ID=0
```

---

## Build

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Rebuild a single package after changes:
```bash
colcon build --packages-select detection_pkg && source install/setup.bash
```

---

## Running

### Split setup - surface board + this PC

**Surface board - Terminal 1** (robot driver + MoveIt2)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch bringup_pkg ur_moveit_with_table.launch.py robot_ip:=143.25.151.38
```
Wait until MoveIt2 reports ready and joint states are publishing.

**This PC - Terminal 1** (camera + detection + motion + coordinator)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch bringup_pkg full_system.launch.py
```

**This PC - Terminal 2** (trigger once everything is up)
```bash
source ~/ros2_ws/install/setup.bash
ros2 service call /robot/start std_srvs/srv/Trigger {}
```

### Simulation - no robot needed

**Terminal 1**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch bringup_pkg ur_sim.launch.py
```

**Terminal 2**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch bringup_pkg full_system.launch.py
```

---

## Launch arguments

All arguments are optional - defaults are shown.

```bash
ros2 launch bringup_pkg full_system.launch.py \
  table_z:=0.047 \          # table surface height in base_link (metres)
  cube_height:=0.10 \       # cube height (metres) - ray intersects at table_z + cube_height
  approach_height:=0.05 \   # distance above cube top to stop at (metres)
  camera_x:=0.01 \          # camera offset from tool0 along X (metres)
  camera_y:=0.01 \          # camera offset from tool0 along Y (metres)
  camera_z:=0.085           # camera offset from tool0 along Z (metres)
```

---

## Task sequence

```
IDLE → MOVING_HOME → MOVING_OVERVIEW → WAITING_TF
     → WAITING_RED  → HOMING_BEFORE_RED  → MOVING_RED  → HOMING_AFTER_RED
     → WAITING_GREEN → HOMING_BEFORE_GREEN → MOVING_GREEN → HOMING_AFTER_GREEN
     → WAITING_BLUE  → HOMING_BEFORE_BLUE  → MOVING_BLUE  → HOMING_AFTER_BLUE
     → DONE
```

If a cube is not detected within `detection_timeout` (5s), the robot searches up to 5 positions. Each search position dwells for `search_timeout` (5 s) with detection open. If the cube is still not found after all positions, the system enters `ALERT` and stops.

---

## Colour tuning (camera only, no robot needed)

```bash
# Terminal 1 - camera
ros2 run camera_pkg camera_node

# Terminal 2 - detection
ros2 run detection_pkg detection_node --ros-args \
  --params-file ~/ros2_ws/src/ur_project/detection_pkg/config/detection_params.yaml

# Terminal 3 - viewer
QT_QPA_PLATFORM=xcb ros2 run rqt_image_view rqt_image_view
# Select /vision/debug_image from the dropdown

# Terminal 4 - check detections
ros2 topic echo /vision/detections
```

Edit `detection_pkg/config/detection_params.yaml` to tune HSV ranges, then restart Terminal 2. No rebuild needed.

---

## Configuration files

| File | What to tune |
|---|---|
| `detection_pkg/config/detection_params.yaml` | HSV colour ranges, min blob area |
| `robot_pkg/config/robot_params.yaml` | Joint positions (home, overview, search), joint limits, approach height, IK settings |
| `camera_pkg/config/camera_params.yaml` | Camera device, resolution, frame rate |
| `camera_pkg/config/calibration.yaml` | Camera intrinsics (from `camera_calibrator`) |
| `~/cyclone_dds.xml` | DDS network interface |

---

## Key topics

| Topic | Type | Description |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Raw camera frames |
| `/vision/debug_image` | `sensor_msgs/Image` | Annotated image with detection overlays |
| `/vision/detections` | `std_msgs/String` | JSON summary of all current detections |
| `/vision/{color}_position` | `geometry_msgs/PointStamped` | 2D pixel centroid (x=col, y=row) |
| `/vision/{color}_position_3d` | `geometry_msgs/PointStamped` | 3D position in `base_link` frame |
| `/robot/start` | `std_srvs/Trigger` (service) | Starts the task sequence |

---

## Diagnostics

```bash
# Check all nodes are running
ros2 node list

# Verify 3D positions are being published
ros2 topic echo /vision/red_position_3d

# Check TF chain is complete (base_link → tool0 → camera_link → camera_optical_link)
ros2 run tf2_tools view_frames

# Check table collision object is in the planning scene
ros2 topic echo /planning_scene --once

# Check approach_height parameter at runtime
ros2 param get /motion_node approach_height
```

---

## Authors

Mathias Kulbotten, Richard Solheim, Bjørn Stabben, Henrik Utgård
