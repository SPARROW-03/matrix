# MATRIX
### ROS 2 Autonomous Number Detection & Navigation

MATRIX is a ROS 2–based autonomous mobile robot built for indoor navigation using computer vision. It continuously processes images from an onboard camera to detect numbered markers (1–6), computes the corresponding goal position for a valid target, and autonomously drives to it.

The project is being developed incrementally, with the current focus on reliable vision-based number detection and closed-loop navigation. Additional autonomous capabilities — exploration and higher-level task planning — are planned as the project matures.

---

## Overview

The robot continuously processes images from its camera to detect numbered targets. Once a valid target (1–6) is identified, the navigation system computes the corresponding goal position and autonomously drives the robot to the selected destination. Locations can also be requested directly through a ROS 2 service/action interface, independent of vision.

---

## Current Features

- Number detection (1–6) using an onboard camera, via multi-scale template matching
- Autonomous navigation toward a detected or requested target using Nav2
- AMCL-based localization against a pre-built map
- Location lookup and goal-dispatch via custom ROS 2 service and action interfaces
- Modular, node-based ROS 2 architecture
- Gazebo simulation support for development and testing

---

## Technology Stack

- ROS 2 Humble
- Nav2 (map_server, AMCL, controller/planner/behavior servers, BT navigator)
- Python 3, OpenCV, cv_bridge
- Gazebo Classic
- Ubuntu 22.04

---

## Repository Layout

```
matrix_bot/           # Robot description, simulation, navigation, and vision nodes
├── config/            # Nav2 params, AMCL/controller config, known locations
├── launch/            # sim / rsp / navigation / bringup launch files
├── Map/               # Pre-built occupancy grid map for localization
├── meshes/            # Visual meshes and digit/ArUco templates
├── scripts/           # Python nodes (vision, capture/calibration tools, navigation)
├── urdf/              # Robot URDF/xacro description
└── world/             # Gazebo world with numbered marker models

matrix_interfaces/     # Custom srv/action definitions used by the navigation node
```

---

## Prerequisites

- Ubuntu 22.04 + ROS 2 Humble (desktop-full recommended)
- Gazebo Classic (installed with `ros-humble-desktop-full` or `gazebo_ros_pkgs`)
- Nav2: `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`
- Python packages: `python3-opencv`, `python3-yaml` (usually already present with `desktop-full`, but confirm before building)

---

## Build

```bash
mkdir -p ~/matrix_ws/src
cd ~/matrix_ws/src
git clone https://github.com/SPARROW-03/Matrix.git
cd ~/matrix_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## Running MATRIX

Simulation and navigation are launched separately by design — this keeps sim-only debugging simple, and means `bringup.launch.py` works unmodified on real hardware once the robot's own drivers are publishing odometry, TF, and camera data in place of Gazebo.

### 1. Start the simulation

```bash
ros2 launch matrix_bot sim.launch.py
```

This starts Gazebo with the MATRIX world, spawns the robot, and starts `robot_state_publisher`. Confirm the robot has actually spawned (check Gazebo, or `ros2 topic echo /odom`) before moving on — Gazebo can occasionally still be loading the world when the spawn is requested.

*(Skip this step entirely when running on real hardware — just make sure your robot's driver stack is publishing `/odom`, TF, and camera topics equivalent to what the sim provides.)*

### 2. Bring up localization, navigation, and vision

In a second terminal:

```bash
ros2 launch matrix_bot bringup.launch.py
```

This starts:
- RViz + `joint_state_publisher` (via `rsp.launch.py`)
- `map_server`, `amcl`, and the full Nav2 stack under a single lifecycle manager (via `navigation.launch.py`) wait for about 15 sec.
- the `goal_pose_and_client` node, exposing location lookup/navigation as ROS 2 service and action interfaces


### 3. Run vision-based marker following (optional)

```bash
ros2 run matrix_bot block_detector.py
```

This drives the robot to search for and approach digit signs 1 through 6 in sequence using the camera feed.

### 4. Request a known location directly (optional, no vision needed)

```bash
ros2 service call /get_location matrix_interfaces/srv/GetLocation "{location: '1'}"
```

or trigger navigation directly via the action interface:

```bash
ros2 action send_goal /navigate_to_location matrix_interfaces/action/NavigateToLocation "{location: '1'}"
```

Known locations are defined in `matrix_bot/config/location.yaml`.

---

## Calibrating New Digit Templates

Template images for digits 1–6 already ship in `meshes/templates/`. To recapture them (e.g. after changing the marker font or lighting in the world):

```bash
ros2 run matrix_bot capture.py raw_digit_1.png     # while facing the sign head-on
ros2 run matrix_bot crop_template.py raw_digit_1.png meshes/templates/digit_1.png
```

Repeat per digit, then rebuild so the updated templates are installed to the share directory.

---

## System Workflow

```
Camera
  │
  ▼
Image Processing (OpenCV template matching)
  │
  ▼
Number Detection (1–6)
  │
  ▼
Target Selection
  │
  ▼
Nav2 Goal Dispatch
  │
  ▼
Robot Movement
```

---

## Repository Status

### Implemented
- Vision-based number detection (multi-scale template matching)
- Autonomous movement toward a detected target
- AMCL-based localization
- Goal-pose based navigation via Nav2
- Custom service/action interfaces for direct location requests
- Costmap-based obstacle avoidance via Nav2's global/local costmaps and lidar

### Planned
- Autonomous exploration
- Higher-level task planning / LLM-based decision making

---

## Project Status

Active development — this repository is continuously updated as new navigation modules and autonomous capabilities are implemented.

---

## Author

**Devaraj V C**
Mechatronics Engineer | Robotics & Autonomous Systems

- LinkedIn: https://linkedin.com/in/vcdevaraj03