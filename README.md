```markdown
# MATRIX
### ROS 2 Autonomous Navigation, RL-Docking & LLM Task Planning

MATRIX is a ROS 2–based autonomous mobile robot platform designed for intelligent indoor logistics and visual navigation. It integrates computer vision (digit detection & ArUco tracking), Nav2 topological waypoint dispatch, a MuJoCo-trained Reinforcement Learning policy for precision docking, and an LLM-driven natural language task planner with tool-calling and kinematic safety interlocks.

---

**Overview**

1. **LLM Semantic Task Planning (High-Level Autonomy):** Natural language conversational agent with function calling, contextual memory, and state-aware collision safety interlocks.
2. **Reinforcement Learning Docking (Learned Control):** Trained neural policy deployed via ONNX Runtime for closed-loop terminal alignment and precision docking.
3. **Nav2 Autonomous Navigation & Sensor Fusion (Foundational Mobility):** AMCL localization fused with IMU and wheel odometry via EKF, with costmap-based global and local path planning.
4. **Computer Vision & Perception (Vision-Guided Behaviors):** Multi-scale template matching for digit marker detection (1–6) and ArUco tracking for dock acquisition.

---

## Key Features

- **LLM Task Planner:** Natural-language conversational interface supporting OpenAI/Groq endpoints with function/tool calling, contextual memory, and state-aware collision avoidance.
- **RL-Based Precision Docking:** Deep RL policy trained in MuJoCo and executed via ONNX Runtime at 50 Hz on relative $dx, dy, \sin(\theta), \cos(\theta), v, \omega$ observations.
- **Nav2 Autonomous Navigation:** Costmap-based global and local obstacle avoidance driven by LiDAR and topological waypoint targets.
- **Sensor Fusion Localization:** AMCL localization fused with wheel odometry and IMU data using `robot_localization` (EKF).
- **Decoupled Architecture:** Custom ROS 2 Actions and Services enabling external orchestrators, terminal scripts, or LLM agents to command the platform.
- **Vision-Based Target Detection:** Multi-scale template matching to detect and sequence digit markers (1–6) via the onboard camera feed.

---

## Technology Stack

- **ROS 2:** Humble Hawksbill
- **Navigation & Control:** Nav2, `robot_localization` (EKF), custom Diff-Drive controllers
- **Inference & Autonomy:** ONNX Runtime (`onnxruntime`), OpenAI Python API, `python-dotenv`
- **Vision:** OpenCV (`cv2.aruco`, multi-scale template matching), `cv_bridge`
- **Simulation:** Gazebo Classic 11, MuJoCo (training environment)
- **OS & Language:** Ubuntu 22.04 LTS, Python 3.10, C++

---

## Repository Layout

The repository is organized into two ROS 2 packages: the core robot package and the custom interface package.

```text
matrix/
├── .env.example
├── .gitignore
├── LICENSE
│
├── matrix_bot/                         # Core robot package
│   ├── config/                         # Navigation, costmap & EKF parameters
│   ├── launch/                         # Simulation, bringup & navigation launch files
│   ├── Map/                            # 2D occupancy-grid maps
│   ├── meshes/                         # CAD meshes, ArUco markers & digit templates
│   ├── Model/                          # ONNX RL docking policy
│   │
│   ├── scripts/                        # Python autonomy & perception nodes
│   │   ├── llm_task_planner.py         # LLM task planner
│   │   ├── dock_controller.py          # RL docking action server
│   │   ├── aruco_dock_detector.py      # ArUco pose estimation
│   │   ├── block_detector.py           # Visual digit detection
│   │   ├── goal_pose_and_client.py     # Waypoint navigation
│   │   ├── capture.py                  # Camera calibration utility
│   │   └── crop_template.py            # Template extraction utility
│   │
│   ├── urdf/                           # URDF/Xacro robot model & sensor plugins
│   └── world/                          # Gazebo simulation worlds
│
├── matrix_interfaces/                  # Custom ROS 2 interfaces
│   ├── action/
│   │   ├── NavigateToLocation.action
│   │   ├── DockToStation.action
│   │   └── UndockFromStation.action
│   └── srv/
│       └── GetLocation.srv
│
└── README.md
```

### Package Responsibilities

**`matrix_bot`**

→ Core autonomy, navigation, perception, simulation, and control logic  
→ Nav2 navigation and localization  
→ Vision-based target detection  
→ RL-based precision docking  
→ LLM task planning  
→ Gazebo simulation and robot description  

**`matrix_interfaces`**

→ Defines the ROS 2 Actions and Services used to expose high-level robot capabilities  
→ Provides the communication boundary between external planners and the robot autonomy stack  


## Prerequisites & Installation

### 1. System Dependencies

Ensure ROS 2 Humble desktop and essential packages are installed:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop-full \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  python3-pip \
  python3-opencv \
  python3-yaml

```

### 2. Python Dependencies

```bash
pip3 install openai python-dotenv onnxruntime numpy

```

---

## Build

```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/SPARROW-03/matrix.git matrix
cd ~/dev_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

```

---

## LLM Configuration

The natural language planner supports standard OpenAI-compatible API backends (e.g., Groq, OpenAI, Ollama).

1. Copy the template configuration inside the repository root:
```bash
cd ~/dev_ws/src/matrix
cp .env.example .env

```


2. Populate your `.env` with your API credentials:
```env
LLM_BASE_URL=https://api.groq.com/openai/v1
LLM_API_KEY=your_actual_api_key_here
LLM_MODEL=llama-3.3-70b-versatile

```



> **Note:** `.env` is listed in `.gitignore` to prevent committing sensitive keys. The planner defaults to reading `src/matrix/.env`, but a custom path can be passed via ROS 2 parameter:
> `--ros-args -p env_file_path:=/path/to/custom/.env`

---

## Running MATRIX

### 1. Launch Simulation Environment

```bash
ros2 launch matrix_bot sim.launch.py

```

*Spawns MATRIX in Gazebo Classic with the warehouse world, numbered signs, and charging station.*

### 2. Bring Up Navigation & Localization

In a second terminal:

```bash
ros2 launch matrix_bot bringup.launch.py

```

*Initializes RViz, EKF sensor fusion, AMCL map localization, Nav2 lifecycle servers, and waypoint services.*

### 3. Launch RL Docking & Vision Nodes

In a third terminal:

```bash
# Start ArUco dock pose detector
ros2 run matrix_bot aruco_detector.py

```

In a fourth terminal:

```bash
# Start ONNX RL Dock Controller Action Server
ros2 run matrix_bot dock_controller.py

```

---

## Operating Interfaces

MATRIX can be controlled via natural language, autonomous vision sequencing, or direct ROS 2 Actions/Services.

### Option A: Conversational Task Planner (LLM Shell)

```bash
ros2 run matrix_bot llm_task_planner.py

```

Interact directly in the terminal:

```text
MATRIX Autonomy Interface - Natural Language Shell
=======================================================

Operator > Go to location 3, then return to docking station and dock.
  -> [Action Dispatch]: navigate_to_location({'location': '3'})
  -> [Action Dispatch]: navigate_to_location({'location': 'Docking Station'})
  -> [Action Dispatch]: dock_to_station({})

MATRIX > I have navigated to location 3, returned to the docking station approach, and completed the precision docking sequence.

Operator > Now go to location 1
  -> [Action Dispatch]: undock_from_station({})
  -> [Action Dispatch]: navigate_to_location({'location': '1'})

MATRIX > Undocked safely from the station and navigated to location 1.

```

---

### Option B: Autonomous Visual Digit Following

```bash
ros2 run matrix_bot block_detector.py

```

*Sequentially scans and drives toward visual digit markers (1 through 6) using multi-scale template matching.*

---

### Option C: Direct ROS 2 Action & Service CLI

**Send Navigation Goal:**

```bash
ros2 action send_goal /navigate_to_location matrix_interfaces/action/NavigateToLocation "{location: '2'}"

```

**Query Coordinate Pose:**

```bash
ros2 service call /get_location matrix_interfaces/srv/GetLocation "{location: 'Docking Station'}"

```

**Execute Precision RL Docking:**

```bash
ros2 action send_goal /dock_to_station matrix_interfaces/action/DockToStation "{}"

```

**Execute Undock Clearance:**

```bash
ros2 action send_goal /undock_from_station matrix_interfaces/action/UndockFromStation "{}"

```

---

## Public Action & Service APIs

All interfaces are declared in `matrix_interfaces`:

| Interface Name | Type | Description |
| --- | --- | --- |
| `/navigate_to_location` | `action/NavigateToLocation` | Dispatches Nav2 waypoint tracking to a named ID from `location.yaml`. Reports remaining distance feedback. |
| `/dock_to_station` | `action/DockToStation` | Executes visual search, ArUco alignment, and 50 Hz ONNX RL policy terminal docking. |
| `/undock_from_station` | `action/UndockFromStation` | Executes closed-loop odometry reverse clearance to free the robot from dock rails. |
| `/get_location` | `srv/GetLocation` | Returns stored $(x, y, \text{yaw})$ coordinates for a specified topological marker. |

---

## System Architecture

MATRIX follows a layered ROS 2 autonomy architecture. High-level task planning is separated from navigation, perception, localization, and low-level control.

```text
┌──────────────────────────────────────────────────────────────┐
│                    HIGH-LEVEL AUTONOMY                       │
│                                                              │
│              LLM Natural Language Task Planner               │
│       Tool Calling • Context • Safety State Machine          │
└──────────────────────────────┬───────────────────────────────┘
                               │
                               │ ROS 2 Actions / Services
                               ▼
┌──────────────────────────────────────────────────────────────┐
│                  BEHAVIOUR / TASK LAYER                      │
│                                                              │
│   Navigation Action       Docking Action      Undock Action  │
│   /navigate_to_location   /dock_to_station    /undock...     │
└───────────────┬──────────────────────┬───────────────────────┘
                │                      │
                ▼                      ▼
┌──────────────────────────┐  ┌────────────────────────────────┐
│ NAVIGATION & LOCALIZATION│  │       PERCEPTION & DOCKING      │
│                          │  │                                │
│ Nav2                     │  │ Camera                         │
│ AMCL                     │  │      │                         │
│ robot_localization (EKF) │  │      ▼                         │
│ LiDAR Costmaps           │  │ ArUco Pose Estimation          │
│ Waypoint Dispatch        │  │      │                         │
│                          │  │      ▼                         │
│                          │  │ RL Docking Controller           │
│                          │  │      │                         │
│                          │  │      ▼                         │
│                          │  │ ONNX Policy @ 50 Hz             │
└──────────────┬───────────┘  └────────────────┬───────────────┘
               │                               │
               └───────────────┬───────────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────────────┐
│                    ROBOT CONTROL LAYER                       │
│                                                              │
│                 Diff-Drive / Velocity Control                │
│                         /cmd_vel                             │
└──────────────────────────────┬───────────────────────────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────────────┐
│                       ROBOT PLATFORM                         │
│                                                              │
│        URDF/Xacro • Sensors • Gazebo Simulation              │
└──────────────────────────────────────────────────────────────┘
```

### Data Flow

The main autonomy flow is:

```text
Operator
   │
   ▼
Natural Language Command
   │
   ▼
LLM Task Planner
   │
   ├──────────────► Navigation Action
   │                     │
   │                     ▼
   │              Nav2 + AMCL + EKF
   │                     │
   │                     ▼
   │                 /cmd_vel
   │
   ├──────────────► Docking Action
   │                     │
   │                     ▼
   │              ArUco Pose Detection
   │                     │
   │                     ▼
   │              ONNX RL Controller
   │                     │
   │                     ▼
   │                 /cmd_vel
   │
   └──────────────► Undock Action
                         │
                         ▼
                  Odometry-based
                   clearance control
```

### Architecture Principles

→ **Layered autonomy:** High-level task planning is separated from navigation and control.

→ **ROS 2 interfaces:** Actions and Services provide clean interfaces between the planner and robot behaviours.

→ **Independent behaviours:** Navigation, docking, undocking, perception, and task planning operate as separate ROS 2 components.

→ **Sensor-driven autonomy:** Localization and perception use the available LiDAR, odometry, IMU, and camera information.

→ **Learned terminal control:** Precision docking uses an ONNX Runtime reinforcement-learning policy.

→ **Simulation-first validation:** The complete stack can be evaluated in Gazebo before deployment to the physical platform.


## Calibrating Digit Templates

Digit templates (1–6) are stored in `meshes/templates/`. To recalibrate templates for custom environments or fonts:

1. Position the robot directly facing the target marker in Gazebo or real world.
2. Capture the frame and crop the bounding template:
```bash
ros2 run matrix_bot capture.py raw_digit_1.png
ros2 run matrix_bot crop_template.py raw_digit_1.png meshes/templates/digit_1.png

```


3. Re-build with `colcon build --symlink-install` to update share assets.

---

## Author

**Devaraj V C**

Mechatronics Engineer | Robotics & Autonomous Systems

* **LinkedIn:** [linkedin.com/in/vcdevaraj03](https://linkedin.com/in/vcdevaraj03)
* **GitHub:** [github.com/SPARROW-03](https://github.com/SPARROW-03)
```

```
