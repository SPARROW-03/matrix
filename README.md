# MATRIX
### ROS 2 Autonomous Number Detection & Navigation

MATRIX is a ROS 2–based autonomous mobile robot built for indoor navigation using computer vision. It continuously processes images from an onboard camera to detect numbered markers (1–6), computes the corresponding goal position for a valid target, and autonomously drives to it.

The project is being developed incrementally, with the current focus on reliable vision-based number detection and closed-loop navigation. Additional autonomous capabilities — localization, path planning, and exploration — are planned as the project matures.

---

## Overview

The robot continuously processes images from its camera to detect numbered targets. Once a valid target (1–6) is identified, the navigation system computes the corresponding goal position and autonomously drives the robot to the selected destination.

Current implementation focuses on reliable vision-based number detection and autonomous movement.

---

## Current Features

- Number detection (1–6) using an onboard camera
- Image processing and target identification with OpenCV
- Autonomous navigation toward the detected target
- Modular, node-based ROS 2 architecture
- Gazebo simulation support for development and testing

---

## Technology Stack

- ROS 2 Humble
- Python
- OpenCV
- Gazebo
- Ubuntu Linux

---

## System Workflow

```
Camera
  │
  ▼
Image Processing
  │
  ▼
Number Detection
  │
  ▼
Target Selection
  │
  ▼
Navigation Node
  │
  ▼
Robot Movement
```

The robot captures a frame, processes it to identify visible numbered markers, selects a target from the valid range (1–6), and hands off to the navigation node, which drives the robot toward the corresponding goal.

---

## Simulation

The repository includes everything needed to run MATRIX in simulation:

- Gazebo world with numbered marker models
- Robot URDF description
- ROS 2 launch files
- Navigation and detection scripts

---

## Repository Status

### Implemented
- Vision-based number detection
- Target identification (1–6)
- Autonomous movement toward the detected target

### Planned
- Goal-pose based navigation
- Localization
- Autonomous path planning
- Autonomous exploration
- Dynamic obstacle avoidance

---

## Project Status

Active development — this repository is continuously updated as new navigation modules and autonomous capabilities are implemented.

---

## Author

**Devaraj V C**
Mechatronics Engineer | Robotics & Autonomous Systems

- LinkedIn: https://linkedin.com/in/vcdevaraj03
