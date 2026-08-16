cmake_minimum_required(VERSION 3.8)
project(matrix_bot)

# -------------------------
# Find packages
# -------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# -------------------------
# Install resources
# -------------------------
install(
  DIRECTORY
    config
    launch
    meshes
    urdf
    world
    Model
    Map
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------
# Install Python nodes
# -------------------------
install(
  PROGRAMS
    scripts/block_detector.py
    scripts/goal_pose_and_client.py
    scripts/crop_template.py
    scripts/capture.py
    scripts/aruco_detector.py
    scripts/dock_controller.py
    scripts/llm_task_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()