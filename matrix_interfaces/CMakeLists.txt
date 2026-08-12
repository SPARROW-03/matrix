cmake_minimum_required(VERSION 3.8)
project(matrix_interfaces)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetLocation.srv"
  "action/NavigateToLocation.action"
)

ament_export_dependencies(rosidl_default_runtime)


ament_package()
