cmake_minimum_required(VERSION 3.8)
project(matrix_bot)

find_package(ament_cmake REQUIRED)


# install files and resources
install(
	DIRECTORY config launch meshes urdf world
	DESTINATION share/${PROJECT_NAME}
)



# Install python scripts
install(
	PROGRAMS
	    scripts/block_detector.py
		scripts/navigation.py
		scripts/point_navigation.py
	DESTINATION lib/${PROJECT_NAME}	
)


ament_package()