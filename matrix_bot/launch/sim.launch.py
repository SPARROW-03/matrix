import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_path=os.path.join(get_package_share_directory('matrix_bot'))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')]),
        launch_arguments=[('use_sim_time','true'),('world','empty_world')]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','robot_description',
                   '-entity','matrix',
                   '-x','0.0','-y','0.0','-z','0'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        spawn_entity
    ])