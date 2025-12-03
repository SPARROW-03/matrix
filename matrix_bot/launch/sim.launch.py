import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    pkg_path=os.path.join(get_package_share_directory('matrix_bot'))
    controller_yaml=os.path.join(pkg_path,'config','controller.yaml')
    
    
    urdf_file = os.path.join(pkg_path, 'urdf', 'Matrix_bot.urdf')
    robot_description=xacro.process_file(urdf_file).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')]),
        launch_arguments=[('use_sim_time','true'),('world','empty_world')]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','robot_description',
                   '-entity','matrix',
                   '-x','0.0','-y','0.0','-z','0.05'],
        output='screen'
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description':robot_description}, controller_yaml],
        output='both'
    )

    Joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    
    return LaunchDescription([
        gazebo,
        spawn_entity,
        controller_manager,
        Joint_state_broadcaster_spawner,
        diff_drive_spawner
    ])