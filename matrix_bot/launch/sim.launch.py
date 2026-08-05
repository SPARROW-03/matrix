import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('matrix_bot'))
    urdf_file = os.path.join(pkg_path, 'urdf', 'Matrix_bot.urdf')
    world = os.path.join(pkg_path, 'world', 'matrix.world')
    # Process XACRO/URDF file
    robot_description_xml = xacro.process_file(urdf_file).toxml()

    # 1. Gazebo Launch (Stripped down to default empty world)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments=[('use_sim_time', 'true'), ('world', world)]  # Added the ('world', world) tuple
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True, 
            'robot_description': robot_description_xml
        }]
    )

    # 3. Spawn Entity (Spawns your bot directly at the center of the empty world)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'matrix_bot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.0'],
        output='screen'
    )

    # 4. Spawners for your controllers
    joint_state_broadcaster_spawner = Node(
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
        robot_state_publisher,
        spawn_entity,
        #joint_state_broadcaster_spawner,
        #diff_drive_spawner
    ])