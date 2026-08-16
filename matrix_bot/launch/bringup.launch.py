from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('matrix_bot')

    rviz_visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', 'rsp.launch.py'))
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', 'navigation.launch.py'))
    )

    Location_navigation_node = Node(
        package='matrix_bot',
        executable='goal_pose_and_client.py',
        name='goal_pose_and_client',
        output='screen',
    )

    dock_controller_node=Node(
        package='matrix_bot',
        executable='dock_controller.py',
        name='dock_controller',
        output='screen'
    )

    tag_node=Node(
        package='matrix_bot',
        executable='aruco_detector.py',
        name='aruco_tag',
        output='screen'
    )

    return LaunchDescription([
        rviz_visualization,
        TimerAction(period=5.0, actions=[navigation]), 
        TimerAction(period=10.0, actions=[Location_navigation_node]),
        TimerAction(period=10.0, actions=[dock_controller_node]),
        TimerAction(period=10.0,actions=[tag_node])
    ])