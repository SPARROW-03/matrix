from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('matrix_bot')
    
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'localization.launch.py')
        )
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'navigation.launch.py')
        )
    )

    rviz_visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'rsp.launch.py')
        )
    )

    return LaunchDescription([
        rviz_visualization,
        TimerAction(period=5.0, actions=[localization]),
        TimerAction(period=10.0, actions=[navigation])
    ])