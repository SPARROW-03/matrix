import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = get_package_share_directory('matrix_bot')

    map_file = os.path.join(
        pkg_path,
        'Map',
        'Matrix_world.yaml'
    )

    params_file = os.path.join(
        pkg_path,
        'config',
        'nav2_params.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {
                'yaml_filename': map_file,
                'use_sim_time': use_sim_time
            }
        ]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time
            }
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl'
                ]
            }
        ]
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time'
        ),

        map_server,
        amcl,
        lifecycle_manager
    ])