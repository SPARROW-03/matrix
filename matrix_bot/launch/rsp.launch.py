import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get package path and URDF file path
    pkg_path = get_package_share_directory('matrix_bot')
    rviz_config_file = os.path.join(pkg_path, 'config', 'rviz.rviz')
    ekf_config_path = os.path.join(pkg_path, 'config', 'ekf.yaml')
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path,{'use_sim_time': use_sim_time}
            ]
        )
    # NEW: Static Transform Publisher to bridge 'odom' to your robot's root frame
    # Arguments: x y z yaw pitch roll parent_frame child_frame
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    static_tf_publisher_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        rviz,
        joint_state_publisher,
        ekf_node,
        #static_tf_publisher,     for the static transform from 'map' to 'odom', you can uncomment this if needed
        #static_tf_publisher_odom    for the static transform from 'odom' to 'base_link', you can uncomment this if needed
    ])