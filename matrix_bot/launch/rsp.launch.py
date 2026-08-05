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
    urdf_file = os.path.join(pkg_path, 'urdf', 'Matrix_bot.urdf')
    rviz_config_file = os.path.join(pkg_path, 'config', 'rviz.rviz')

    # Process URDF file with xacro
    robot_description = xacro.process_file(urdf_file).toxml()

    # Robot state publisher - pass URDF CONTENT, not path
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
    )
    
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
        node_robot_state_publisher,
        rviz,
        joint_state_publisher,
        static_tf_publisher ,  # Added here
        static_tf_publisher_odom  # Added here
    ])