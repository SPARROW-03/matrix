# matrix_bot/launch/navigation.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_path = get_package_share_directory('matrix_bot')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')

    # The manager configures/activates them in this order, waiting for each
    # acknowledgment before moving to the next.
    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={'use_sim_time': use_sim_time, 'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[configured_params, {'yaml_filename': map_file}],
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl',
        output='screen', parameters=[configured_params],
    )
    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        output='screen', parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )
    smoother_server = Node(
        package='nav2_smoother', executable='smoother_server', name='smoother_server',
        output='screen', parameters=[configured_params],
    )
    planner_server = Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen', parameters=[configured_params],
    )
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server', name='behavior_server',
        output='screen', parameters=[configured_params],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
        output='screen', parameters=[configured_params],
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
        output='screen', parameters=[configured_params],
    )
    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
        output='screen', parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_path, 'config', 'nav2_params.yaml'),  # <-- your file, not nav2_bringup's
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_path, 'Map', 'Matrix_world.yaml'),
        ),
        map_server, amcl, controller_server, smoother_server,
        planner_server, behavior_server, bt_navigator,
        waypoint_follower, velocity_smoother,
        lifecycle_manager,
    ])