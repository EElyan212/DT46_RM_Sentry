import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, PushRosNamespace
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter import ParameterFile

def generate_launch_description():
    # Declare launch arguments for topic remapping
    odom_topic = LaunchConfiguration('odom_topic', default='odom')
    vel_topic = LaunchConfiguration('vel_topic', default='cmd_vel')

    # Declare parameters
    map_param_dir = os.path.join(
        get_package_share_directory('bot_navigation2'), 'param')

    # Node configuration and parameter files
    move_base_params = [
        ParameterFile(os.path.join(map_param_dir, 'costmap_common_params.yaml'), namespace='global_costmap'),
        ParameterFile(os.path.join(map_param_dir, 'costmap_common_params.yaml'), namespace='local_costmap'),
        ParameterFile(os.path.join(map_param_dir, 'local_costmap_params.yaml')),
        ParameterFile(os.path.join(map_param_dir, 'global_costmap_params.yaml')),
        ParameterFile(os.path.join(map_param_dir, 'dwa_local_planner_params.yaml')),
        ParameterFile(os.path.join(map_param_dir, 'move_base_params.yaml')),
        ParameterFile(os.path.join(map_param_dir, 'global_planner_params.yaml')),
    ]

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('odom_topic', default_value='odom', description='Odometry topic'),
        DeclareLaunchArgument('vel_topic', default_value='cmd_vel', description='Velocity topic'),

        # Include node with parameters and remapping
        Node(
            package='move_base',
            executable='move_base',
            name='move_base_node',
            output='screen',
            respawn=False,
            parameters=move_base_params,  # Add parameter files here
            remappings=[
                ('/odom', odom_topic),
                ('/cmd_vel', vel_topic)
            ]
        ),
    ])
