from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='livox', default_value='livox',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            # remappings=[('cloud_in', [LaunchConfiguration(variable_name='livox'), '/lidar']),
            #             ('scan',['/scan'])],
            remappings=[('cloud_in', '/cloud_registered_body'),
            ('scan',['/scan'])],
            parameters=[{
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.01,
                'min_height': -0.50,
                'max_height': 0.1,
                'angle_min': -3.14159,  # -M_PI/2
                'angle_max': 3.14159,  # M_PI/2
                'angle_increment': 0.0043,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.0,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
