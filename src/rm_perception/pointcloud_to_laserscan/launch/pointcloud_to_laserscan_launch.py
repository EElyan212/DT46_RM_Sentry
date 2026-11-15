from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(initial_entities=[
        DeclareLaunchArgument(
            name='livox', default_value='livox',
            description='Namespace for sample topics'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_odom_to_livox_odom',
            arguments=[
                '0.0', '0.178', '0.352',
                '0.0', '0.0', '0.0',
                'odom', 'livox_odom'
            ],
            output='screen'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            # remappings=[('cloud_in', [LaunchConfiguration(variable_name='livox'), '/lidar/pcd2']),
            #             ('scan',['/scan'])],
            remappings=[('cloud_in', '/cloud_registered_body'),
                        ('scan',['/scan'])],
            parameters=[{
                'target_frame': 'base_footprint',

                'transform_tolerance': 0.1,
                'min_height': 0.1,

                'max_height': 0.2,
                'angle_min': -3.14159,  # -M_PI/2
                'angle_max': 3.14159,  # M_PI/2
                'angle_increment': 0.0035,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 0.25,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'scan_qos_reliability': 'best_effort'  # 对应源码新加入的配置参数
            }],
            name='pointcloud_to_laserscan'
        )
    ])
