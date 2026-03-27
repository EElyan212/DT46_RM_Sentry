from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # DeclareLaunchArgument(
        #     name='livox', default_value='/livox',
        #     description='Namespace for sample topics'
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            # 必须使用 --frame-id 这种显式标签格式
            arguments=['--x', '0', '--y', '0', '--z', '0.1', 
                       '--yaw', '0', '--pitch', '0', '--roll', '0', 
                       '--frame-id', 'base_link', 
                       '--child-frame-id', 'livox_frame']
        ),
        Node(
            package='pointcloud_to_laserscan', 
            executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/livox/lidar'),
                # 修复点 1：移除 ['/scan'] 这种错误的列表格式，直接用字符串
                ('scan', '/scan'), 
            ],
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'use_reliability_qos': True,   # 很多 Humble 版本认这个
                'qos_reliability': 'best_effort',
                'reliability': 'best_effort',
                'min_height': -1.0,
                'max_height': -1.0,
                'angle_min': -3.14159,  # -M_PI/2
                'angle_max': 3.14159,  # M_PI/2
                'angle_increment': 0.0043,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 0.6,
                'range_max': 5.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])