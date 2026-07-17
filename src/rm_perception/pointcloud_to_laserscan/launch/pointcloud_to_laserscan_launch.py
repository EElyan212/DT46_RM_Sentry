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

        # ================== 新增的静态 TF 广播节点 ==================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_livox_frame',
            # 参数顺序: x, y, z, roll, pitch, yaw, 父坐标系, 子坐标系
            arguments=[
                '0.0', '0.0', '0.0',
                '0.0', '0.0', '0.0',
                'base_footprint', 'livox_frame'
            ],
            output='screen'
        ),
        # =========================================================

        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            # remappings=[('cloud_in', [LaunchConfiguration(variable_name='livox'), '/lidar/pcd2']),
            #             ('scan',['/scan'])],
            remappings=[('cloud_in', '/cloud_registered_body'),
                        ('scan',['/scan'])],
            parameters=[{
                'target_frame': 'livox_frame',
                'transform_tolerance': 0.01,
                'min_height': -0.50,
                'max_height': 0.1,
                'angle_min': -3.14159,  # -M_PI/2
                'angle_max': 3.14159,  # M_PI/2
                'angle_increment': 0.0043,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.30,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'scan_qos_reliability': 'reliable'  # 对应源码新加入的配置参数
            }],
            name='pointcloud_to_laserscan'
        )
    ])
