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

        # ================== 静态 TF 广播 ==================
        # 参数顺序: x, y, z, yaw, pitch, roll, 父, 子
        #
        # livox_frame → base_footprint（URDF 反向）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_livox_to_base',
            arguments=[
                '0.0', '0.37', '-0.15',
                '0.0', '0.0', '1.571',
                'livox_frame', 'base_footprint'
            ],
            output='screen'
        ),
        # odom → livox_odom（只平移，旋转已由 Point-LIO IMU 初始化处理）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_odom_to_livox_odom',
            arguments=[
                '0.0', '0.15', '0.37',
                '0.0', '0.0', '0.0',
                'odom', 'livox_odom'
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
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.1,
                'min_height': 0.06,
                'max_height': 0.25,
                'angle_min': -3.14159,  # -M_PI/2
                'angle_max': 3.14159,  # M_PI/2
                'angle_increment': 0.0043,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.5,
                'range_max': 5.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'scan_qos_reliability': 'reliable'  # 对应源码新加入的配置参数
            }],
            name='pointcloud_to_laserscan'
        )
    ])
