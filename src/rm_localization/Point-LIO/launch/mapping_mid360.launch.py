from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 声明 RViz 参数
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='True',
        description='Flag to launch RViz.')

    # 获取 YAML 配置文件路径
    config_path = PathJoinSubstitution([
        FindPackageShare('point_lio'),
        'config', 'mid360.yaml'
    ])

    # 节点参数：回归原生 Point-LIO 逻辑
    laser_mapping_params = [
        config_path,
        {
            # --- 回归官方默认设置 ---
            'use_imu_as_input': True,          
            'prop_at_freq_of_imu': True,       
            'publish.tf_send_en': True,        # 必须由节点发送 TF
            'publish.path_en': True,
            
            # --- 移除 common.map_frame 和 common.body_frame 覆盖 ---
            # 让代码使用 .cpp 里默认的 camera_init 和 body
        }
    ]

    # Point-LIO 主节点定义
    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
        # --- 移除 remappings ---
        # 此时先不强制映射为 /odom，先看原生话题 /aft_mapped_to_init 是否稳定
        env={
            **os.environ,  
            "LD_PRELOAD": "/usr/lib/x86_64-linux-gnu/libusb-1.0.so.0"
        }
    )

    # RViz 节点定义（使用你恢复后的官方原始 rviz 配置文件）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg', 'loam_livox.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )

    # 组装 Launch 描述
    ld = LaunchDescription([
        rviz_arg,
        laser_mapping_node,
        GroupAction(
            actions=[rviz_node],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])

    return ld