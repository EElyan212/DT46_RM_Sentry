import os
import launch
import launch_ros
from launch_ros.actions import Node  # 添加这一行
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource,AnyLaunchDescriptionSource


def generate_launch_description():
    #driver_path
    ladir_driver = get_package_share_directory('livox_ros_driver2')
    #msg_to_pcd2
    msg_to_pcd2 = get_package_share_directory('livox_to_pointcloud2')
    #pcd2_to_scan
    pcd2_to_scan = get_package_share_directory('pointcloud_to_laserscan')
    #point_lio
    point_lio = get_package_share_directory('point_lio')

    # 获取与拼接默认路径
    rm_navigation2_dir = get_package_share_directory(
        'rm_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='false')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(rm_navigation2_dir, 'maps', 'room.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(rm_navigation2_dir, 'config', 'nav2_params.yaml'))


    # static_map_to_instance = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='map_to_camera_init_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    # )
    

    return launch.LaunchDescription([
        # static_map_to_instance,
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ladir_driver, '/launch_ROS2', '/msg_MID360_launch.py']),
        ),
        launch.actions.IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                [msg_to_pcd2, '/launch', '/livox_to_pointcloud2.launch.py']),
        ),
        launch.actions.IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                [pcd2_to_scan, '/launch', '/pointcloud_to_laserscan_launch.py']),
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [point_lio, '/launch', '/mapping_mid360.launch.py']),
        ),
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        launch_ros.actions.Node(
            package='rm_navigation2',
            executable='map_clear',
            name='map_clear_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])