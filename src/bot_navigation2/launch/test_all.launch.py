from launch import LaunchDescription
import os
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 路径配置
    ydlidar_pkg_share = FindPackageShare(package='ydlidar_ros2_driver').find('ydlidar_ros2_driver')
    ydlidar_launch_file_path = os.path.join(ydlidar_pkg_share, 'launch', 'x3_ydlidar_launch.py')
    
    ydlidar2_launch_file_path = os.path.join(ydlidar_pkg_share, 'launch', 'two_ydlidar_launch.py')
    
    map_tools_pkg_share = FindPackageShare(package='map_tools').find('map_tools')``
    merged_scan_launch_file_path = os.path.join(map_tools_pkg_share, 'launch', 'merged_scan.launch.py')
    tf_static_launch_file_path = os.path.join(map_tools_pkg_share, 'launch', 'tf_static_launch.py')
    amcl_launch_file_path = os.path.join(map_tools_pkg_share, 'launch', 'amcl_launch.py')
    
    odometry_pkg_share = FindPackageShare(package='rf2o_laser_odometry').find('rf2o_laser_odometry')
    odometry_launch_file_path = os.path.join(odometry_pkg_share, 'launch', 'rf2o_laser_odometry.launch.py')
    
    # turtlebot_pkg_share = FindPackageShare(package='bot_navigation2').find('bot_navigation2')
    # navigation_launch_file_path = os.path.join(turtlebot_pkg_share, 'launch', 'navigation2.launch.py')

    # 创建包含描述
    ydlidar_launch_file = IncludeLaunchDescription(PythonLaunchDescriptionSource(ydlidar_launch_file_path))
    ydlidar2_launch_file = IncludeLaunchDescription(PythonLaunchDescriptionSource(ydlidar2_launch_file_path))
    merged_scan_file = IncludeLaunchDescription(PythonLaunchDescriptionSource(merged_scan_launch_file_path))
    tf_static_file = IncludeLaunchDescription(PythonLaunchDescriptionSource(tf_static_launch_file_path))
    amcl_file = IncludeLaunchDescription(PythonLaunchDescriptionSource(amcl_launch_file_path))
    odometry_file = IncludeLaunchDescription(PythonLaunchDescriptionSource(odometry_launch_file_path))#記得註釋
    # navigation_file = IncludeLaunchDescription(PythonLaunchDescriptionSource(navigation_launch_file_path))

    # navigation_file = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(navigation_launch_file_path),
    #     launch_arguments={
    #         'use_sim_time': 'False',
    #         'slam': 'False',
    #         'map': '/home/morefine/rm2025_hzu_sentry_ws/src/map_tools/maps/slam_map.yaml',
    #         # 'controller_frequency': '20.0',
    #         # 'planner_frequency': '5.0',
    #         # 'max_vel_x': '0.5',
    #         # 'min_vel_x': '0.1',
    #     }.items()
    # )


    # 创建LaunchDescription对象
    ld = LaunchDescription()
    ld.add_action(ydlidar_launch_file)
    ld.add_action(ydlidar2_launch_file)
    ld.add_action(merged_scan_file)
    ld.add_action(tf_static_file)
    ld.add_action(amcl_file)
    # ld.add_action(odometry_file)
    # ld.add_action(navigation_file)

    return ld
