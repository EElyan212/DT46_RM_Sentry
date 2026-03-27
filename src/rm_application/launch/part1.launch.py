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
    ])