import os
import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # 1. 初始化 LaunchDescription 对象
    ld = LaunchDescription()

    # 2. 获取基础路径
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio.rviz')

    # 3. 核心修复：合并并注入环境变量
    # 使用 os.environ.copy() 确保 ROS 2 的 LD_LIBRARY_PATH (如 nav_msgs) 不会丢失
    fixed_env = os.environ.copy()
    fixed_env['LD_PRELOAD'] = '/lib/x86_64-linux-gnu/libusb-1.0.so.0'
    fixed_env['PYTHONUNBUFFERED'] = '1'

    # 4. 定义 Launch 配置项
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    # 5. 定义参数声明指令
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    # 6. 调试指令：启动前打印 ldd 信息 (确认 libusb 路径)
    ldd_check_cmd = ExecuteProcess(
        cmd=['ldd', os.path.join(package_path, '../../lib/fast_lio/fastlio_mapping')],
        output='screen'
    )

    # 7. 定义 Fast-LIO 算法节点
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        # 关键：传入合并后的环境变量，解决 symbol lookup error 和 library missing
        env=fixed_env,
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # 8. 定义 RViz2 可视化节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    # 9. 将所有 Action 添加到描述符中
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    
    ld.add_action(ldd_check_cmd) # 调试输出
    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld