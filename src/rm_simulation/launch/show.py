import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ----------------------------
    # 路径配置
    # ----------------------------
    pkg_rm = get_package_share_directory("rm_simulation")
    default_xacro_path = os.path.join(pkg_rm, "urdf", "robot.xacro")

    # Gazebo 空 world
    gazebo_world = os.path.join(
        get_package_share_directory("gazebo_ros"), "worlds", "empty.world"
    )

    # ----------------------------
    # 声明参数
    # ----------------------------
    declare_robot_model = DeclareLaunchArgument(
        name="robot_model",
        default_value=default_xacro_path,
        description="Path to robot xacro file",
    )

    # ----------------------------
    # robot_state_publisher
    # ----------------------------
    robot_description_command = Command(["xacro ", LaunchConfiguration("robot_model")])
    robot_description = {"robot_description": robot_description_command}

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ----------------------------
    # Gazebo 启动
    # ----------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={"world": gazebo_world, "verbose": "true"}.items(),
    )

    # ----------------------------
    # Spawn robot
    # ----------------------------
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description", "-entity", "rm_sentry"],
        output="screen",
    )
    # 延迟 2 秒确保 Gazebo 启动完成
    delayed_spawn = TimerAction(period=2.0, actions=[spawn_entity])

    # ----------------------------
    # RViz
    # ----------------------------
    # rviz_config_file = os.path.join(pkg_rm, "config", "rviz", "mid360.rviz")
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=["-d", rviz_config_file],
    #     output="screen"
    # )

    # ----------------------------
    # LaunchDescription
    # ----------------------------
    return LaunchDescription(
        [
            declare_robot_model,
            rsp_node,
            gazebo_launch,
            delayed_spawn,
            # rviz_node
        ]
    )
