import os

import launch
import launch_ros
import launch_ros.parameter_descriptions
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction


def generate_launch_description():
    # 声明模型目录参数，方便修改
    # 获取功能包的share路径
    urdf_package_path = get_package_share_directory("rm_description")
    # 默认的xacro路径
    default_xacro_path = os.path.join(
        urdf_package_path, "urdf", "rm_sentry", "rm_sentry.urdf.xacro"
    )
    default_rviz_path = os.path.join(
        urdf_package_path, "config", "rviz", "diasplay_rm_sentry.rviz"
    )
    default_gazebo_path = os.path.join(urdf_package_path, "world", "RMUL_game.world")
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name="rm_sentry_model",
        default_value=str(default_xacro_path),
        description="模型文件路径",
    )
    # 通过文件路径获取内容，并转换成参数值对象，以供传入 robot_state_publisher
    substitutions_command_result = launch.substitutions.Command(
        ["xacro ", launch.substitutions.LaunchConfiguration("rm_sentry_model")]
    )
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        substitutions_command_result, value_type=str
    )
    action_sentry_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_value}],
    )
    # 有gazebo是不需要它来发布
    action_joint_state_publisher = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )

    action_rviz_node = launch_ros.actions.Node(
        package="rviz2", executable="rviz2", arguments=["-d", default_rviz_path]
    )

    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments=[("world", default_gazebo_path), ("verbose", "true")],
    )
    # 通过话题方式加载urdf的模型内容
    action_spawn_entity = launch_ros.actions.Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description", "-entity", "rm_sentry"],
    )
    delayed_spawn = TimerAction(period=3.0, actions=[action_spawn_entity])
    return launch.LaunchDescription(
        [
            action_declare_arg_mode_path,
            action_sentry_state_publisher,
            action_rviz_node,
            action_launch_gazebo,
            delayed_spawn,
            # action_spawn_entity,
            # action_joint_state_publisher,
        ]
    )
