import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("rm_application"),
        "config",
        "axis_align_params.yaml",
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.3"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            launch_ros.actions.Node(
                package="rm_application",
                executable="axis_align_fake_base",
                name="axis_align_fake_base",
                output="screen",
                parameters=[
                    {
                        "initial_x": LaunchConfiguration("initial_x"),
                        "initial_y": LaunchConfiguration("initial_y"),
                        "initial_yaw": LaunchConfiguration("initial_yaw"),
                        "cmd_vel_topic": "/sim/cmd_vel",
                        "pose_topic": "/sim/base_pose",
                    }
                ],
            ),
            launch_ros.actions.Node(
                package="rm_application",
                executable="axis_align",
                name="axis_align_node",
                output="screen",
                parameters=[
                    config,
                    {
                        "cmd_vel_topic": "/sim/cmd_vel",
                        "aligned_topic": "/sim/aligned",
                    },
                ],
            ),
        ]
    )
