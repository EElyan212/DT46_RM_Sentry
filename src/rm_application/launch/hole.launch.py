import os

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("rm_application"),
        "config",
        "hole_params.yaml",
    )

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="rm_application",
            executable="hole_detector",
            name="hole_detector",
            output="screen",
            parameters=[config],
        ),
        launch_ros.actions.Node(
            package="rm_application",
            executable="hole_behavior",
            name="hole_behavior",
            output="screen",
            parameters=[config],
        ),
    ])
