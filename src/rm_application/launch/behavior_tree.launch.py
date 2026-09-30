import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取行为树参数文件的安装路径
    behavior_tree_config = os.path.join(
        get_package_share_directory("rm_application"),
        "config",
        "behavior_tree_params.yaml",
    )

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="rm_application",
            executable="behavior_tree_node",
            output="screen",
            parameters=[behavior_tree_config],
        ),
    ])