import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory("rm_application"),
        "config",
    )
    behavior_tree_config = os.path.join(config_dir, "behavior_tree_params.yaml")
    chase_config = os.path.join(config_dir, "chase_params.yaml")

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="rm_application",
            executable="behavior_tree_node",
            output="screen",
            # 策略参数和共用视觉超时分别从对应配置读取
            parameters=[behavior_tree_config, chase_config],
        ),
    ])