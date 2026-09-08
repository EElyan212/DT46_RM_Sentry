import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rm_application'),
        'config',
        'target_pose_server_params.yaml'
    )
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rm_application',
            executable='target_pose_server',
            name='target_pose_server',
            output='screen',
            parameters=[config]
        ),
    ])
