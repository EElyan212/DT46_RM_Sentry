import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rm_application'),
        'config',
        'chase_client_params.yaml'
    )
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rm_application',
            executable='chase_client',
            name='chase_client',
            output='screen',
            parameters=[config]
        ),
    ])
