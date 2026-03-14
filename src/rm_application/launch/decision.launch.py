import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rm_application'),
        'config',
        'decision_params.yaml'
    )
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rm_application',
            executable='rm_decision',
            name='rm_decision',
            output='screen',
            parameters=[config]
        ),
    ])