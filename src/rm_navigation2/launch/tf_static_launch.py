import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        # ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'map', '--child-frame-id', 'odom']
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'base_link', '--child-frame-id', 'base_footprint']
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'odom', '--child-frame-id', 'base_link']
        # ),
    ])

if __name__ == '__main__':
    generate_launch_description()