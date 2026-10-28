#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class GravityTfPublisher(Node):
    def __init__(self):
        super().__init__('gravity_tf_publisher')
        self.broadcaster = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'livox_frame'
        t.child_frame_id = 'gravity_aligned'

        # 平移按需填
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.7071068
        t.transform.rotation.w = -0.7071068

        self.broadcaster.sendTransform(t)
        self.get_logger().info('Published static TF: livox_frame -> gravity_aligned')


def main(args=None):
    rclpy.init(args=args)
    node = GravityTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
