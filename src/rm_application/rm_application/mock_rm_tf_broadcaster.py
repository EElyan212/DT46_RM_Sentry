"""
mock_rm_tf_broadcaster.py — 测试用 TF 广播节点

功能：
  模拟 rm_tf_broadcaster，用于在无实物环境下测试追击系统。
  订阅 /tracker/enemy_datas，发布：
    1. TF: camera_optical_link → enemy_link
    2. Topic: /tracker/enemy_pose (PoseStamped)

使用方式：
  ros2 run rm_application mock_rm_tf_broadcaster
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

from rm_interfaces.msg import EnemyCenter

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class MockTfBroadcaster(Node):
    def __init__(self):
        super().__init__('mock_rm_tf_broadcaster')

        # TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 订阅敌人数据
        enemy_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(
            EnemyCenter,
            '/tracker/enemy_datas',
            self.enemy_callback,
            enemy_qos
        )

        # 发布 /tracker/enemy_pose
        self.pose_pub = self.create_publisher(
            PoseStamped, '/tracker/enemy_pose', 10)

        self.get_logger().info('mock_rm_tf_broadcaster 已启动')
        self.get_logger().info('订阅: /tracker/enemy_datas')
        self.get_logger().info('发布: TF camera_optical_link→enemy_link, /tracker/enemy_pose')

    def enemy_callback(self, msg: EnemyCenter):
        if not msg.tracked:
            return

        # EnemyCenter 中 x/y/z 是相机坐标系下的坐标
        # 相机坐标系: x-右, y-下, z-前
        cam_x = msg.x
        cam_y = msg.y
        cam_z = msg.z

        # 发布 TF: camera_optical_link → enemy_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'camera_optical_link'
        tf_msg.child_frame_id = 'enemy_link'
        tf_msg.transform.translation.x = cam_x
        tf_msg.transform.translation.y = cam_y
        tf_msg.transform.translation.z = cam_z
        tf_msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_msg)

        # 发布 /tracker/enemy_pose
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'camera_optical_link'
        pose.pose.position.x = cam_x
        pose.pose.position.y = cam_y
        pose.pose.position.z = cam_z
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = MockTfBroadcaster()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
