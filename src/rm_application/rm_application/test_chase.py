#!/usr/bin/env python3
"""
test_chase.py — 测试 chase_client 的 Chase Action

使用方式：
  ros2 run rm_application test_chase
  
  或带参数：
  ros2 run rm_application test_chase -- --offset 2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rm_interfaces.action import Chase


class TestChaseNode(Node):
    def __init__(self):
        super().__init__('test_chase')
        self.chase_client = ActionClient(self, Chase, '/tracker/chase')

        # 声明参数
        self.declare_parameter('offset', 1.5)
        self.offset = self.get_parameter('offset').value

    def send_goal(self):
        """发送 Chase Goal"""
        self.get_logger().info('等待 chase_client...')
        if not self.chase_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('chase_client 未就绪')
            rclpy.shutdown()
            return

        goal = Chase.Goal()
        goal.offset_distance = self.offset

        self.get_logger().info(f'发送 Chase Goal: offset={self.offset}m')
        future = self.chase_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Goal 被接受/拒绝的回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal 被拒绝')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal 已接受，等待结果...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """收到 Result 的回调"""
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f'获取 Result 异常: {e}')
            rclpy.shutdown()
            return

        if result.success:
            self.get_logger().info(f'追击成功: {result.message}')
        else:
            self.get_logger().warn(f'追击失败: {result.message}')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TestChaseNode()
    node.send_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
