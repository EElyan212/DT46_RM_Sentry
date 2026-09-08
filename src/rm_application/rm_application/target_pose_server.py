"""
target_pose_server.py — 导航方追击目标点计算节点

功能：
  订阅视觉方发布的 /tracker/enemy_pose（相机坐标系 PoseStamped），
  通过 tf2 查询 map→enemy 获取 map 系坐标，提供 Action 服务。

通信链路：
  rm_tf_broadcaster ── /tracker/enemy_pose (PoseStamped) ──► 本节点
  tf2 树: map→odom→base_footprint→...→enemy
  调用方 ── GetTargetPose Action ──► 本节点 ──► 返回 PoseStamped
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# Action 相关
from rclpy.action import ActionServer
from rm_interfaces.action import GetTargetPose

# 消息类型
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header

# TF 监听（获取机器人在 map 坐标系下的位姿）
from tf2_ros import TransformListener, Buffer


class TargetPoseServer(Node):
    def __init__(self):
        super().__init__('target_pose_server')

        # ==================== 参数声明 ====================
        # 默认偏移距离（米）：当 Goal 中 offset_distance=0 时使用此值
        self.declare_parameter('default_offset_distance', 1.5)
        self.default_offset_distance = self.get_parameter('default_offset_distance').value

        # ==================== TF 监听器 ====================
        # 用于查询 map→enemy 和 map→base_footprint
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ==================== 订阅敌人位姿 ====================
        # /tracker/enemy_pose：相机坐标系 PoseStamped，用于超时判断 tracked 状态
        self.sub_enemy_pose = self.create_subscription(
            PoseStamped,
            '/tracker/enemy_pose',
            self.enemy_pose_callback,
            10  # DEFAULT QoS
        )

        # ==================== 存储最新数据 ====================
        self.enemy_pose = None
        self.enemy_pose_timestamp = None
        self.enemy_pose_timeout = 0.5  # 超时阈值（秒），超过未收到视为丢失

        # ==================== Action Server ====================
        self.action_callback_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(
            self,
            GetTargetPose,
            '/tracker/get_target_pose',
            self.execute_callback,
            callback_group=self.action_callback_group
        )

        self.get_logger().info('target_pose_server 已启动')
        self.get_logger().info(f'默认偏移距离: {self.default_offset_distance}m')

    def enemy_pose_callback(self, msg: PoseStamped):
        """存储最新的敌人位姿（相机坐标系，视觉方已转换好）"""
        self.enemy_pose = msg
        self.enemy_pose_timestamp = self.get_clock().now()

    def is_enemy_tracked(self):
        """通过消息超时判断敌人是否被跟踪（rm_tf_broadcaster 不发布时即为丢失）"""
        if self.enemy_pose is None or self.enemy_pose_timestamp is None:
            return False
        elapsed = (self.get_clock().now() - self.enemy_pose_timestamp).nanoseconds / 1e9
        return elapsed < self.enemy_pose_timeout

    def get_robot_pose(self):
        """
        通过 TF 获取机器人在 map 坐标系下的 (x, y) 坐标。
        返回: (x, y) 或 None（TF 查不到时）
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                Time(),
                Duration(seconds=0.1)
            )
            t = transform.transform.translation
            return (t.x, t.y)
        except Exception as e:
            self.get_logger().warn(f'TF 查找失败: {e}')
            return None

    def compute_target_pose(self, offset_distance):
        """
        核心计算：通过 tf2 获取敌人 map 坐标，计算偏移后的导航目标点。

        流程：
          1. tf2 查询 map → enemy_link（自动完成相机→机体→世界转换）
          2. TF 查询 map → base_link（获取机器人位置）
          3. 沿 机器人→敌人 方向，从敌人位置向后偏移
        """
        if not self.is_enemy_tracked():
            return None, False, '敌人未被跟踪或数据超时'

        # 通过 tf2 获取敌人在 map 坐标系下的绝对坐标
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'enemy', Time(), Duration(seconds=0.1))
            enemy_map_x = transform.transform.translation.x
            enemy_map_y = transform.transform.translation.y
        except Exception as e:
            return None, False, f'TF 查询 enemy 失败: {e}'

        # 获取机器人在 map 坐标系下的绝对位置
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return None, False, 'TF 查询 base_link 失败'

        robot_x, robot_y = robot_pose

        # 计算从机器人指向敌人的方向向量
        dx = enemy_map_x - robot_x
        dy = enemy_map_y - robot_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.01:
            pose = self._make_pose(enemy_map_x, enemy_map_y, 0.0)
            return pose, True, '机器人与敌人距离过近，返回敌人坐标'

        # 单位化方向向量
        ux = dx / dist
        uy = dy / dist

        # 沿方向从敌人位置向后偏移
        target_x = enemy_map_x - ux * offset_distance
        target_y = enemy_map_y - uy * offset_distance

        pose = self._make_pose(target_x, target_y, 0.0)
        return pose, True, f'目标点({target_x:.2f}, {target_y:.2f})，距敌{offset_distance:.1f}m'

    def _make_pose(self, x, y, z):
        """构造 PoseStamped 消息"""
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position = Point(x=float(x), y=float(y), z=float(z))
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return pose

    def execute_callback(self, goal_handle):
        """
        Action Goal 处理回调。

        流程：
          1. 确定偏移距离（Goal 指定 或 使用默认值）
          2. 尝试计算目标点，带有限次重试（应对瞬态 TF 失败）
          3. 成功返回结果，失败返回错误
        """
        offset_distance = goal_handle.request.offset_distance
        if offset_distance <= 0.0:
            offset_distance = self.default_offset_distance

        self.get_logger().info(f'收到 Goal: offset={offset_distance:.2f}m')

        max_retries = 50  # 最多重试 50 次（约 5 秒），匹配 client 超时
        for retry in range(max_retries):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal 已被取消')
                return GetTargetPose.Result()

            # 检查是否被跟踪
            if not self.is_enemy_tracked():
                feedback = GetTargetPose.Feedback()
                feedback.enemy_x = 0.0
                feedback.enemy_y = 0.0
                feedback.enemy_tracked = False
                goal_handle.publish_feedback(feedback)
                time.sleep(0.1)
                continue

            # 发布 Feedback（有跟踪数据时）
            feedback = GetTargetPose.Feedback()
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', 'enemy', Time(), Duration(seconds=0.1))
                feedback.enemy_x = transform.transform.translation.x
                feedback.enemy_y = transform.transform.translation.y
                feedback.enemy_tracked = True
            except Exception:
                feedback.enemy_x = 0.0
                feedback.enemy_y = 0.0
                feedback.enemy_tracked = False
            goal_handle.publish_feedback(feedback)

            # 计算目标点
            pose, success, message = self.compute_target_pose(offset_distance)

            if success:
                goal_handle.succeed()
                result = GetTargetPose.Result()
                result.target_pose = pose
                result.success = True
                result.message = message
                self.get_logger().info(f'返回结果: {message}')
                return result

            # 计算失败，短暂等待后重试（可能是瞬态 TF 失败）
            time.sleep(0.1)

        # 重试耗尽，返回失败
        self.get_logger().warn(f'重试超过 {max_retries} 次，无法计算目标点')
        goal_handle.abort()
        return GetTargetPose.Result()


def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseServer()
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
