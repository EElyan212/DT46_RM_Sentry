"""
chase_client.py — 追击服务节点（可复用 Action Server）

功能：
  提供 Chase Action 服务，其他节点调用即可触发追击。
  内部订阅 /tracker/enemy_datas 检查 tracked 状态，
  调用 target_pose_server 获取目标点，通过 Nav2 执行导航。

通信链路：
  任意节点 ── Chase Action ──► 本节点
  本节点 ── GetTargetPose Action ──► target_pose_server
  本节点 ── Nav2 ──► 导航执行

使用方式：
  其他节点创建 ActionClient(Chase, '/tracker/chase') 并发送 Goal 即可触发追击。
"""

import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# Action 相关
from rclpy.action import ActionServer, ActionClient
from rm_interfaces.action import Chase, GetTargetPose

# 消息类型
from rm_interfaces.msg import EnemyCenter
from std_msgs.msg import Bool

# Nav2
from nav2_simple_commander.robot_navigator import BasicNavigator

# QoS 配置（与视觉方 BEST_EFFORT 匹配）
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class ChaseClient(Node):
    def __init__(self):
        super().__init__('chase_client')

        # ==================== 参数声明 ====================
        self.declare_parameter('offset_distance', 1.5)       # 偏移距离（米）
        self.declare_parameter('chase_rate', 10)             # 追击刷新频率（Hz）
        self.declare_parameter('enemy_pose_timeout', 0.5)     # 敌人数据超时阈值（秒）

        self.offset_distance = self.get_parameter('offset_distance').value
        self.chase_rate = self.get_parameter('chase_rate').value
        self.enemy_pose_timeout = self.get_parameter('enemy_pose_timeout').value

        # ==================== 状态变量 ====================
        self.enemy_tracked = False          # 当前敌人是否被跟踪
        self.enemy_data_timestamp = None    # 最后收到敌人数据的时间

        # ==================== 双线程同步变量 ====================
        self.target_pose_lock = threading.Lock()
        self.latest_target_pose = None      # 后台线程写入，主线程读取
        self.target_updater_thread = None   # 后台线程引用
        self.target_updater_stop_event = threading.Event()  # 停止信号

        # ==================== Nav2 导航器 ====================
        self.navigator = BasicNavigator()

        # ==================== Chase Action Server ====================
        self.action_callback_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(
            self,
            Chase,
            '/tracker/chase',
            self.execute_callback,
            callback_group=self.action_callback_group
        )

        # ==================== GetTargetPose Action Client ====================
        self.get_target_client = ActionClient(
            self,
            GetTargetPose,
            '/tracker/get_target_pose',
            callback_group=self.action_callback_group
        )

        # ==================== 订阅敌人数据（用于检查 tracked 状态）====================
        enemy_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub_enemy = self.create_subscription(
            EnemyCenter,
            '/tracker/enemy_datas',
            self.enemy_callback,
            enemy_qos
        )

        self.get_logger().info('chase_client 已启动')
        self.get_logger().info(f'偏移距离: {self.offset_distance}m')
        self.get_logger().info(f'追击频率: {self.chase_rate}Hz')
        self.get_logger().info(f'敌人数据超时: {self.enemy_pose_timeout}s')

    def enemy_callback(self, msg: EnemyCenter):
        """存储最新的敌人跟踪状态"""
        self.enemy_tracked = msg.tracked
        self.enemy_data_timestamp = self.get_clock().now()

    def is_enemy_tracked(self):
        """通过消息超时判断敌人是否被跟踪"""
        if self.enemy_data_timestamp is None:
            return False
        elapsed = (self.get_clock().now() - self.enemy_data_timestamp).nanoseconds / 1e9
        if elapsed > self.enemy_pose_timeout:
            self.enemy_tracked = False
        return self.enemy_tracked

    def _target_pose_updater(self, offset_distance):
        """
        后台线程：按 chase_rate 频率定时获取目标点。
        获取成功后将目标点存入 self.latest_target_pose。
        """
        interval = 1.0 / self.chase_rate
        self.get_logger().info(f'目标点更新线程已启动，频率: {self.chase_rate}Hz')

        while not self.target_updater_stop_event.is_set():
            # 检查敌人是否被跟踪
            if self.is_enemy_tracked():
                target_pose = self._get_target_pose(offset_distance)
                if target_pose is not None:
                    with self.target_pose_lock:
                        self.latest_target_pose = target_pose
                    self.get_logger().debug(
                        f'目标点已更新: ({target_pose.pose.position.x:.2f}, {target_pose.pose.position.y:.2f})')

            # 等待下一个周期，或收到停止信号时立即退出
            self.target_updater_stop_event.wait(timeout=interval)

        self.get_logger().info('目标点更新线程已停止')

    def _start_target_updater(self, offset_distance):
        """启动后台目标点更新线程"""
        self.target_updater_stop_event.clear()
        with self.target_pose_lock:
            self.latest_target_pose = None
        self.target_updater_thread = threading.Thread(
            target=self._target_pose_updater,
            args=(offset_distance,),
            daemon=True
        )
        self.target_updater_thread.start()

    def _stop_target_updater(self):
        """停止后台目标点更新线程"""
        self.target_updater_stop_event.set()
        if self.target_updater_thread is not None and self.target_updater_thread.is_alive():
            self.target_updater_thread.join(timeout=2.0)
        self.target_updater_thread = None

    def _has_pose_changed(self, pose1, pose2, threshold=0.1):
        """比较两个 PoseStamped 的位置差异，超过阈值视为变化"""
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        distance = math.sqrt(dx * dx + dy * dy)
        return distance > threshold

    def execute_callback(self, goal_handle):
        """
        Chase Action 处理回调。

        流程：
          1. 检查 tracked 状态
          2. 循环调用 GetTargetPose 获取目标点
          3. 通过 Nav2 导航
          4. 导航完成后重新获取目标点（追击循环）
        """
        offset_distance = goal_handle.request.offset_distance
        if offset_distance <= 0.0:
            offset_distance = self.offset_distance

        self.get_logger().info(f'收到 Chase Goal: offset={offset_distance:.2f}m')

        # 检查 tracked 状态
        if not self.is_enemy_tracked():
            result = Chase.Result()
            result.success = False
            result.message = '敌人未被跟踪 (tracked=false)'
            self.get_logger().warn(result.message)
            goal_handle.abort(result)
            return result

        # 等待 GetTargetPose Server 就绪
        if not self.get_target_client.wait_for_server(timeout_sec=3.0):
            result = Chase.Result()
            result.success = False
            result.message = 'target_pose_server 未就绪'
            self.get_logger().warn(result.message)
            goal_handle.abort(result)
            return result

        # 启动后台目标点更新线程
        self._start_target_updater(offset_distance)

        # 追击循环
        last_sent_pose = None  # 上次发送给 Nav2 的目标点

        try:
            while rclpy.ok():
                # 检查取消请求
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Chase Goal 已被取消')
                    goal_handle.canceled()
                    return Chase.Result()

                # 检查 tracked 状态
                if not self.is_enemy_tracked():
                    feedback = Chase.Feedback()
                    feedback.chasing = False
                    feedback.status = '等待敌人数据...'
                    goal_handle.publish_feedback(feedback)
                    time.sleep(0.1)
                    continue

                # 从后台线程获取最新目标点
                with self.target_pose_lock:
                    current_target_pose = self.latest_target_pose

                # 检查是否有新目标点
                if current_target_pose is None:
                    feedback = Chase.Feedback()
                    feedback.chasing = True
                    feedback.status = '等待目标点...'
                    goal_handle.publish_feedback(feedback)
                    time.sleep(0.1)
                    continue

                # 检查目标点是否发生变化
                need_new_nav = False
                if last_sent_pose is None:
                    # 首次获取目标点，需要导航
                    need_new_nav = True
                elif self._has_pose_changed(last_sent_pose, current_target_pose, threshold=0.1):
                    # 目标点位置有变化，取消旧导航，导航到新目标点
                    need_new_nav = True
                    self.get_logger().info('目标点已更新，取消旧导航任务')
                    self.navigator.cancelTask()

                if need_new_nav:
                    # 发布 Feedback：正在导航
                    feedback = Chase.Feedback()
                    feedback.chasing = True
                    feedback.status = f'导航到 ({current_target_pose.pose.position.x:.2f}, {current_target_pose.pose.position.y:.2f})'
                    goal_handle.publish_feedback(feedback)

                    # 发送新的导航任务
                    self.navigator.goToPose(current_target_pose)
                    last_sent_pose = current_target_pose

                # 检查导航是否完成
                if self.navigator.isTaskComplete():
                    # 导航完成，检查是否有新目标点（下一轮循环会处理）
                    self.get_logger().info('导航任务已完成')
                    last_sent_pose = None

                time.sleep(0.1)

        finally:
            # 停止后台目标点更新线程
            self._stop_target_updater()

    def _get_target_pose(self, offset_distance):
        """
        调用 GetTargetPose Action 获取目标点。
        返回: PoseStamped 或 None（失败时）

        使用 threading.Event 等待异步结果，避免在 executor 回调中
        嵌套调用 spin_until_future_complete 导致竞态/死锁。
        """
        goal = GetTargetPose.Goal()
        goal.offset_distance = offset_distance

        # ---- 发送 Goal ----
        goal_event = threading.Event()
        goal_future = self.get_target_client.send_goal_async(goal)
        goal_future.add_done_callback(lambda f: goal_event.set())

        if not goal_event.wait(timeout=3.0):
            self.get_logger().warn('GetTargetPose Goal 发送失败')
            return None

        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('GetTargetPose Goal 被拒绝')
            return None

        # ---- 等待 Result ----
        result_event = threading.Event()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: result_event.set())

        if not result_event.wait(timeout=5.0):
            self.get_logger().warn('GetTargetPose Result 获取失败')
            return None

        result = result_future.result().result
        if not result.success:
            self.get_logger().warn(f'GetTargetPose 失败: {result.message}')
            return None

        return result.target_pose


def main(args=None):
    rclpy.init(args=args)
    node = ChaseClient()
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
