"""
target_pose_server.py — 导航方追击目标点计算节点（解耦版）

功能：
  直接订阅视觉方 /tracker/enemy_datas（EnemyCenter, BEST_EFFORT），以 msg.tracked
  为准判定视觉锁定；只要锁定即由后台线程自主计算目标点并缓存，
  通过 GetTargetPose Action 将缓存结果返回给调用方（chase_client / 行为树）。

目标点计算：
  优先调用 Nav2 全局规划器（ComputePathToPose）规划 机器人→敌人 路径，
  从路径末端沿路径按弧长回退 offset_distance 得到目标点（保证不在障碍物内）；
  敌人移动超过阈值才重新规划，路径缓存复用。
  规划失败/服务未就绪/路径过短时按规则回退（直线偏移 / clamp 路径起点）。

通信链路：
  视觉方 ── /tracker/enemy_datas (EnemyCenter) ──► 本节点（tracked 判定 + 后台计算）
  tf2 树: map→odom→base_footprint→...→enemy（rm_tf_broadcaster 发布 enemy 系 TF）
  本节点 ── ComputePathToPose ──► Nav2 planner_server
  调用方 ── GetTargetPose Action ──► 本节点 ──► 返回缓存结果（Result 含 has_target）
"""

import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Action 相关
from rclpy.action import ActionServer, ActionClient
from rm_interfaces.action import GetTargetPose
from rm_interfaces.msg import EnemyCenter
from nav2_msgs.action import ComputePathToPose
from action_msgs.msg import GoalStatus

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
        self.declare_parameter('default_offset_distance', 0.5)
        self.default_offset_distance = self.get_parameter('default_offset_distance').value

        # 路径偏移模式开关（false 时退回旧的直线偏移逻辑）
        self.declare_parameter('use_path_offset', True)
        self.use_path_offset = self.get_parameter('use_path_offset').value

        # 敌人位姿相对上次规划移动超过该距离（米）才重新全局规划
        self.declare_parameter('enemy_replan_threshold', 0.1)
        self.enemy_replan_threshold = self.get_parameter('enemy_replan_threshold').value

        # Nav2 全局规划器 action 名称
        self.declare_parameter('planner_action_name', '/compute_path_to_pose')
        self.planner_action_name = self.get_parameter('planner_action_name').value

        # 单次全局规划超时（秒）
        self.declare_parameter('planner_timeout', 0.5)
        self.planner_timeout = float(self.get_parameter('planner_timeout').value)

        # 后台计算频率（Hz）：视觉锁定时自主计算目标点
        self.declare_parameter('compute_rate', 5.0)
        self.compute_rate = float(self.get_parameter('compute_rate').value)

        # 敌人数据超时（秒）：超过未收到 enemy_datas 视为丢失
        self.declare_parameter('tracked_timeout', 0.5)
        self.tracked_timeout = float(self.get_parameter('tracked_timeout').value)

        # Action Goal 到达后等待后台缓存刷新的上限（秒）
        self.declare_parameter('fresh_wait', 0.3)
        self.fresh_wait = float(self.get_parameter('fresh_wait').value)

        # ==================== TF 监听器 ====================
        # 用于查询 map→enemy 和 map→base_footprint
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ==================== 全局规划 Action Client ====================
        self.planner_callback_group = ReentrantCallbackGroup()
        self.planner_client = ActionClient(
            self,
            ComputePathToPose,
            self.planner_action_name,
            callback_group=self.planner_callback_group
        )
        # 路径缓存：{'enemy': (x, y), 'points': [(x, y), ...]}，敌人移动超阈值才重规划
        self._path_cache = None
        # 规划失败后的负缓存截止时间（期间直接回退直线偏移，不重复打规划器）
        self._plan_fail_until = 0.0
        self.plan_fail_backoff = 0.5

        # ==================== 订阅敌人数据（视觉锁定判定）====================
        # /tracker/enemy_datas：以 msg.tracked 为准，QoS 与视觉方 BEST_EFFORT 匹配
        enemy_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub_enemy_datas = self.create_subscription(
            EnemyCenter,
            '/tracker/enemy_datas',
            self.enemy_data_callback,
            enemy_qos
        )
        self._enemy_tracked = False
        self._enemy_data_timestamp = None

        # ==================== 目标点缓存（后台写，Action 读）====================
        self._active_offset = float(self.default_offset_distance)
        self._cache_lock = threading.Lock()
        self._cache = {
            'pose': None,            # PoseStamped 或 None
            'has_target': False,     # 是否算出有效目标点
            'message': '尚未计算',    # 最近一次计算的状态描述
            'stamp': 0.0,            # 缓存更新时刻（monotonic）
            'enemy_x': 0.0,          # 敌人 map 坐标（最近一次计算时）
            'enemy_y': 0.0,
        }

        # ==================== 后台计算线程 ====================
        self._compute_stop = threading.Event()
        self._compute_thread = threading.Thread(
            target=self._compute_loop, daemon=True, name='target_compute')

        # ==================== Action Server ====================
        self.action_callback_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(
            self,
            GetTargetPose,
            '/tracker/get_target_pose',
            self.execute_callback,
            callback_group=self.action_callback_group
        )

        self._compute_thread.start()

        self.get_logger().info('target_pose_server 已启动（解耦版：后台自主计算）')
        self.get_logger().info(f'默认偏移距离: {self.default_offset_distance}m')
        self.get_logger().info(
            f'路径偏移模式: {"开" if self.use_path_offset else "关"}，'
            f'敌人重规划阈值: {self.enemy_replan_threshold}m，'
            f'规划器: {self.planner_action_name}，'
            f'计算频率: {self.compute_rate}Hz，'
            f'数据超时: {self.tracked_timeout}s'
        )

    # ==================== 视觉锁定判定 ====================

    def enemy_data_callback(self, msg: EnemyCenter):
        """存储视觉锁定状态（msg.tracked）与消息时间戳"""
        self._enemy_tracked = msg.tracked
        self._enemy_data_timestamp = self.get_clock().now()

    def is_enemy_tracked(self):
        """视觉锁定且消息未超时才算被跟踪"""
        if not self._enemy_tracked or self._enemy_data_timestamp is None:
            return False
        elapsed = (self.get_clock().now() - self._enemy_data_timestamp).nanoseconds / 1e9
        return elapsed < self.tracked_timeout

    # ==================== 后台计算线程 ====================

    def _compute_loop(self):
        """
        后台线程：只要视觉锁定就按 compute_rate 计算目标点并写入缓存；
        未锁定时把缓存置为 has_target=False。Action Goal 只读缓存。
        """
        interval = 1.0 / max(self.compute_rate, 0.1)
        while not self._compute_stop.is_set():
            if self.is_enemy_tracked():
                offset = self._active_offset
                pose, success, message = self.compute_target_pose(offset)
                enemy_x, enemy_y = self._cache_enemy_pose()
                with self._cache_lock:
                    self._cache['pose'] = pose
                    self._cache['has_target'] = success
                    self._cache['message'] = message
                    self._cache['enemy_x'] = enemy_x
                    self._cache['enemy_y'] = enemy_y
                    self._cache['stamp'] = time.monotonic()
            else:
                with self._cache_lock:
                    self._cache['pose'] = None
                    self._cache['has_target'] = False
                    self._cache['message'] = '敌人未被跟踪或数据超时'
                    self._cache['enemy_x'] = 0.0
                    self._cache['enemy_y'] = 0.0
                    self._cache['stamp'] = time.monotonic()
            self._compute_stop.wait(timeout=interval)

    def _cache_enemy_pose(self):
        """TF 查询敌人 map 坐标（供 Feedback 用），失败返回 (0, 0)"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'enemy', Time(), Duration(seconds=0.1))
            return (transform.transform.translation.x,
                    transform.transform.translation.y)
        except Exception:
            return 0.0, 0.0

    # ==================== TF 查询 ====================

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
          2. TF 查询 map → base_footprint（获取机器人位置）
          3. 优先：全局规划 机器人→敌人，从路径末端沿路径回退 offset_distance
             （敌人移动超阈值或无缓存时重新规划；路径过短则 clamp 到路径起点）
          4. 回退：规划失败/关闭路径模式时，沿 机器人→敌人 直线从敌人向后偏移
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

        # 路径偏移模式：先尝试沿全局规划路径回退
        if self.use_path_offset:
            path_points = self._get_or_plan_path(
                (robot_x, robot_y), (enemy_map_x, enemy_map_y))
            if path_points:
                target_x, target_y, clamped = self._offset_along_path(
                    path_points, offset_distance)
                pose = self._make_pose(target_x, target_y, 0.0)
                if clamped:
                    message = (
                        f'路径偏移(clamp)({target_x:.2f}, {target_y:.2f})，'
                        f'路径长度不足{offset_distance:.1f}m')
                else:
                    message = (
                        f'路径偏移({target_x:.2f}, {target_y:.2f})，'
                        f'沿路径距敌{offset_distance:.1f}m')
                return pose, True, message
            self.get_logger().warn(
                '全局规划不可用，回退直线偏移', throttle_duration_sec=2.0)

        # 单位化方向向量（直线偏移，规划失败时的回退路径）
        ux = dx / dist
        uy = dy / dist

        # 沿方向从敌人位置向后偏移
        target_x = enemy_map_x - ux * offset_distance
        target_y = enemy_map_y - uy * offset_distance

        pose = self._make_pose(target_x, target_y, 0.0)
        if self.use_path_offset:
            message = (
                f'直线偏移(规划失败回退)({target_x:.2f}, {target_y:.2f})，'
                f'距敌{offset_distance:.1f}m')
        else:
            message = f'目标点({target_x:.2f}, {target_y:.2f})，距敌{offset_distance:.1f}m'
        return pose, True, message

    def _get_or_plan_path(self, robot_xy, enemy_xy):
        """
        获取通往敌人的全局路径点列表；必要时重新规划并缓存。

        重规划条件：无缓存，或敌人相对上次规划参考点移动超过阈值。
        规划失败时清空缓存并返回 None（上层回退直线偏移）。
        返回: [(x, y), ...] 或 None
        """
        enemy = (enemy_xy[0], enemy_xy[1])
        if time.time() < self._plan_fail_until:
            return None

        cache = self._path_cache
        if cache is not None:
            cached_enemy = cache['enemy']
            moved = math.hypot(enemy[0] - cached_enemy[0],
                               enemy[1] - cached_enemy[1])
            if moved <= self.enemy_replan_threshold:
                return cache['points']

        points = self._plan_path(robot_xy, enemy)
        if points:
            self._path_cache = {'enemy': enemy, 'points': points}
            self._plan_fail_until = 0.0
            return points

        # 规划失败：作废缓存并短暂退避，期间回退直线偏移
        self._path_cache = None
        self._plan_fail_until = time.time() + self.plan_fail_backoff
        return None

    def _plan_path(self, robot_xy, enemy_xy):
        """
        调用 Nav2 planner_server 的 ComputePathToPose 规划 机器人→敌人 路径。
        返回: [(x, y), ...] 或 None（服务未就绪/超时/失败/空路径）
        """
        if not self.planner_client.server_is_ready():
            # 给服务短暂就绪时间，避免启动竞争
            deadline = time.time() + min(self.planner_timeout, 0.3)
            while not self.planner_client.server_is_ready() and time.time() < deadline:
                time.sleep(0.02)
            if not self.planner_client.server_is_ready():
                self.get_logger().warn(
                    f'规划器 {self.planner_action_name} 未就绪',
                    throttle_duration_sec=5.0)
                return None

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = self._make_pose(robot_xy[0], robot_xy[1], 0.0)
        goal_msg.goal = self._make_pose(enemy_xy[0], enemy_xy[1], 0.0)
        goal_msg.planner_id = 'GridBased'
        goal_msg.use_start = True

        deadline = time.time() + self.planner_timeout

        try:
            send_future = self.planner_client.send_goal_async(goal_msg)
        except Exception as e:
            self.get_logger().warn(f'发送规划 Goal 失败: {e}')
            return None

        while not send_future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not send_future.done():
            self.get_logger().warn('发送规划 Goal 超时')
            return None

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('规划 Goal 被拒绝')
            return None

        result_future = goal_handle.get_result_async()
        while not result_future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not result_future.done():
            goal_handle.cancel_goal_async()
            self.get_logger().warn('等待规划结果超时，已取消')
            return None

        try:
            wrapped = result_future.result()
        except Exception as e:
            self.get_logger().warn(f'获取规划结果失败: {e}')
            return None

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'全局规划失败 (status={wrapped.status})',
                throttle_duration_sec=2.0)
            return None

        poses = wrapped.result.path.poses
        points = [(p.pose.position.x, p.pose.position.y) for p in poses]
        if not points:
            self.get_logger().warn('全局规划返回空路径')
            return None
        return points

    def _offset_along_path(self, points, offset_distance):
        """
        从路径末端（≈敌人）沿路径向起点方向回退 arc_length=offset_distance。
        路径总长不足时 clamp 到路径起点。
        返回: (x, y, clamped)
        """
        if len(points) == 1:
            x, y = points[0]
            return x, y, True

        remaining = offset_distance
        for i in range(len(points) - 1, 0, -1):
            x1, y1 = points[i]
            x0, y0 = points[i - 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg < 1e-9:
                continue
            if remaining <= seg:
                t = remaining / seg
                return x1 + (x0 - x1) * t, y1 + (y0 - y1) * t, False
            remaining -= seg

        # 路径总长 < offset：clamp 到路径起点（规划时的机器人位置）
        return points[0][0], points[0][1], True

    def _make_pose(self, x, y, z):
        """构造 PoseStamped 消息"""
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position = Point(x=float(x), y=float(y), z=float(z))
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return pose

    # ==================== Action Server ====================

    def execute_callback(self, goal_handle):
        """
        Action Goal 处理回调（解耦后为纯读缓存）：

          1. Goal.offset > 0 → 更新活动 offset（后台随之使用）
          2. 等待后台缓存刷新（最多 fresh_wait 秒）
          3. 发布 Feedback（enemy_x/y、enemy_tracked、has_target、target_pose）
          4. 返回 Result（success == has_target）
        """
        offset_distance = goal_handle.request.offset_distance
        if offset_distance > 0.0 and abs(offset_distance - self._active_offset) > 1e-6:
            self.get_logger().info(
                f'更新活动 offset: {self._active_offset:.2f} → {offset_distance:.2f}m')
            self._active_offset = float(offset_distance)

        # 等待后台缓存刷新（目标是拿到 ≥ 本次 Goal 到达时刻 的数据）
        goal_arrival = time.monotonic()
        deadline = goal_arrival + self.fresh_wait
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal 已被取消')
                return GetTargetPose.Result()
            with self._cache_lock:
                if self._cache['stamp'] >= goal_arrival:
                    break
            time.sleep(0.02)

        # 读取缓存快照
        with self._cache_lock:
            pose = self._cache['pose']
            has_target = self._cache['has_target']
            message = self._cache['message']
            enemy_x = self._cache['enemy_x']
            enemy_y = self._cache['enemy_y']

        tracked = self.is_enemy_tracked()

        # 发布 Feedback（完整状态）
        feedback = GetTargetPose.Feedback()
        feedback.enemy_x = float(enemy_x)
        feedback.enemy_y = float(enemy_y)
        feedback.enemy_tracked = tracked
        feedback.has_target = bool(has_target)
        if pose is not None:
            feedback.target_pose = pose
        goal_handle.publish_feedback(feedback)

        # 返回 Result
        result = GetTargetPose.Result()
        result.success = bool(has_target)
        result.has_target = bool(has_target)
        result.message = message
        if pose is not None:
            result.target_pose = pose

        if has_target:
            goal_handle.succeed()
            self.get_logger().info(f'返回结果: {message}')
        else:
            goal_handle.abort()
            self.get_logger().warn(f'无目标点: {message}')
        return result


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
        node._compute_stop.set()
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
