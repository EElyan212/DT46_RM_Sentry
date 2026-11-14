"""按请求计算追击目标点；没有请求时不查询目标 TF 或调用规划器。"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from rclpy.action import (
    ActionServer,
    ActionClient,
    CancelResponse,
    GoalResponse,
)

from rm_interfaces.action import GetTargetPose
from rm_interfaces.msg import EnemyCenter
from nav2_msgs.action import ComputePathToPose
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header

from tf2_ros import TransformListener, Buffer


class TargetPoseServer(Node):
    def __init__(self):
        super().__init__('target_pose_server')

        # 默认追击距离只由计算端管理，请求中的正数可临时覆盖
        self.declare_parameter('offset_distance', 0.5)
        self.offset_distance = float(
            self.get_parameter('offset_distance').value
        )

        # 优先沿规划路径选点，规划失败时保留原有直线回退
        self.declare_parameter('use_path_offset', True)
        self.use_path_offset = self.get_parameter('use_path_offset').value

        # 自己或敌人移动超过这个距离后，重新规划路径
        self.declare_parameter('enemy_replan_threshold', 0.1)
        self.enemy_replan_threshold = float(
            self.get_parameter('enemy_replan_threshold').value
        )

        self.declare_parameter('planner_action_name', '/compute_path_to_pose')
        self.planner_action_name = self.get_parameter('planner_action_name').value

        self.declare_parameter('planner_timeout', 0.5)
        self.planner_timeout = float(
            self.get_parameter('planner_timeout').value
        )

        # 与追击执行端、行为树使用同一份视觉超时配置
        self.declare_parameter('vision_timeout', 0.5)
        self.vision_timeout = float(
            self.get_parameter('vision_timeout').value
        )

        if not all(
            math.isfinite(value) and value > 0.0
            for value in (
                self.offset_distance,
                self.planner_timeout,
                self.vision_timeout,
            )
        ):
            raise ValueError('偏移距离、规划超时和视觉超时必须为有限正数')

        # 允许设为0，表示位置发生变化就重新规划
        if (
            not math.isfinite(self.enemy_replan_threshold)
            or self.enemy_replan_threshold < 0.0
        ):
            raise ValueError('路径重规划距离必须为有限非负数')

        # TF 监听保持运行，只有计算请求到来后才查询位置
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.planner_callback_group = ReentrantCallbackGroup()

        self.planner_client = ActionClient(
            self,
            ComputePathToPose,
            self.planner_action_name,
            callback_group=self.planner_callback_group,
        )

        # 这里只缓存可复用的路径，不缓存成品目标点供请求直接读取
        self._path_cache = None
        self._plan_fail_until = 0.0
        self.plan_fail_backoff = 0.5

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
            enemy_qos,
        )

        self._enemy_tracked = False
        self._enemy_data_timestamp = None
        self._vision_generation = 0

        # 防止两个请求同时修改路径缓存
        self._request_lock = threading.Lock()
        self._request_busy = False

        self.action_callback_group = ReentrantCallbackGroup()

        self.action_server = ActionServer(
            self,
            GetTargetPose,
            '/tracker/get_target_pose',
            self.execute_callback,
            callback_group=self.action_callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )

        self.get_logger().info(
            'target_pose_server 已启动：收到请求才计算目标点'
        )

    def goal_callback(self, _request):
        # 只接受串行请求，不排队计算已经过时的目标
        with self._request_lock:
            if self._request_busy:
                return GoalResponse.REJECT

            self._request_busy = True

        return GoalResponse.ACCEPT

    def enemy_data_callback(self, msg: EnemyCenter):
        # 此回调只更新状态，不触发目标计算
        now = self.get_clock().now()
        interrupted = not msg.tracked or not self._enemy_tracked

        if self._enemy_data_timestamp is not None:
            age = (
                now - self._enemy_data_timestamp
            ).nanoseconds / 1e9

            interrupted = (
                interrupted
                or not 0.0 <= age <= self.vision_timeout
            )

        # 丢失或断流后恢复时更换代次
        # 防止把丢失前开始计算的目标当作恢复后的新目标
        if interrupted:
            self._vision_generation += 1

        self._enemy_tracked = bool(msg.tracked)
        self._enemy_data_timestamp = now

    def is_enemy_tracked(self):
        if (
            not self._enemy_tracked
            or self._enemy_data_timestamp is None
        ):
            return False

        elapsed = (
            self.get_clock().now() - self._enemy_data_timestamp
        ).nanoseconds / 1e9

        return 0.0 <= elapsed <= self.vision_timeout

    def get_robot_pose(self):
        # 自身坐标用于计算追击目标点和规划起点
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                Time(),
                Duration(seconds=0.1),
            )

            position = transform.transform.translation
            return position.x, position.y

        except Exception as error:
            self.get_logger().warn(
                f'TF 查找失败: {error}'
            )
            return None

    def compute_target_pose(
        self,
        offset_distance,
        interrupted=lambda: False,
    ):
        # 此方法仅由一次 GetTargetPose 请求触发
        if not self.is_enemy_tracked():
            return None, False, '敌人未被跟踪或数据超时'

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'enemy',
                Time(),
                Duration(seconds=0.1),
            )

            tf_stamp = Time.from_msg(transform.header.stamp)
            age = (
                self.get_clock().now().nanoseconds - tf_stamp.nanoseconds
            ) / 1e9

            # 复用共用视觉超时，不新增TF超时参数
            if not 0.0 <= age <= self.vision_timeout:
                return None, False, 'enemy TF 已过期或时间异常'

            enemy_map_x = transform.transform.translation.x
            enemy_map_y = transform.transform.translation.y

        except Exception as error:
            return None, False, f'TF 查询 enemy 失败: {error}'

        robot_pose = self.get_robot_pose()

        if robot_pose is None:
            return None, False, 'TF 查询 base_footprint 失败'

        if interrupted():
            return None, False, '目标计算已中断'

        robot_x, robot_y = robot_pose

        dx = enemy_map_x - robot_x
        dy = enemy_map_y - robot_y
        distance = math.hypot(dx, dy)

        if distance < 0.01:
            pose = self._make_pose(
                enemy_map_x,
                enemy_map_y,
                0.0,
            )
            return pose, True, '机器人与敌人距离过近，返回敌人坐标'

        # 保留原有路径弧长偏移算法
        if self.use_path_offset:
            path_points = self._get_or_plan_path(
                (robot_x, robot_y),
                (enemy_map_x, enemy_map_y),
                interrupted,
            )

            if interrupted():
                return None, False, '目标计算已中断'

            if path_points:
                target_x, target_y, clamped = self._offset_along_path(
                    path_points,
                    offset_distance,
                )

                pose = self._make_pose(
                    target_x,
                    target_y,
                    0.0,
                )

                if clamped:
                    message = (
                        f'路径偏移(clamp)({target_x:.2f}, {target_y:.2f})，'
                        f'路径长度不足{offset_distance:.1f}m'
                    )
                else:
                    message = (
                        f'路径偏移({target_x:.2f}, {target_y:.2f})，'
                        f'沿路径距敌{offset_distance:.1f}m'
                    )

                return pose, True, message

            self.get_logger().warn(
                '全局规划不可用，回退直线偏移',
                throttle_duration_sec=2.0,
            )

        # 保留原有规划失败后的直线偏移回退
        unit_x = dx / distance
        unit_y = dy / distance

        target_x = enemy_map_x - unit_x * offset_distance
        target_y = enemy_map_y - unit_y * offset_distance

        pose = self._make_pose(
            target_x,
            target_y,
            0.0,
        )

        if self.use_path_offset:
            message = (
                f'直线偏移(规划失败回退)'
                f'({target_x:.2f}, {target_y:.2f})，'
                f'距敌{offset_distance:.1f}m'
            )
        else:
            message = (
                f'目标点({target_x:.2f}, {target_y:.2f})，'
                f'距敌{offset_distance:.1f}m'
            )

        return pose, True, message

    def _get_or_plan_path(
        self,
        robot_xy,
        enemy_xy,
        interrupted=lambda: False,
    ):
        enemy = (enemy_xy[0], enemy_xy[1])

        # 保留规划失败后的短暂退避
        if time.time() < self._plan_fail_until:
            return None

        cache = self._path_cache

        if cache is not None:
            cached_enemy = cache['enemy']

            enemy_moved = math.hypot(
                enemy[0] - cached_enemy[0],
                enemy[1] - cached_enemy[1],
            )

            # 机器人起点发生明显变化时，也不能复用旧路径起点
            robot_moved = math.hypot(
                robot_xy[0] - cache['robot'][0],
                robot_xy[1] - cache['robot'][1],
            )

            if (
                max(enemy_moved, robot_moved)
                <= self.enemy_replan_threshold
            ):
                return cache['points']

        points = self._plan_path(
            robot_xy,
            enemy,
            interrupted,
        )

        if interrupted():
            self._path_cache = None
            return None

        if points:
            self._path_cache = {
                'enemy': enemy,
                'robot': tuple(robot_xy),
                'points': points,
            }
            self._plan_fail_until = 0.0
            return points

        self._path_cache = None
        self._plan_fail_until = (
            time.time() + self.plan_fail_backoff
        )
        return None

    def _plan_path(
        self,
        robot_xy,
        enemy_xy,
        interrupted=lambda: False,
    ):
        if not self.planner_client.server_is_ready():
            deadline = (
                time.time() + min(self.planner_timeout, 0.3)
            )

            while (
                not self.planner_client.server_is_ready()
                and time.time() < deadline
            ):
                if interrupted():
                    return None

                time.sleep(0.02)

            if not self.planner_client.server_is_ready():
                self.get_logger().warn(
                    f'规划器 {self.planner_action_name} 未就绪',
                    throttle_duration_sec=5.0,
                )
                return None

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = self._make_pose(
            robot_xy[0],
            robot_xy[1],
            0.0,
        )
        goal_msg.goal = self._make_pose(
            enemy_xy[0],
            enemy_xy[1],
            0.0,
        )
        goal_msg.planner_id = 'GridBased'
        goal_msg.use_start = True

        deadline = time.time() + self.planner_timeout

        try:
            send_future = self.planner_client.send_goal_async(
                goal_msg
            )
        except Exception as error:
            self.get_logger().warn(
                f'发送规划 Goal 失败: {error}'
            )
            return None

        while not send_future.done() and time.time() < deadline:
            if interrupted():
                break

            time.sleep(0.01)

        if not send_future.done():
            # 接受答复迟到时，也取消这个已经失效的规划请求
            def cancel_late(future):
                try:
                    handle = future.result()

                    if handle is not None and handle.accepted:
                        handle.cancel_goal_async()

                except Exception:
                    pass

            send_future.add_done_callback(cancel_late)
            return None

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('规划 Goal 被拒绝')
            return None

        if interrupted():
            goal_handle.cancel_goal_async()
            return None

        result_future = goal_handle.get_result_async()

        while not result_future.done() and time.time() < deadline:
            if interrupted():
                goal_handle.cancel_goal_async()
                return None

            time.sleep(0.01)

        if not result_future.done():
            goal_handle.cancel_goal_async()
            self.get_logger().warn(
                '等待规划结果超时，已取消'
            )
            return None

        try:
            wrapped = result_future.result()
        except Exception as error:
            self.get_logger().warn(
                f'获取规划结果失败: {error}'
            )
            return None

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'全局规划失败 (status={wrapped.status})',
                throttle_duration_sec=2.0,
            )
            return None

        points = [
            (
                item.pose.position.x,
                item.pose.position.y,
            )
            for item in wrapped.result.path.poses
        ]

        if not points:
            self.get_logger().warn('全局规划返回空路径')
            return None

        return points

    def _offset_along_path(self, points, offset_distance):
        # 从路径末端向起点回退指定弧长
        if len(points) == 1:
            x, y = points[0]
            return x, y, True

        remaining = offset_distance

        for index in range(len(points) - 1, 0, -1):
            x1, y1 = points[index]
            x0, y0 = points[index - 1]

            segment = math.hypot(
                x1 - x0,
                y1 - y0,
            )

            if segment < 1e-9:
                continue

            if remaining <= segment:
                ratio = remaining / segment

                return (
                    x1 + (x0 - x1) * ratio,
                    y1 + (y0 - y1) * ratio,
                    False,
                )

            remaining -= segment

        # 路径长度不足时，使用本次规划路径的起点
        return points[0][0], points[0][1], True

    def _make_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        pose.pose.position = Point(
            x=float(x),
            y=float(y),
            z=float(z),
        )
        pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=0.0,
            w=1.0,
        )

        return pose

    def execute_callback(self, goal_handle):
        # 每次请求单独计算，不从成品目标缓存中读取结果
        result = GetTargetPose.Result()
        generation = self._vision_generation

        def interrupted():
            return (
                not rclpy.ok()
                or goal_handle.is_cancel_requested
                or generation != self._vision_generation
                or not self.is_enemy_tracked()
            )

        try:
            offset = float(
                goal_handle.request.offset_distance
            )

            if not math.isfinite(offset):
                raise ValueError(
                    '目标偏移距离必须为有限数'
                )

            if offset <= 0.0:
                # 未指定正数时，使用本节点配置的默认追击距离
                offset = self.offset_distance

            pose = None
            success = False
            message = '敌人未被跟踪或数据超时'

            if not interrupted():
                pose, success, message = self.compute_target_pose(
                    offset,
                    interrupted,
                )

            # 计算中途取消、丢失或重新跟踪，都不能返回此前的点
            if interrupted():
                pose = None
                success = False
                message = '目标计算已取消或视觉目标已失效'

            success = bool(
                success and pose is not None
            )

            result.success = success
            result.has_target = success
            result.message = message

            if success:
                result.target_pose = pose
                goal_handle.succeed()

            else:
                # 失效后不能让后续请求沿用旧的路径状态
                self._path_cache = None
                self._plan_fail_until = 0.0

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    # 本次没取得目标点；是否结束追击由追击端和行为树决定
                    goal_handle.abort()

            return result

        except Exception as error:
            self._path_cache = None
            self._plan_fail_until = 0.0

            result.message = (
                f'目标点计算失败: {error}'
            )

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()

            return result

        finally:
            with self._request_lock:
                self._request_busy = False


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