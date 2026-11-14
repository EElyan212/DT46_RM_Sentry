"""行为树启动 Chase 后，串行按需获取目标点并管理 Nav2 导航。"""

import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rm_interfaces.action import Chase, GetTargetPose
from rm_interfaces.msg import EnemyCenter


class ChaseClient(Node):
    def __init__(self):
        super().__init__('chase_client')

        # 视觉超时与目标计算端、行为树共用；执行端不保存默认偏移距离
        self.declare_parameter('vision_timeout', 0.5)
        self.declare_parameter('chase_rate', 10)

        self.vision_timeout = float(
            self.get_parameter('vision_timeout').value
        )
        self.chase_rate = float(
            self.get_parameter('chase_rate').value
        )

        if not all(
            math.isfinite(value) and value > 0.0
            for value in (self.vision_timeout, self.chase_rate)
        ):
            raise ValueError('视觉超时和追击请求频率必须为有限正数')

        self.enemy_tracked = False
        self.enemy_data_timestamp = None

        # 视觉丢失后递增代次，防止恢复时接收丢失前发出的请求结果
        self.vision_generation = 0

        # 同一个导航器同一时间只接受一项 Chase
        self.chase_lock = threading.Lock()
        self.chase_active = False
        # 跨追击任务保留未结束的计算请求，只用于确认收尾
        self.target_request = None

        self.navigator = BasicNavigator()

        self.action_callback_group = ReentrantCallbackGroup()

        self.action_server = ActionServer(
            self,
            Chase,
            '/tracker/chase',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_callback_group,
        )

        self.get_target_client = ActionClient(
            self,
            GetTargetPose,
            '/tracker/get_target_pose',
            callback_group=self.action_callback_group,
        )

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
            enemy_qos,
        )

        self.get_logger().info(
            'chase_client 已启动：追击期间按需请求目标点'
        )

    def goal_callback(self, _request):
        # 共用一个 Nav2 导航器，同一时间只允许一项追击任务
        with self.chase_lock:
            if self.chase_active:
                return GoalResponse.REJECT

            self.chase_active = True

        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self.get_logger().info('收到停止追击请求')
        return CancelResponse.ACCEPT

    def enemy_callback(self, msg):
        now = self.get_clock().now()
        interrupted = not msg.tracked or not self.enemy_tracked

        if self.enemy_data_timestamp is not None:
            age = (
                now - self.enemy_data_timestamp
            ).nanoseconds / 1e9

            interrupted = (
                interrupted
                or not 0.0 <= age <= self.vision_timeout
            )

        # 明确丢失或断流后恢复，都不能接受之前发出的目标请求
        if interrupted:
            self.vision_generation += 1

        self.enemy_tracked = bool(msg.tracked)
        self.enemy_data_timestamp = now

    def is_enemy_tracked(self):
        if (
            self.enemy_data_timestamp is None
            or not self.enemy_tracked
        ):
            return False

        elapsed = (
            self.get_clock().now() - self.enemy_data_timestamp
        ).nanoseconds / 1e9

        return 0.0 <= elapsed <= self.vision_timeout

    def _start_target_request(self, offset_distance):
        goal = GetTargetPose.Goal()
        goal.offset_distance = float(offset_distance)

        # 先记录视觉代次，防止接收丢失前发出的请求结果
        generation = self.vision_generation

        # 只保留请求句柄，不缓存可供下一次追击使用的目标点
        request = {
            'sent_at': time.monotonic(),
            'accepted_at': None,
            'send_future': self.get_target_client.send_goal_async(goal),
            'handle': None,
            'result_future': None,
            'abandoned': False,
            'cancel_sent': False,
            'generation': generation,
        }
        self.target_request = request
        return request

    @staticmethod
    def _discard_target_request(request):
        if request is None or request['abandoned']:
            return

        request['abandoned'] = True

        def cancel_handle(handle):
            if (
                handle is not None
                and handle.accepted
                and not request['cancel_sent']
            ):
                request['cancel_sent'] = True
                handle.cancel_goal_async()

        try:
            if request['handle'] is not None:
                future = request['result_future']

                if future is None or not future.done():
                    cancel_handle(request['handle'])

            else:
                # GoalHandle 尚未返回时，收到迟到答复后仍请求取消
                # 此回调不向任何 Chase 回写目标点
                def cancel_late(future):
                    try:
                        cancel_handle(future.result())
                    except Exception:
                        pass

                request['send_future'].add_done_callback(cancel_late)

        except Exception:
            # 即使取消答复无法确认，也不再使用这个请求的结果
            pass

    @staticmethod
    def _is_target_request_stopped(request):
        # 取消答复不代表计算结束，必须等到最终结果
        if request is None:
            return True

        try:
            if not request['send_future'].done():
                return False

            handle = request['handle']
            if handle is None:
                handle = request['send_future'].result()
                request['handle'] = handle

            if handle is None or not handle.accepted:
                return True

            if request['result_future'] is None:
                request['result_future'] = handle.get_result_async()

            future = request['result_future']
            if not future.done():
                return False

            return future.result().status in (
                GoalStatus.STATUS_SUCCEEDED,
                GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_ABORTED,
            )
        except Exception:
            # 无法确认终态时保留请求，不启动下一次计算
            return False

    def _poll_target_request(self, request):
        """返回等待、成功或失败；不阻塞等待目标点回调。"""
        if request['abandoned']:
            return 'failed', '目标点请求已作废'

        try:
            now = time.monotonic()

            if request['handle'] is None:
                # 保留原来的请求接受超时预算
                if now - request['sent_at'] >= 3.0:
                    self._discard_target_request(request)
                    return 'failed', 'GetTargetPose 接受请求超时'

                if not request['send_future'].done():
                    return 'waiting', None

                handle = request['send_future'].result()

                if handle is None or not handle.accepted:
                    return 'failed', 'GetTargetPose 请求被拒绝'

                request['handle'] = handle
                request['accepted_at'] = now
                request['result_future'] = handle.get_result_async()

            future = request['result_future']

            # 保留原来的结果等待超时预算
            if now - request['accepted_at'] >= 5.0:
                self._discard_target_request(request)
                return 'failed', 'GetTargetPose 结果超时'

            if not future.done():
                return 'waiting', None

            wrapped = future.result()
            result = wrapped.result

            if (
                wrapped.status != GoalStatus.STATUS_SUCCEEDED
                or not result.success
                or not result.has_target
            ):
                return (
                    'failed',
                    result.message or '本次没有有效目标点',
                )

            pose = result.target_pose
            position = pose.pose.position

            # 只接受地图坐标系下的有限坐标
            if (
                pose.header.frame_id != 'map'
                or not all(
                    math.isfinite(value)
                    for value in (
                        position.x,
                        position.y,
                        position.z,
                    )
                )
            ):
                return 'failed', '目标点坐标系或数值无效'

            return 'ready', pose

        except Exception as error:
            self._discard_target_request(request)
            return 'failed', f'获取目标点异常: {error}'

    def _has_pose_changed(self, pose1, pose2, threshold=0.1):
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y

        return math.hypot(dx, dy) > threshold

    def execute_callback(self, goal_handle):
        # 上次遗留的请求只等待收尾，不能再读取其目标点
        request = self.target_request
        last_sent_pose = None

        # stopping 非空表示已请求取消旧导航，正在等待终态
        stopping = None
        failure = None

        next_request_at = 0.0
        server_deadline = time.monotonic() + 3.0
        seen_generation = self.vision_generation
        feedback_state = None

        interval = 1.0 / self.chase_rate

        def discard_request():
            nonlocal request

            # 作废后仍保留请求，确认计算结束后才能清理
            self._discard_target_request(request)

        def wait_for_new_target():
            nonlocal stopping, next_request_at

            # 本次取点失败，只作废旧点并停止旧导航，不结束追击任务
            discard_request()
            if last_sent_pose is not None and stopping is None:
                self.navigator.cancelTask()
                stopping = '目标失效'
            next_request_at = time.monotonic() + interval

        def feedback(message, chasing=False):
            nonlocal feedback_state

            # 相同状态不重复发送 Feedback
            if feedback_state != (message, chasing):
                item = Chase.Feedback()
                item.chasing = chasing
                item.status = message
                goal_handle.publish_feedback(item)

                feedback_state = (message, chasing)

        try:
            offset = float(goal_handle.request.offset_distance)

            if not math.isfinite(offset):
                failure = '追击偏移距离必须为有限数'
            elif offset <= 0.0:
                # 交给目标计算端使用唯一的默认偏移距离
                offset = 0.0

            if not self.is_enemy_tracked():
                failure = '敌人未被跟踪或消息已超时'

            while rclpy.ok():
                try:
                    # 等待目标点期间也检查取消和视觉状态
                    canceled = bool(goal_handle.is_cancel_requested)
                    tracked = self.is_enemy_tracked()

                    changed = (
                        seen_generation != self.vision_generation
                    )

                    if changed:
                        seen_generation = self.vision_generation

                    # 取消、失败或视觉失效后，立即作废旧目标请求
                    if canceled or failure or not tracked or changed:
                        discard_request()
                        next_request_at = 0.0

                        # 同一轮停止只请求取消一次
                        if (
                            last_sent_pose is not None
                            and stopping is None
                        ):
                            self.navigator.cancelTask()
                            stopping = (
                                '退出'
                                if canceled or failure
                                else '目标失效'
                            )

                    # 旧计算返回终态后，才允许释放本次请求
                    if (
                        request is not None
                        and request['abandoned']
                        and self._is_target_request_stopped(request)
                    ):
                        request = None
                        self.target_request = None

                    # 已发送目标必须等 Nav2 终态后才能清理记录
                    if (
                        last_sent_pose is not None
                        and self.navigator.isTaskComplete()
                    ):
                        nav_result = self.navigator.getResult()

                        # 保留已经确认的导航失败，不被新目标覆盖
                        if (
                            not canceled
                            and nav_result in (
                                TaskResult.FAILED,
                                TaskResult.UNKNOWN,
                            )
                        ):
                            failure = (
                                f'追击导航未成功: {nav_result}'
                            )
                            discard_request()

                        last_sent_pose = None
                        stopping = None

                    # 先确认旧导航结束；旧计算留给下一轮继续收尾
                    # 计算端断联时，不阻塞回血等其他导航任务
                    if canceled or failure:
                        if last_sent_pose is not None:
                            feedback('等待旧导航停止')
                            time.sleep(0.02)
                            continue

                        result = Chase.Result()
                        result.success = False
                        result.message = (
                            '追击已取消'
                            if canceled
                            else failure
                        )

                        if canceled:
                            goal_handle.canceled()
                        else:
                            goal_handle.abort()

                        return result

                    if stopping is not None:
                        # 旧导航未释放时，不计算或发送新目标
                        feedback('等待旧导航停止')
                        time.sleep(0.02)
                        continue

                    if not tracked or changed:
                        # 行为树负责丢失宽限和最终任务切换
                        # 本服务等待恢复，期间不沿用旧目标运动
                        feedback('等待有效视觉目标')
                        time.sleep(0.02)
                        continue

                    if request is not None and request['abandoned']:
                        # 视觉已恢复，也不能抢在旧计算结束前发出新请求
                        feedback('等待旧目标计算结束')
                        time.sleep(0.02)
                        continue

                    # 同一时间只保留一个目标点请求
                    if (
                        request is None
                        and time.monotonic() >= next_request_at
                    ):
                        if not self.get_target_client.server_is_ready():
                            if server_deadline is not None:
                                # 保留旧版启动时等待服务就绪的3秒限制
                                if time.monotonic() >= server_deadline:
                                    failure = 'target_pose_server 未就绪'
                            else:
                                # 已在追击期间断联，先停旧导航，再等待服务恢复
                                wait_for_new_target()
                                feedback('等待目标计算服务恢复')
                            time.sleep(0.02)
                            continue

                        server_deadline = None
                        try:
                            request = self._start_target_request(offset)
                        except Exception:
                            wait_for_new_target()
                            feedback('目标点请求失败，等待重试')
                            time.sleep(0.02)
                            continue

                    if request is not None:
                        state, value = self._poll_target_request(
                            request
                        )

                        if state == 'failed':
                            # 取点失败不等于Nav2导航失败，清理后等待新点
                            wait_for_new_target()
                            feedback('等待重新获取目标点')
                            time.sleep(0.02)
                            continue

                        if state == 'ready':
                            # 结果返回到实际发送之间，再检查一次有效性
                            valid = (
                                request['generation']
                                == self.vision_generation
                                and self.is_enemy_tracked()
                                and not goal_handle.is_cancel_requested
                            )

                            request = None
                            self.target_request = None
                            next_request_at = (
                                time.monotonic() + interval
                            )

                            if valid:
                                if last_sent_pose is None:
                                    # 先保存记录，发送异常时也能进入停止流程
                                    last_sent_pose = value

                                    if not self.navigator.goToPose(value):
                                        last_sent_pose = None
                                        failure = 'Nav2 拒绝追击目标'
                                    else:
                                        feedback('正在追击', True)

                                elif self._has_pose_changed(
                                    last_sent_pose,
                                    value,
                                ):
                                    # 新点只用来决定换点，不保存为待发送旧缓存
                                    # 旧导航结束后重新请求当前目标
                                    self.navigator.cancelTask()
                                    stopping = '更新目标'
                                    next_request_at = 0.0

                    time.sleep(0.02)

                except Exception as error:
                    # 接口异常也要经过导航取消确认，不能直接释放控制权
                    failure = f'追击执行异常: {error}'
                    discard_request()
                    time.sleep(0.02)

        finally:
            discard_request()

            # 进程关闭时请求停止，但不冒充已经完成导航交接
            if last_sent_pose is not None and stopping is None:
                self.navigator.cancelTask()

            with self.chase_lock:
                self.chase_active = False


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

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()