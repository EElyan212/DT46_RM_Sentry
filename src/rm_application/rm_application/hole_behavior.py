

import time

import rclpy
from rclpy.node import Node
from rm_interfaces.msg import HoleStatus
from std_msgs.msg import Bool



class HoleBehavior(Node):
    STATE_IDLE = "IDLE"
    STATE_WAIT_GIMBAL = "WAIT_GIMBAL"
    STATE_ENTERING = "ENTERING"
    STATE_INSIDE = "INSIDE"
    STATE_EXIT_TRAVEL = "EXIT_TRAVEL"
    STATE_WAIT_EXIT_GIMBAL = "WAIT_EXIT_GIMBAL"
    STATE_DONE = "DONE"

    def __init__(self):
        super().__init__("hole_behavior")

        self.declare_parameter("hole_status_topic", "/hole/status")
        self.declare_parameter("hole_block_topic", "/hole/block")
        self.declare_parameter(
            "gimbal_complete_topic",
            "/gimbal/action_complete",
        )
        self.declare_parameter(
            "gimbal_exit_complete_topic",
            "/gimbal/exit_action_complete",
        )
        self.declare_parameter(
            "axis_align_enable_topic",
            "/axis_align/enable",
        )
        self.declare_parameter("post_exit_distance", 0.5)
        self.declare_parameter("status_timeout", 1.0)
        self.declare_parameter("block_on_missing_status", True)
        self.declare_parameter("publish_rate", 10.0)
        # 调试用：无需手动发布云台完成话题，等待一段时间后自动放行
        self.declare_parameter("auto_gimbal_complete", False)
        self.declare_parameter("auto_exit_gimbal_complete", False)
        self.declare_parameter("gimbal_auto_delay", 1.0)

        self.hole_status_topic = self.get_parameter(
            "hole_status_topic"
        ).value
        self.hole_block_topic = self.get_parameter(
            "hole_block_topic"
        ).value
        self.gimbal_complete_topic = self.get_parameter(
            "gimbal_complete_topic"
        ).value
        self.gimbal_exit_complete_topic = self.get_parameter(
            "gimbal_exit_complete_topic"
        ).value
        self.axis_align_enable_topic = self.get_parameter(
            "axis_align_enable_topic"
        ).value
        self.post_exit_distance = float(
            self.get_parameter("post_exit_distance").value
        )
        self.status_timeout = float(
            self.get_parameter("status_timeout").value
        )
        self.block_on_missing_status = bool(
            self.get_parameter("block_on_missing_status").value
        )
        self.publish_rate = float(
            self.get_parameter("publish_rate").value
        )
        self.auto_gimbal_complete = bool(
            self.get_parameter("auto_gimbal_complete").value
        )
        self.auto_exit_gimbal_complete = bool(
            self.get_parameter("auto_exit_gimbal_complete").value
        )
        self.gimbal_auto_delay = max(
            float(self.get_parameter("gimbal_auto_delay").value), 0.0
        )

        self.state = self.STATE_IDLE
        self.block = False
        self.latest_status = None
        self.latest_status_time = None
        self.gimbal_complete_event = False
        self.gimbal_exit_complete_event = False
        self.gimbal_wait_since = None
        self.exit_gimbal_wait_since = None
        self.axis_align_request = False

        self.status_subscription = self.create_subscription(
            HoleStatus,
            self.hole_status_topic,
            self.status_callback,
            10,
        )
        self.gimbal_complete_subscription = self.create_subscription(
            Bool,
            self.gimbal_complete_topic,
            self.gimbal_complete_callback,
            10,
        )
        self.gimbal_exit_complete_subscription = self.create_subscription(
            Bool,
            self.gimbal_exit_complete_topic,
            self.gimbal_exit_complete_callback,
            10,
        )
        self.block_publisher = self.create_publisher(
            Bool,
            self.hole_block_topic,
            10,
        )
        self.axis_align_enable_publisher = self.create_publisher(
            Bool,
            self.axis_align_enable_topic,
            10,
        )

        period = 1.0 / max(self.publish_rate, 0.1)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info("hole_behavior 已启动")

    def status_callback(self, msg):
        self.latest_status = msg
        self.latest_status_time = self.get_clock().now()

    def gimbal_complete_callback(self, msg):
        if not msg.data:
            return

        if self.state == self.STATE_WAIT_GIMBAL:
            self.gimbal_complete_event = True
            self.get_logger().info("收到云台动作完成信号")

    def gimbal_exit_complete_callback(self, msg):
        if not msg.data:
            return

        if self.state == self.STATE_WAIT_EXIT_GIMBAL:
            self.gimbal_exit_complete_event = True
            self.get_logger().info("收到云台出洞动作完成信号")

    def status_is_fresh(self):
        if self.latest_status is None or self.latest_status_time is None:
            return False

        age = (
            self.get_clock().now() - self.latest_status_time
        ).nanoseconds / 1e9
        return age <= self.status_timeout

    """
    set_block(True)：打印“已关闭底盘速度输出”
    set_block(False)：打印“已恢复底盘速度输出”
    publish_block()：把当前阻塞状态发布到 /hole/block
    """
    def set_block(self, blocked):
        blocked = bool(blocked)

        if blocked == self.block:
            return

        if blocked:
            self.get_logger().info("已关闭底盘速度输出")
        else:
            self.get_logger().info("已恢复底盘速度输出")

        self.block = blocked

    def publish_block(self):
        self.block_publisher.publish(Bool(data=self.block))

    """
    核心状态机
    timer_callback() 只负责分发状态，每个 Handler 负责一个状态下的判断，并返回当前周期是否应该阻塞底盘。
    返回 True：/hole/block=true，串口线速度置零
    返回 False：允许底盘继续移动
    """
    def timer_callback(self):
        if not self.status_is_fresh():
            blocked = self._handle_stale_status()
        else:
            status = self.latest_status

            if not status.enabled:
                blocked = self._handle_disabled()
            elif self.state == self.STATE_IDLE:
                blocked = self._handle_idle(status)
            elif self.state == self.STATE_WAIT_GIMBAL:
                blocked = self._handle_wait_gimbal(status)
            elif self.state == self.STATE_ENTERING:
                blocked = self._handle_entering(status)
            elif self.state == self.STATE_INSIDE:
                blocked = self._handle_inside(status)
            elif self.state == self.STATE_EXIT_TRAVEL:
                blocked = self._handle_exit_travel(status)
            elif self.state == self.STATE_WAIT_EXIT_GIMBAL:
                blocked = self._handle_wait_exit_gimbal(status)
            elif self.state == self.STATE_DONE:
                blocked = self._handle_done(status)
            else:
                blocked = False

        self.set_block(blocked)
        self.publish_block()
        self.axis_align_enable_publisher.publish(
            Bool(data=self.axis_align_request)
        )
    # 处理 /hole/status 超时或尚未收到状态的情况
    def _handle_stale_status(self):
        return (
            self.state in (
                self.STATE_WAIT_GIMBAL,
                self.STATE_WAIT_EXIT_GIMBAL,
            )
            and self.block_on_missing_status
        )
    # 处理 status.enabled=false，即过洞检测功能关闭或矩形参数非
    def _handle_disabled(self):
        self.state = self.STATE_IDLE
        self.axis_align_request = False
        return False
    # 待机状态，判断是否开始入洞流程，
    def _handle_idle(self, status):
        # 如果 status.inside=true： 表示机器人已经在洞里，直接进入洞内状态
        if status.inside:
            self.state = self.STATE_INSIDE
            self.get_logger().info("已进入洞口")
            return False

        # 如果 status.approaching=true：表示接近洞口，开始阻塞并等待入洞云台完成
        if status.approaching:
            self.state = self.STATE_WAIT_GIMBAL
            self.gimbal_complete_event = False
            self.gimbal_wait_since = time.monotonic()
            self.get_logger().info(
                "接近洞口，等待云台动作完成"
            )
            return True

        return False
    # 入洞前等待云台动作完成，
    def _handle_wait_gimbal(self, status):
        if status.inside:
            self.state = self.STATE_INSIDE
            self.get_logger().warn(
                "未收到云台完成信号但已进入洞口"
            )
            self.get_logger().info("已进入洞口")
            return False

        if (
            self.auto_gimbal_complete
            and not self.gimbal_complete_event
            and self.gimbal_wait_since is not None
            and time.monotonic() - self.gimbal_wait_since
            >= self.gimbal_auto_delay
        ):
            self.gimbal_complete_event = True
            self.get_logger().info(
                "调试模式：自动完成入洞云台动作"
            )

        if self.gimbal_complete_event:
            self.state = self.STATE_ENTERING
            self.axis_align_request = True
            self.get_logger().info(
                "云台动作完成，允许进入洞口"
            )
            return False

        # 只有拿到有效的新路径、且确认它不再经过洞口时才取消等待；
        # 路径超时（path_valid=False）时保持等待，避免来回抖动。
        if (
            status.path_valid
            and not status.path_intersects
            and not status.approaching
        ):
            self.state = self.STATE_IDLE
            self.get_logger().info(
                "路径已不再经过洞口，取消等待"
            )
            return False

        return True
    # 入洞云台已经完成，等待机器人真正进入洞内
    def _handle_entering(self, status):
        if status.inside:
            self.state = self.STATE_INSIDE
            self.get_logger().info("已进入洞口")
        elif (
            status.path_valid
            and not status.path_intersects
            and not status.approaching
        ):
            self.state = self.STATE_IDLE
            self.axis_align_request = False

        return False

    # 机器人已经在洞内，等待离开洞口
    def _handle_inside(self, status):
        if not status.inside:
            self.state = self.STATE_EXIT_TRAVEL
            self.axis_align_request = False
            self.get_logger().info("已出洞")

        return False
    # 机器人已经出洞，累计移动距离，判断是否触发第二段云台动作
    def _handle_exit_travel(self, status):
        if status.inside:
            self.state = self.STATE_INSIDE
            return False

        if status.distance_since_exit >= self.post_exit_distance:
            self.state = self.STATE_WAIT_EXIT_GIMBAL
            self.gimbal_exit_complete_event = False
            self.exit_gimbal_wait_since = time.monotonic()
            self.get_logger().info(
                f"已离开洞口 {status.distance_since_exit:.2f}m，"
                f"等待云台出洞动作完成"
            )
            return True

        return False
    # 出洞后等待专用云台完成信号
    def _handle_wait_exit_gimbal(self, status):
        if (
            self.auto_exit_gimbal_complete
            and not self.gimbal_exit_complete_event
            and self.exit_gimbal_wait_since is not None
            and time.monotonic() - self.exit_gimbal_wait_since
            >= self.gimbal_auto_delay
        ):
            self.gimbal_exit_complete_event = True
            self.get_logger().info(
                "调试模式：自动完成出洞云台动作"
            )

        if self.gimbal_exit_complete_event:
            self.state = self.STATE_DONE
            self.get_logger().info(
                "出洞云台动作完成，允许继续移动"
            )
            return False

        if status.inside:
            self.state = self.STATE_INSIDE
            return False

        return True
    # 出洞流程完成后的收尾，回到待机状态
    def _handle_done(self, status):
        self.state = self.STATE_IDLE
        return False


def main(args=None):
    rclpy.init(args=args)
    node = HoleBehavior()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.set_block(False)
                node.publish_block()
                node.axis_align_enable_publisher.publish(
                    Bool(data=False)
                )
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
