# 入洞时手动放行 ros2 topic pub --once /gimbal/action_complete std_msgs/msg/Bool "{data: true}"
# 出洞时手动放行 ros2 topic pub --once /gimbal/exit_action_complete std_msgs/msg/Bool "{data: true}"

import math

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from rclpy.time import Time
from rm_interfaces.msg import HoleStatus
from tf2_ros import Buffer, TransformListener, TransformException

from rm_application.hole_geometry import (
    cumulative_lengths,
    first_entry_arc_length,
    point_in_rect,
    project_robot_on_path,
)


class HoleDetector(Node):
    def __init__(self):
        super().__init__("hole_detector")

        self.declare_parameter("enabled", True)
        self.declare_parameter("hole_x_min", 0.0)
        self.declare_parameter("hole_x_max", 0.0)
        self.declare_parameter("hole_y_min", 0.0)
        self.declare_parameter("hole_y_max", 0.0)
        self.declare_parameter("pre_stop_distance", 0.8)   # 到洞口多少距离停下
        self.declare_parameter("path_timeout", 2.0)        # 在洞口停下的时间
        self.declare_parameter("inside_debounce_time", 0.5)  # 判定为进出洞状态需要持续满足条件的时间
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("status_topic", "/hole/status")

        self.enabled = bool(self.get_parameter("enabled").value)
        self.x_min = float(self.get_parameter("hole_x_min").value)
        self.x_max = float(self.get_parameter("hole_x_max").value)
        self.y_min = float(self.get_parameter("hole_y_min").value)
        self.y_max = float(self.get_parameter("hole_y_max").value)
        self.pre_stop_distance = float(
            self.get_parameter("pre_stop_distance").value
        )
        self.path_timeout = float(self.get_parameter("path_timeout").value)
        self.inside_debounce_time = float(
            self.get_parameter("inside_debounce_time").value
        )
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.plan_topic = self.get_parameter("plan_topic").value
        self.status_topic = self.get_parameter("status_topic").value

        self.rect = (self.x_min, self.x_max, self.y_min, self.y_max)
        self.rect_valid = (
            math.isfinite(self.x_min)
            and math.isfinite(self.x_max)
            and math.isfinite(self.y_min)
            and math.isfinite(self.y_max)
            and self.x_min < self.x_max
            and self.y_min < self.y_max
        )

        if not self.rect_valid:
            self.get_logger().warn(
                "洞区域参数无效，hole_detector 将保持禁用状态"
            )
            self.enabled = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_path = None
        self.cumulative = []
        self.entry_arc_length = None
        self.latest_path_time = None
        self.last_progress_index = 0
        self.was_inside = False
        self.exit_distance = None
        self.last_state = None
        self.inside_state = False
        self.inside_candidate = None
        self.inside_candidate_since = None
        # 出洞后累计位移用 TF 位移计算，避免路径重规划导致进度原点跳变
        self.last_robot_xy = None

        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.path_subscription = self.create_subscription(
            Path,
            self.plan_topic,
            self.path_callback,
            path_qos,
        )
        self.status_publisher = self.create_publisher(
            HoleStatus,
            self.status_topic,
            10,
        )

        period = 1.0 / max(self.publish_rate, 0.1)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f"hole_detector 已启动，区域=({self.x_min:.2f}, "
            f"{self.x_max:.2f}, {self.y_min:.2f}, {self.y_max:.2f})，"
            f"提前停止距离={self.pre_stop_distance:.2f}m"
        )

    def path_callback(self, msg):
        if not self.enabled:
            return

        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f"忽略非 {self.map_frame} 坐标系的全局路径: "
                f"{msg.header.frame_id}",
                throttle_duration_sec=5.0,
            )
            return

        points = []
        for pose in msg.poses:
            x = float(pose.pose.position.x)
            y = float(pose.pose.position.y)
            if math.isfinite(x) and math.isfinite(y):
                points.append((x, y))

        if not points:
            self.latest_path = None
            self.cumulative = []
            self.entry_arc_length = None
            self.latest_path_time = None
            return

        self.latest_path = points
        self.cumulative = cumulative_lengths(points)
        self.entry_arc_length = first_entry_arc_length(
            points,
            self.cumulative,
            self.rect,
        )
        self.latest_path_time = self.get_clock().now()
        self.last_progress_index = 0

    def timer_callback(self):
        status = HoleStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self.map_frame
        status.enabled = self.enabled
        status.path_valid = False
        status.path_intersects = False
        status.approaching = False
        status.inside = False
        status.distance_to_entry = -1.0
        status.distance_since_exit = -1.0

        if not self.enabled:
            self.status_publisher.publish(status)
            return

        robot_position = self.get_robot_position()
        if robot_position is None:
            self.get_logger().warn(
                f"无法获取 {self.map_frame}->{self.robot_frame} 变换，"
                f"请检查 TF/里程计是否发布",
                throttle_duration_sec=5.0,
            )
            self.finalize_status(status, 0.0)
            return

        # 是否在洞内只依赖 TF 与矩形，不依赖全局路径，
        # 这样静止放在洞里也能检测到。
        robot_x, robot_y = robot_position
        raw_inside = point_in_rect(
            robot_x,
            robot_y,
            self.x_min,
            self.x_max,
            self.y_min,
            self.y_max,
        )
        status.inside = self.update_inside(raw_inside)
        if status.inside:
            status.path_intersects = True
            status.distance_to_entry = 0.0

        # 用相邻两帧 TF 位移累计“出洞后行驶距离”，
        # 不依赖 /plan 进度，重规划/路径超时都不会把它清零。
        moved = 0.0
        if self.last_robot_xy is not None:
            step = math.hypot(
                robot_x - self.last_robot_xy[0],
                robot_y - self.last_robot_xy[1],
            )
            if step <= 0.5:  # 过滤定位跳变
                moved = step
        self.last_robot_xy = (robot_x, robot_y)

        if self.latest_path is None or self.latest_path_time is None:
            self.get_logger().info(
                f"尚未收到全局路径 {self.plan_topic}，"
                f"暂只判断是否在洞内",
                throttle_duration_sec=5.0,
            )
            self.finalize_status(status, moved)
            return

        path_age = (
            self.get_clock().now() - self.latest_path_time
        ).nanoseconds / 1e9
        if path_age > self.path_timeout:
            self.get_logger().warn(
                f"全局路径已超时 {path_age:.1f}s（阈值 "
                f"{self.path_timeout:.1f}s），等待新路径，"
                f"暂只判断是否在洞内",
                throttle_duration_sec=5.0,
            )
            self.finalize_status(status, moved)
            return

        status.path_valid = True

        robot_progress = None
        projection = project_robot_on_path(
            self.latest_path,
            self.cumulative,
            robot_position,
            hint_index=self.last_progress_index,
        )
        if projection is not None:
            robot_progress, index, _ = projection
            self.last_progress_index = index

        if self.entry_arc_length is None:
            self.get_logger().warn(
                f"全局路径未与洞矩形 "
                f"({self.x_min:.2f},{self.x_max:.2f},"
                f"{self.y_min:.2f},{self.y_max:.2f}) 相交，",
                throttle_duration_sec=2.0,
            )
        elif not status.inside and robot_progress is not None:
            distance = self.entry_arc_length - robot_progress
            distance = max(0.0, distance)
            status.path_intersects = True
            status.distance_to_entry = distance
            status.approaching = (
                distance <= self.pre_stop_distance
            )
            self.get_logger().info(
                f"入口弧长={self.entry_arc_length:.2f}m 当前位置弧长="
                f"{robot_progress:.2f}m 距入口={distance:.2f}m "
                f"提前停止距离={self.pre_stop_distance:.2f}m "
                f"approaching={status.approaching}",
                throttle_duration_sec=2.0,
            )

        self.finalize_status(status, moved)

    def finalize_status(self, status, moved):
        if status.inside:
            self.exit_distance = None
            self.was_inside = True
        else:
            if self.was_inside:
                self.exit_distance = 0.0

            if self.exit_distance is not None:
                self.exit_distance += moved
                status.distance_since_exit = self.exit_distance

            self.was_inside = False

        self.log_transition(status)
        self.status_publisher.publish(status)

    def update_inside(self, raw_inside):
        # 只有进出洞状态持续时间满足条件才会更新状态，避免洞口边界处定位抖动导致状态反复跳变。
        if raw_inside == self.inside_state:
            self.inside_candidate = None
            self.inside_candidate_since = None
            return self.inside_state

        now = self.get_clock().now()
        if (
            self.inside_candidate != raw_inside
            or self.inside_candidate_since is None
        ):
            self.inside_candidate = raw_inside
            self.inside_candidate_since = now
            return self.inside_state

        elapsed = (
            now - self.inside_candidate_since
        ).nanoseconds / 1e9
        if elapsed >= self.inside_debounce_time:
            self.inside_state = raw_inside
            self.inside_candidate = None
            self.inside_candidate_since = None

        return self.inside_state

    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                Time(),
            )
        except TransformException:
            return None

        translation = transform.transform.translation
        return float(translation.x), float(translation.y)

    def log_transition(self, status):
        current = (
            bool(status.path_valid),
            bool(status.path_intersects),
            bool(status.approaching),
            bool(status.inside),
        )

        previous = self.last_state
        self.last_state = current

        if previous is None:
            if current[3]:
                self.get_logger().info("机器人已在洞区域内")
            elif current[2]:
                self.get_logger().info(
                    f"全局路径经过洞，距离入口 "
                    f"{status.distance_to_entry:.2f}m"
                )
            elif current[1]:
                self.get_logger().info("全局路径经过洞区域")
            return

        if previous == current:
            return

        if current[3] and not previous[3]:
            self.get_logger().info("机器人已进入洞区域")
        elif previous[3] and not current[3]:
            self.get_logger().info("机器人已离开洞区域")
        elif current[2] and not previous[2]:
            self.get_logger().info(
                f"全局路径经过洞，距离入口 "
                f"{status.distance_to_entry:.2f}m"
            )
        elif current[1] and not previous[1]:
            self.get_logger().info("全局路径经过洞区域")
        elif previous[1] and not current[1] and not current[3]:
            self.get_logger().info("全局路径已不再经过洞区域")


def main(args=None):
    rclpy.init(args=args)
    node = HoleDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
