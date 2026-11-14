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
        self.last_robot_progress = None
        self.was_inside = False
        self.exit_distance = None
        self.last_state = None

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

        if self.latest_path is None or self.latest_path_time is None:
            self.status_publisher.publish(status)
            return

        path_age = (
            self.get_clock().now() - self.latest_path_time
        ).nanoseconds / 1e9
        if path_age > self.path_timeout:
            self.status_publisher.publish(status)
            return

        robot_position = self.get_robot_position()
        if robot_position is None:
            self.status_publisher.publish(status)
            return

        status.path_valid = True
        robot_x, robot_y = robot_position
        status.inside = point_in_rect(
            robot_x,
            robot_y,
            self.x_min,
            self.x_max,
            self.y_min,
            self.y_max,
        )

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

        progress_delta = 0.0
        if (
            robot_progress is not None
            and self.last_robot_progress is not None
        ):
            delta = robot_progress - self.last_robot_progress
            if 0.0 <= delta <= 1.0:
                progress_delta = delta

        if robot_progress is not None:
            self.last_robot_progress = robot_progress

        if status.inside:
            status.path_intersects = True
            status.approaching = False
            status.distance_to_entry = 0.0
        elif self.entry_arc_length is not None and robot_progress is not None:
            distance = self.entry_arc_length - robot_progress
            if distance >= -1e-6:
                distance = max(0.0, distance)
                status.path_intersects = True
                status.distance_to_entry = distance
                status.approaching = (
                    distance <= self.pre_stop_distance
                )

        if status.inside:
            self.exit_distance = None
            self.was_inside = True
        else:
            if self.was_inside:
                self.exit_distance = 0.0

            if self.exit_distance is not None:
                self.exit_distance += progress_delta
                status.distance_since_exit = self.exit_distance

            self.was_inside = False

        self.log_transition(status)
        self.status_publisher.publish(status)

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

        if previous is None or previous == current:
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
