import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Duration, Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from rm_application.axis_align.centerline_math import (
    lateral_velocity,
    map_velocity_to_base,
    quaternion_to_yaw,
    signed_distance_to_line,
    slew_limit,
)


class AxisAlignNode(Node):
    def __init__(self, **kwargs):
        super().__init__("axis_align_node", **kwargs)

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("aligned_topic", "/axis_align/aligned")

        self.declare_parameter("centerline_point", [0.0, 0.0])
        self.declare_parameter("centerline_yaw", 0.0)

        self.declare_parameter("deadband_lat", 0.02)
        self.declare_parameter("settle_time", 0.3)
        self.declare_parameter("kp_lat", 1.0)
        self.declare_parameter("min_speed", 0.02)
        self.declare_parameter("max_speed", 0.15)
        self.declare_parameter("max_accel", 0.5)
        self.declare_parameter("tf_timeout", 0.2)

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.control_rate = max(
            float(self.get_parameter("control_rate").value), 1.0
        )
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.aligned_topic = self.get_parameter("aligned_topic").value

        point = list(self.get_parameter("centerline_point").value)
        if len(point) != 2:
            raise ValueError("centerline_point must contain exactly [x, y]")

        self.centerline_point = (float(point[0]), float(point[1]))
        self.centerline_yaw = float(
            self.get_parameter("centerline_yaw").value
        )

        self.deadband_lat = max(
            float(self.get_parameter("deadband_lat").value), 0.0
        )
        self.settle_time = max(
            float(self.get_parameter("settle_time").value), 0.0
        )
        self.kp_lat = float(self.get_parameter("kp_lat").value)
        self.min_speed = max(
            float(self.get_parameter("min_speed").value), 0.0
        )
        self.max_speed = max(
            float(self.get_parameter("max_speed").value),
            self.min_speed,
        )
        self.max_accel = max(
            float(self.get_parameter("max_accel").value), 0.0
        )
        self.tf_timeout = max(
            float(self.get_parameter("tf_timeout").value), 0.0
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_publisher = self.create_publisher(
            Twist, self.cmd_vel_topic, 10
        )

        aligned_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.aligned_publisher = self.create_publisher(
            Bool, self.aligned_topic, aligned_qos
        )
        self.aligned_publisher.publish(Bool(data=False))

        self.last_control_time = time.monotonic()
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.inside_deadband_since = None
        self.aligned = False
        self.last_tf_warn_time = 0.0

        self.timer = self.create_timer(
            1.0 / self.control_rate, self.control_loop
        )

        self.get_logger().info(
            "axis_align_node started: "
            f"point=({self.centerline_point[0]:.3f}, "
            f"{self.centerline_point[1]:.3f}), "
            f"yaw={self.centerline_yaw:.3f} rad"
        )

    def control_loop(self):
        now = time.monotonic()
        dt = max(now - self.last_control_time, 1e-3)
        self.last_control_time = now

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.stop()
            self.set_aligned(False)
            return

        robot_x, robot_y, robot_yaw = robot_pose
        error = signed_distance_to_line(
            robot_x,
            robot_y,
            self.centerline_point,
            self.centerline_yaw,
        )

        if abs(error) <= self.deadband_lat:
            if self.inside_deadband_since is None:
                self.inside_deadband_since = now

            if now - self.inside_deadband_since >= self.settle_time:
                self.set_aligned(True)

            self.command_velocity(0.0, 0.0, dt)
            return

        self.inside_deadband_since = None
        self.set_aligned(False)

        v_n = lateral_velocity(
            error,
            self.kp_lat,
            self.deadband_lat,
            self.min_speed,
            self.max_speed,
        )

        nx = -math.sin(self.centerline_yaw)
        ny = math.cos(self.centerline_yaw)

        vx_map = v_n * nx
        vy_map = v_n * ny

        vx_base, vy_base = map_velocity_to_base(
            vx_map, vy_map, robot_yaw
        )

        self.command_velocity(vx_base, vy_base, dt)

    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                Duration(seconds=self.tf_timeout),
            )
        except Exception as exc:
            now = time.monotonic()
            if now - self.last_tf_warn_time > 2.0:
                self.get_logger().warn(f"TF lookup failed: {exc}")
                self.last_tf_warn_time = now
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        return (
            translation.x,
            translation.y,
            quaternion_to_yaw(
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ),
        )

    def command_velocity(self, target_vx, target_vy, dt):
        max_delta = self.max_accel * dt

        vx = slew_limit(self.last_vx, target_vx, max_delta)
        vy = slew_limit(self.last_vy, target_vy, max_delta)

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        self.cmd_publisher.publish(cmd)

        self.last_vx = vx
        self.last_vy = vy

    def set_aligned(self, aligned):
        if self.aligned == aligned:
            return

        self.aligned = aligned
        self.aligned_publisher.publish(Bool(data=aligned))

        if aligned:
            self.get_logger().info("centerline alignment reached")
        else:
            self.get_logger().info("centerline alignment lost")

    def stop(self):
        cmd = Twist()
        self.cmd_publisher.publish(cmd)
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.inside_deadband_since = None


def main(args=None):
    rclpy.init(args=args)
    node = AxisAlignNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
