import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FakeBaseNode(Node):
    def __init__(self, initial_pose=None, **kwargs):
        super().__init__("axis_align_fake_base", **kwargs)

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("pose_topic", "/sim/base_pose")
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.3)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("sim_rate", 50.0)
        self.declare_parameter("cmd_timeout", 0.5)

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.sim_rate = max(
            float(self.get_parameter("sim_rate").value), 1.0
        )
        self.cmd_timeout = max(
            float(self.get_parameter("cmd_timeout").value), 0.0
        )

        if initial_pose is None:
            self.x = float(self.get_parameter("initial_x").value)
            self.y = float(self.get_parameter("initial_y").value)
            self.yaw = float(self.get_parameter("initial_yaw").value)
        else:
            self.x, self.y, self.yaw = [
                float(value) for value in initial_pose
            ]

        self.lock = threading.Lock()
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        self.last_cmd_time = time.monotonic()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_publisher = self.create_publisher(
            PoseStamped, self.pose_topic, 10
        )
        self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        self.last_update_time = time.monotonic()
        self.timer = self.create_timer(
            1.0 / self.sim_rate, self.update
        )

        self.get_logger().info(
            "axis_align_fake_base started: "
            f"pose=({self.x:.3f}, {self.y:.3f}, {self.yaw:.3f}), "
            f"cmd_topic={self.cmd_vel_topic}"
        )

    def cmd_vel_callback(self, msg):
        with self.lock:
            self.cmd_vx = float(msg.linear.x)
            self.cmd_vy = float(msg.linear.y)
            self.cmd_wz = float(msg.angular.z)
            self.last_cmd_time = time.monotonic()

    def update(self):
        now = time.monotonic()
        dt = min(max(now - self.last_update_time, 0.0), 0.1)
        self.last_update_time = now

        with self.lock:
            if now - self.last_cmd_time > self.cmd_timeout:
                vx = 0.0
                vy = 0.0
                wz = 0.0
            else:
                vx = self.cmd_vx
                vy = self.cmd_vy
                wz = self.cmd_wz

            self.x += (
                math.cos(self.yaw) * vx - math.sin(self.yaw) * vy
            ) * dt
            self.y += (
                math.sin(self.yaw) * vx + math.cos(self.yaw) * vy
            ) * dt
            self.yaw += wz * dt

            x = self.x
            y = self.y
            yaw = self.yaw

        self.publish_pose(x, y, yaw)

    def publish_pose(self, x, y, yaw):
        stamp = self.get_clock().now().to_msg()

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(yaw * 0.5)
        transform.transform.rotation.w = math.cos(yaw * 0.5)
        self.tf_broadcaster.sendTransform(transform)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        self.pose_publisher.publish(pose)

    def get_pose(self):
        with self.lock:
            return self.x, self.y, self.yaw


def main(args=None):
    rclpy.init(args=args)
    node = FakeBaseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
