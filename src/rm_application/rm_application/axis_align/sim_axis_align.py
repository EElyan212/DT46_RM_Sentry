import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool

from rm_application.axis_align.axis_align_node import AxisAlignNode
from rm_application.axis_align.centerline_math import (
    signed_distance_to_line,
)
from rm_application.axis_align.fake_base_node import FakeBaseNode

CMD_TOPIC = "/sim/cmd_vel"
ALIGNED_TOPIC = "/sim/aligned"
POSE_TOPIC = "/sim/base_pose"


def main(args=None):
    rclpy.init(args=args)

    sim = FakeBaseNode(
        parameter_overrides=[
            Parameter("cmd_vel_topic", value=CMD_TOPIC),
            Parameter("pose_topic", value=POSE_TOPIC),
        ]
    )
    align = AxisAlignNode(
        parameter_overrides=[
            Parameter("cmd_vel_topic", value=CMD_TOPIC),
            Parameter("aligned_topic", value=ALIGNED_TOPIC),
        ]
    )

    aligned_values = []
    commands = []

    aligned_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    sim.create_subscription(
        Bool,
        ALIGNED_TOPIC,
        lambda msg: aligned_values.append(msg.data),
        aligned_qos,
    )
    sim.create_subscription(
        Twist,
        CMD_TOPIC,
        commands.append,
        10,
    )

    executor = MultiThreadedExecutor()
    executor.add_node(sim)
    executor.add_node(align)

    start_time = time.monotonic()
    deadline = start_time + 15.0
    finished = False

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if aligned_values and aligned_values[-1]:
                finished = True
                break
    finally:
        elapsed = time.monotonic() - start_time
        x, y, yaw = sim.get_pose()
        centerline_point = align.centerline_point
        centerline_yaw = align.centerline_yaw
        deadband_lat = align.deadband_lat
        speed_limit = align.max_speed

        executor.remove_node(align)
        executor.remove_node(sim)
        align.destroy_node()
        sim.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

    error = signed_distance_to_line(
        x,
        y,
        centerline_point,
        centerline_yaw,
    )
    max_speed = max(
        (math.hypot(cmd.linear.x, cmd.linear.y) for cmd in commands),
        default=0.0,
    )
    max_wz = max(
        (abs(cmd.angular.z) for cmd in commands),
        default=0.0,
    )

    print(f"finished={finished}")
    print(
        f"centerline=({centerline_point[0]:.4f}, "
        f"{centerline_point[1]:.4f}, "
        f"{centerline_yaw:.4f})"
    )
    print(f"deadband={deadband_lat:.4f}m")
    print(f"speed_limit={speed_limit:.4f}m/s")
    print(f"elapsed={elapsed:.3f}s")
    print(f"pose=({x:.4f}, {y:.4f}, {yaw:.4f})")
    print(f"lateral_error={error:.4f}m")
    print(f"max_speed={max_speed:.4f}m/s")
    print(f"max_angular_z={max_wz:.6f}rad/s")
    print(f"samples={len(commands)}")

    assert finished, "alignment did not finish before timeout"
    assert abs(error) <= deadband_lat + 0.005, (
        "final lateral error exceeds deadband"
    )
    assert max_speed <= speed_limit + 0.005, (
        "commanded speed exceeds limit"
    )
    assert max_wz <= 1e-9, "angular command must remain zero"
    print("PASS")


if __name__ == "__main__":
    main()
