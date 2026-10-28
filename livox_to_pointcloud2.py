#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg
import struct
import numpy as np


class LivoxToPointCloud2(Node):
    def __init__(self):
        super().__init__('livox_to_pointcloud2')

        # 输入：Livox 自定义消息
        self.sub = self.create_subscription(
            CustomMsg,
            '/livox/lidar',
            self.callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        # 输出：标准 PointCloud2
        self.pub = self.create_publisher(
            PointCloud2,
            '/livox/points',
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        self.get_logger().info('livox_to_pointcloud2 started')
        self.get_logger().info('  input : /livox/lidar (CustomMsg)')
        self.get_logger().info('  output: /livox/points (PointCloud2)')

    def callback(self, msg: CustomMsg):
        n = msg.point_num
        if n == 0:
            return

        # 预分配 numpy 数组，提高效率
        points = np.zeros((n, 4), dtype=np.float32)  # x, y, z, intensity

        for i, p in enumerate(msg.points):
            points[i, 0] = p.x
            points[i, 1] = p.y
            points[i, 2] = p.z
            points[i, 3] = float(p.reflectivity)

        # 构造 PointCloud2
        cloud_msg = PointCloud2()
        cloud_msg.header = msg.header  # 保留原始时间戳和 frame_id
        cloud_msg.height = 1
        cloud_msg.width = n
        cloud_msg.is_bigendian = False
        cloud_msg.is_dense = True
        cloud_msg.point_step = 16  # 4 个 float32 = 16 字节
        cloud_msg.row_step = cloud_msg.point_step * n
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg.data = points.tobytes()

        self.pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxToPointCloud2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
