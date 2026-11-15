import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import math

# 手动实现四元数到欧拉角的转换，彻底避开 numpy 版本冲突
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z


class TFListener(Node):
    def __init__(self):
        super().__init__('tf2_listener')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(1.0, self.get_transform)

    def get_transform(self):
        try:
            tf = self.buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))
            transform = tf.transform
            
            # 调用内置的转换函数
            rotation_euler = euler_from_quaternion(
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w
            )
            
            self.get_logger().info(
                f'平移: {transform.translation}, 旋转欧拉角(弧度): {rotation_euler}'
            )
        except Exception as e:
            self.get_logger().warn(f'无法获取坐标变换，原因: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = TFListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()