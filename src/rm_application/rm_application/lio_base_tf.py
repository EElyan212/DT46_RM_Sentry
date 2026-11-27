#!/usr/bin/env python3
"""把 Point-LIO 的「雷达位姿」里程计换算成机器人基座（base_footprint）的 TF。

背景
----
Point-LIO 估计的是雷达自身（livox_frame）在 odom 下的位姿。
URDF 里 livox_frame 已经挂在机器人树上（base_footprint -> base_link -> Yaw -> livox_frame），
所以 livox_frame 的父节点只能由 robot_state_publisher 提供；定位节点再广播
odom -> livox_frame 就会出现「同一个 frame 两个父节点」，把整棵 TF 树劈成两半：
  * `view_frames` / `tf2_monitor` 里 livox_frame 有两个 broadcaster
  * AMCL、pointcloud_to_laserscan 间歇性 lookup 失败
  * RViz 机器人模型跳变

做法
----
本节点只补「世界系 -> 机器人基座」这一条边，不动机器人内部的任何 frame：

    T_odom_base = T_odom_livox * T_livox_base

  * T_odom_livox 来自 Point-LIO 的 nav_msgs/Odometry（默认 /aft_mapped_to_init）
  * T_livox_base 从 TF（即 URDF）里查一次并缓存，不在这里写死任何数字，
    所以改 CAD / 改安装位置只需要改 URDF

配合
----
  * Point-LIO 关闭自己的 TF 广播：mapping_mid360.launch.py publish_tf:=false
  * 底盘 <-> 云台 <-> 雷达 的相对关系全部由 robot_state_publisher 按 URDF 发布
  * livox_frame_joint 在 URDF 里是 fixed，所以外参是常量，查一次就够；
    将来如果雷达要相对底盘转动，把 refresh_extrinsic 置 true（每帧重查）
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def quat_mul(a, b):
    """四元数乘法 a*b（Hamilton 约定，元素顺序 x, y, z, w）。"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q, v):
    """用四元数 q 旋转向量 v。"""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def compose(a_q, a_t, b_q, b_t):
    """组合两个刚体变换 (R_a, t_a) * (R_b, t_b)。"""
    q = quat_mul(a_q, b_q)
    rot_t = quat_rotate(a_q, b_t)
    t = (a_t[0] + rot_t[0], a_t[1] + rot_t[1], a_t[2] + rot_t[2])
    return q, t


def quat_to_rpy(q):
    """四元数转 roll/pitch/yaw（仅用于日志打印）。"""
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) \
        if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class LioBaseTf(Node):
    def __init__(self):
        super().__init__('lio_base_tf')

        self.declare_parameter('odom_topic', '/aft_mapped_to_init')
        self.declare_parameter('base_frame', 'base_footprint')
        # 里程计消息里 child_frame_id 为空时的兜底名字
        self.declare_parameter('body_frame_fallback', 'livox_frame')
        # false: 只在启动时查一次外参并缓存（livox_frame 相对底盘固定时用）
        # true : 每条里程计都重查（雷达会相对底盘转动时用）
        self.declare_parameter('refresh_extrinsic', False)

        self.base_frame = self.get_parameter('base_frame').value
        self.body_frame_fallback = self.get_parameter('body_frame_fallback').value
        self.refresh_extrinsic = bool(
            self.get_parameter('refresh_extrinsic').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self._body_to_base = None   # T_livox_base，启动后缓存
        self._waiting_logged = False

        odom_topic = self.get_parameter('odom_topic').value
        self.subscription = self.create_subscription(
            Odometry, odom_topic, self.odometry_callback, 20)

        self.get_logger().info(
            f'lio_base_tf: {odom_topic} (T_odom_livox) + URDF T_livox_base '
            f'-> TF odom -> {self.base_frame}')

    def _lookup_body_to_base(self, body_frame):
        """查 T_livox_base：base_frame 在 body_frame 下的位姿（来自 URDF）。"""
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                body_frame, self.base_frame, Time())
        except TransformException as exc:
            if not self._waiting_logged:
                self._waiting_logged = True
                self.get_logger().warn(
                    f'还不确定 {body_frame} <- {self.base_frame}（{exc}），'
                    '先不发布 TF。检查 robot_state_publisher 是否加载了 URDF')
            return None

        self._waiting_logged = False
        q = (tf_msg.transform.rotation.x, tf_msg.transform.rotation.y,
             tf_msg.transform.rotation.z, tf_msg.transform.rotation.w)
        t = (tf_msg.transform.translation.x, tf_msg.transform.translation.y,
             tf_msg.transform.translation.z)

        # 取逆打印成「雷达在基座系下的安装外参」，方便和 CAD / 卷尺对数
        inv_q = (-q[0], -q[1], -q[2], q[3])
        inv_t = quat_rotate(inv_q, (-t[0], -t[1], -t[2]))
        roll, pitch, yaw = quat_to_rpy(inv_q)
        self.get_logger().info(
            f'URDF 外参 {self.base_frame} <- {body_frame}: '
            f'xyz=({inv_t[0]:.4f}, {inv_t[1]:.4f}, {inv_t[2]:.4f}) '
            f'rpy=({roll:.4f}, {pitch:.4f}, {yaw:.4f})')
        return q, t

    def odometry_callback(self, msg):
        world_frame = msg.header.frame_id
        body_frame = msg.child_frame_id or self.body_frame_fallback

        if not world_frame or world_frame == self.base_frame or \
                body_frame == self.base_frame:
            self.get_logger().warn(
                f'里程计 frame 不合法：{world_frame} -> {body_frame}',
                throttle_duration_sec=5.0)
            return

        if self._body_to_base is None or self.refresh_extrinsic:
            body_to_base = self._lookup_body_to_base(body_frame)
            if body_to_base is not None:
                self._body_to_base = body_to_base
            elif self._body_to_base is None:
                return

        world_to_body_q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                           msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        world_to_body_t = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                           msg.pose.pose.position.z)

        # T_odom_base = T_odom_livox * T_livox_base
        base_q, base_t = compose(
            world_to_body_q, world_to_body_t, self._body_to_base[0],
            self._body_to_base[1])

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = world_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = base_t[0]
        transform.transform.translation.y = base_t[1]
        transform.transform.translation.z = base_t[2]
        transform.transform.rotation.x = base_q[0]
        transform.transform.rotation.y = base_q[1]
        transform.transform.rotation.z = base_q[2]
        transform.transform.rotation.w = base_q[3]
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = LioBaseTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
