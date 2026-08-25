import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped,PoseStamped,Quaternion
from tf2_ros import  TransformBroadcaster, Buffer, TransformListener, TransformException
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
import numpy as np
import math
import threading
import time
from rm_interfaces.srv import stalker_data
class Stalker(Node):
    def __init__(self,msg):
        super().__init__(node_name='stalker')
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = msg.map_frame
        self.target_frame = msg.target_frame
        self.camera_frame = msg.camera_frame
        self.map_frame = msg.map_frame
        self.can_transform = False
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.stalker_service = self.create_service(stalker_data, 'stalker', self.stalker_callback)

    #目标消息回调函数
    def stalker_callback(self, msg):
        # 使用can_transform检查（不会抛出异常）
        self.can_transform = self.tf_buffer.can_transform(
            self.target_frame,
            self.camera_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0)
        )
        if self.can_transform:
            try:
                # 获取目标相对于相机的变换
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.target_frame,
                    rclpy.time.Time()
                )
                self.goal_pose.header.stamp = transform.header.stamp
                self.goal_pose.pose.position.x = transform.transform.translation.x
                self.goal_pose.pose.position.y = transform.transform.translation.y
                self.goal_pose.pose.orientation.w = transform.transform.rotation.w

                self.navigator.cancelTask()
                self.navigator.goToPose(self.goal_pose)
                while not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                    self.navigator.get_logger().info(
                        f'距离目标还有: {feedback.distance_remaining:.2f} 米')
                    #取消导航任务
                    # self.navigator.cancelTask()
                result = self.navigator.getResult()
                self.navigator.get_logger().info(f'导航任务完成，结果: {result}')

            except Exception as e:
                self.get_logger().error(f'Failed to get transform: {e}')

