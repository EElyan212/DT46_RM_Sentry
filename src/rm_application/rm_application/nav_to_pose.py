from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy


def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 3.8
    goal_pose.pose.position.y = -1.1
    goal_pose.pose.orientation.w = 0.84
    # navigator.setInitialPose(goal_pose)
    navigator.goToPose(goal_pose)
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.get_logger().info(
            f'距离目标还有: {feedback.distance_remaining:.2f} 米')
        #取消导航任务
        # navigator.cancelTask()
    result = navigator.getResult()
    navigator.get_logger().info(f'导航任务完成，结果: {result}')
    # rclpy.spin(navigator)
    # rclpy.shutdown()

