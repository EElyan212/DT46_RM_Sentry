from rclpy.node import Node
from rm_interfaces.msg import Decision 
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.decision = Decision()
        # 1. 先获取参数并完成 PoseStamped 转换
        self.get_points()

        # 2. 再创建订阅者，确保回调触发时坐标点已准备就绪
        self.sub_decision = self.create_subscription(Decision, "/nav/decision", self.decision_callback, 10)
        self.sub_vision = self.create_subscription(Decision, "/vision/decision", self.vision_callback, 10)

    def create_pose_stamped(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        
        # 全向机器人平移，固定姿态 w=1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        
        return pose

    def get_points(self):
        # 声明参数，默认值为 [x, y] 浮点数数组
        self.declare_parameters(
            namespace='',
            parameters=[
                ('center', [0.0, 0.0]),
                ('home', [0.0, 0.0]),
                ('defence_left', [0.0, 0.0]),          
                ('defence_right', [0.0, 0.0]),     
                ('attack', [0.0, 0.0]),
                ('to_center_param1', [0.0, 0.0]),
                ('to_center_param2', [0.0, 0.0]),
                ('to_center_param3', [0.0, 0.0]),
                ('hp_limit', 0.0)
            ]
        )
        self.to_center_poses = []
        # 获取参数数组，并在赋值处直接转换为 PoseStamped 对象
        # 中心点
        center_param = self.get_parameter("center").value
        self.center = self.create_pose_stamped(center_param[0], center_param[1])
        #中心点路点1
        to_center_param1 = self.get_parameter("to_center_param1").value
        self.to_center_pose1 = self.create_pose_stamped(to_center_param1[0], to_center_param1[1])
        self.to_center_poses.append(self.to_center_pose1)

        to_center_param2 = self.get_parameter("to_center_param2").value
        self.to_center_pose2 = self.create_pose_stamped(to_center_param2[0], to_center_param2[1])
        self.to_center_poses.append(self.to_center_pose2)

        to_center_param3 = self.get_parameter("to_center_param3").value
        self.to_center_pose3 = self.create_pose_stamped(to_center_param3[0], to_center_param3[1])
        self.to_center_poses.append(self.to_center_pose3)

        # 出生点
        home_param = self.get_parameter("home").value
        self.home = self.create_pose_stamped(home_param[0], home_param[1])
        # 防守点左
        def_left_param = self.get_parameter("defence_left").value
        self.defence_left = self.create_pose_stamped(def_left_param[0], def_left_param[1])
        # 防守点右
        def_right_param = self.get_parameter("defence_right").value
        self.defence_right = self.create_pose_stamped(def_right_param[0], def_right_param[1])
        # 攻击点
        attack_param = self.get_parameter("attack").value
        self.attack = self.create_pose_stamped(attack_param[0], attack_param[1])

        #血量下限
        self.hp_limit = self.get_parameter("hp_limit").value

        self.get_logger().info("导航点参数已读取并转换为 PoseStamped 对象")

    def vision_callback(self, msg):
        pass
    def go_center(self):
        self.navigator.goToPose(self.center)
        print("正在前往中心点")
        print(self.center)
    def decision_callback(self, msg):
        # 此时 self.center 等变量已经是 PoseStamped 对象，可以直接发送
        # if not self.navigator.isTaskComplete():
        #     self.navigator.cancelTask()
        #     self.get_logger().info("上一个任务尚未完成,中断任务并开始新的任务")
            #血量检测，低于阈值则返回补给点
        self.decision = msg
        # print(msg)
        if self.decision.self_sentry_hp < self.hp_limit:
            if self.current_goal_type != "HOME":
                self.get_logger().warn(f"血量过低 ({self.self_sentry_hp})！立即撤回补给点")
                self.navigator.goToPose(self.home)
                self.current_goal_type = "HOME"
            return 
        # if self.remain_time >150 and self.remain_bullet>100:

            
        #     if self.current_goal_type != "defence_left":
        #         self.navigator.goToPose(self.defence_left)
        #         self.current_goal_type = "defence_left"
        #         self.get_logger().info(f"当前目标：{self.current_goal_type}")
        #         return
        # if self.remain_time <150 and self.remain_bullet>50:
        #     if self.current_goal_type != "defence_right":
        #         self.navigator.goToPose(self.defence_right)
        #         self.current_goal_type = "defence_right"
        #         self.get_logger().info(f"当前目标：{self.current_goal_type}")
        #         return
        #     #占领之后，血量充足前压路口
        # if self.occupation == 1 and self.sentry_hp >50:
        #     if self.current_goal_type != "attack":
        #         self.navigator.goToPose(self.attack)
        #         self.current_goal_type = "attack"
        #         self.get_logger().info(f"当前目标：{self.current_goal_type}")
        #         return
def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    node.navigator.waitUntilNav2Active()
    node.get_logger().info("Nav2 激活，等待比赛开始信号...")
    #比赛状态判断
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        
        if node.decision.match_progress is not None:
            # 仅在检测到 match_progress == 4 时执行一次
            if node.decision.match_progress == 0:
                node.get_logger().info(">>> 检测到比赛开始！下发中心点目标 <<<")
                node.go_center()
                break
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点正在关闭...")
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



