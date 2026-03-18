from rclpy.node import Node
from rm_interfaces.msg import Decision 
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from std_msgs.msg import Int32
class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        
        self.navigator = BasicNavigator()
        # self.navigator.waitUntilNav2Active()
        self.decision = Decision()
        self.decision_flag = "READY"
        # 1. 先获取参数并完成 PoseStamped 转换
        self.get_points()
        self.current_goal = None
        # 2. 再创建订阅者，确保回调触发时坐标点已准备就绪
        self.sub_decision = self.create_subscription(Decision, "/nav/decision", self.decision_msg_callback, 10)
        self.sub_vision = self.create_subscription(Decision, "/vision/decision", self.vision_callback, 10)
        self.pub_gimbal_mode = self.create_publisher(Int32, "/gimbal_mode", 1)
        self.timer = self.create_timer(0.5, self.timer_callback)
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
                ('defence_mid', [0.0, 0.0]),
                ('defence_mid_left', [0.0, 0.0]),
                ('defence_mid_right', [0.0, 0.0]),    
                ('attack', [0.0, 0.0]),
                ('to_center_param1', [0.0, 0.0]),
                ('to_center_param2', [0.0, 0.0]),
                ('to_center_param3', [0.0, 0.0]),
                ('hp_limit', 150),
                ('hp_up', 380),

            ]
        )
        self.to_center_poses = []
        self.defences = []
        self.defences_behind = []
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
        self.defences.append(self.defence_left)
        # 防守点中右
        def_mid_right_param = self.get_parameter("defence_mid_right").value
        self.defence_mid_right = self.create_pose_stamped(def_mid_right_param[0], def_mid_right_param[1])
        self.defences.append(self.defence_mid_right)
        # 防守点右
        def_right_param = self.get_parameter("defence_right").value
        self.defence_right = self.create_pose_stamped(def_right_param[0], def_right_param[1])
        self.defences.append(self.defence_right)
        # 防守点中
        def_mid_param = self.get_parameter("defence_mid").value
        self.defence_mid = self.create_pose_stamped(def_mid_param[0], def_mid_param[1])
        self.defences.append(self.defence_mid)
        # 防守点中左
        def_mid_left_param = self.get_parameter("defence_mid_left").value
        self.defence_mid_left = self.create_pose_stamped(def_mid_left_param[0], def_mid_left_param[1])
        self.defences.append(self.defence_mid_left)
        
        # 攻击点
        attack_param = self.get_parameter("attack").value
        self.attack = self.create_pose_stamped(attack_param[0], attack_param[1])

        #血量下限
        self.hp_limit = self.get_parameter("hp_limit").value

        self.hp_up = self.get_parameter("hp_up").value
        self.get_logger().info("导航点参数已读取并转换为 PoseStamped 对象")

    def vision_callback(self, msg):
        pass
    def send_goal(self, pose_or_waypoints, is_waypoint=False):
        """统一发送目标的辅助函数，实时更新时间戳"""
        now = self.get_clock().now().to_msg()
        if is_waypoint:
            if not pose_or_waypoints: # 拦截空列表
                self.get_logger().warn("路点列表为空，跳过导航指令")
                return
            for p in pose_or_waypoints:
                p.header.stamp = now
            self.navigator.followWaypoints(pose_or_waypoints)
        else:
            pose_or_waypoints.header.stamp = now
            self.navigator.goToPose(pose_or_waypoints)

    def decision_msg_callback(self, msg):
        self.decision = msg
        # print(self.decision)
        new_flag = self.decision_flag
        gimbal_mode = 0
        # 1. 优先级最高：比赛结束强制归零
        if self.decision.match_progress != 4 :
            new_flag = "READY"
        # 2. 血量不足撤退
        elif self.decision.self_sentry_hp <= self.hp_limit and self.decision.match_progress == 4:
            new_flag = "RETREAT"
        # # 3. 比赛初始快速抢点
        # elif self.decision.match_progress == 4 and self.decision.remain_time > 290:
        #     new_flag = "OCCUPY_BEGIN"
        # 4. 正常占点逻辑
        elif self.decision.occupation == 0 and self.decision.self_sentry_hp > self.hp_limit and self.decision.match_progress == 4 :
            new_flag = "OCCUPY"
        # 5. 防守逻辑
        elif self.decision.occupation == 1 and self.decision.self_sentry_hp > self.hp_limit and self.decision.match_progress == 4:
            new_flag = "DEFENCE"
        # 6. 后点防守
        # elif self.decision.occupation == 1 and self.decision.self_sentry_hp < self.hp_up and self.decision.self_sentry_hp > self.hp_limit and  self.decision.match_progress == 4:
        #     new_flag = "DEFENCE_BEHIND"
        # # 7. 前压逻辑
        # elif self.decision.occupation == 1 and self.decision.self_sentry_hp > self.hp_up and self.decision.remain_time >=250 and self.decision.match_progress == 4:
        #     new_flag = "ATTACK"
        # #8.决胜占点
        # elif self.decision.occupation == 0 and self.decision.self_sentry_hp > self.hp_limit and self.decision.remain_time <=100 and self.decision.match_progress == 4:
        #     new_flag = "OCCUPY_END"

        if self.decision.occupation == 1 or self.decision.self_sentry_hp < self.hp_up or self.decision.yaw != 0 or self.decision.can_fire == 1:
            gimbal_mode = 1
        if new_flag != self.decision_flag:
            self.get_logger().info(f"切换状态至: {new_flag}")
            self.decision_flag = new_flag
            self.navigator.cancelTask() # 切换时立即中断当前动作
        self.pub_gimbal_mode.publish(Int32(data=gimbal_mode))
        self.get_logger().info(f"gimbal_mode: {gimbal_mode}")
    def timer_callback(self):
    # 如果导航正在进行，且不是因为状态切换被 cancel，则不重复指令
        if not self.navigator.isTaskComplete():
            return
        
        match self.decision_flag:
            case "OCCUPY_BEGIN":
                self.get_logger().info("开始抢点")
                self.send_goal(self.to_center_poses, is_waypoint=True)
                # self.send_goal(self.center)
            case "OCCUPY":
                self.get_logger().info("占点")
                self.send_goal(self.center)
            case "RETREAT":
                self.get_logger().info("撤退")
                self.send_goal(self.home)
            case "DEFENCE":
                self.get_logger().info("防守")
                self.send_goal(self.defences, is_waypoint=True)
            # case "DEFENCE_BEHIND":
            #     self.get_logger().info("后点防守")
            #     self.send_goal(self.defences_behind, is_waypoint=True)
            # case "ATTACK":
            #     self.get_logger().info("前压")
            #     self.send_goal(self.attack)
            # case "OCCUPY_END":
            #     self.get_logger().info("决胜占点")
            #     self.send_goal(self.center)

def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    node.navigator.waitUntilNav2Active()
    node.get_logger().info("Nav2 激活，等待比赛开始信号...")
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



