import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from rclpy.action import ActionClient
from rm_interfaces.action import Chase
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from rm_interfaces.msg import Decision, EnemyCenter
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_msgs.msg import Int32
from rm_application.behavior_tree.builder import BehaviorTreeBuilder

class BehaviorTreeNode(Node):
    def __init__(self):
        super().__init__("behavior_tree_node")

        # 声明参数；启动时加载的 YAML 会覆盖默认值
        self.declare_parameter("control_zone_enabled", False)
        self.declare_parameter("control_zone_x_min", 0.0)
        self.declare_parameter("control_zone_x_max", 0.0)
        self.declare_parameter("control_zone_y_min", 0.0)
        self.declare_parameter("control_zone_y_max", 0.0)

        # 声明最大追击区域参数；YAML 中的配置会覆盖这些默认值
        self.declare_parameter("chase_boundary_enabled", False)
        self.declare_parameter("chase_boundary_x_min", 0.0)
        self.declare_parameter("chase_boundary_x_max", 0.0)
        self.declare_parameter("chase_boundary_y_min", 0.0)
        self.declare_parameter("chase_boundary_y_max", 0.0)

        self.declare_parameter("hp_limit", 150)
        self.declare_parameter("hp_up", 380)
        self.declare_parameter("attack_range", 10.0)

        # 声明并读取占点和补给点的 map 坐标
        self.declare_parameter("center", [4.86, -2.39])
        self.declare_parameter("home", [-0.233, 0.833])

        self.center_position = self.get_parameter("center").value
        self.home_position = self.get_parameter("home").value

        # 保存参数，供后续控制区判断使用
        self.control_zone_enabled = self.get_parameter(
            "control_zone_enabled"
        ).value
        self.control_zone_x_min = self.get_parameter(
            "control_zone_x_min"
        ).value
        self.control_zone_x_max = self.get_parameter(
            "control_zone_x_max"
        ).value
        self.control_zone_y_min = self.get_parameter(
            "control_zone_y_min"
        ).value
        self.control_zone_y_max = self.get_parameter(
            "control_zone_y_max"
        ).value

        # 保存最大追击区域参数，供追击边界判断使用
        self.chase_boundary_enabled = self.get_parameter(
            "chase_boundary_enabled"
        ).value
        self.chase_boundary_x_min = self.get_parameter(
            "chase_boundary_x_min"
        ).value
        self.chase_boundary_x_max = self.get_parameter(
            "chase_boundary_x_max"
        ).value
        self.chase_boundary_y_min = self.get_parameter(
            "chase_boundary_y_min"
        ).value
        self.chase_boundary_y_max = self.get_parameter(
            "chase_boundary_y_max"
        ).value

        # 读取回补给和攻击距离参数，供 builder 使用
        self.hp_limit = self.get_parameter("hp_limit").value
        self.hp_up = self.get_parameter("hp_up").value
        self.attack_range = self.get_parameter("attack_range").value
        
        # 尚未收到裁判系统数据
        self.latest_decision = None

        # 尚未收到敌人数据
        self.latest_enemy = None

        # 缓存 TF 数据，用于查询敌人的地图位置
        self.tf_buffer = Buffer()

        # 接收 TF 更新并存入缓存
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 占点和回补给共用的 Nav2 导航接口
        self.navigator = BasicNavigator(
            node_name="behavior_tree_navigator"
        )

        # 记录当前 Nav2 导航属于哪个行为
        self.navigation_owner = None
        
        # 调用现有追击功能提供的 /tracker/chase Action
        self.chase_action_client = ActionClient(
            self,
            Chase,
            "/tracker/chase",
        )

        # 发布小陀螺模式，由 serial_node 转发给下位机
        self.gimbal_mode_publisher = self.create_publisher(
        Int32,
        "/gimbal_mode",
        1,
        )

        # 保存追击请求和运行结果
        self.chase_goal_handle = None
        self.chase_result = None

        # 追击请求已经发出，但服务端还没有返回 GoalHandle
        self.chase_goal_pending = False

        # 在 GoalHandle 返回前是否已经要求停止追击
        self.chase_cancel_pending = False

        # 订阅血量、比赛阶段和控制区状态
        self.decision_subscription = self.create_subscription(
            Decision,
            "/nav/decision",
            self.decision_callback,
            10,
        )

        # 与自瞄发布端使用兼容的通信配置
        enemy_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 订阅自瞄提供的敌人坐标和跟踪状态
        self.enemy_subscription = self.create_subscription(
            EnemyCenter,             # 接收的数据格式
            "/tracker/enemy_datas",  # 接收哪个话题
            self.enemy_callback,     # 收到消息后调用哪个方法
            enemy_qos,               # 消息传输配置
        )

        # 使用当前 ROS 节点提供的数据和通信方法构建行为树
        self.behavior_tree = BehaviorTreeBuilder(self).build()

        # 每 0.1 秒运行一次行为树，即 10 Hz
        self.behavior_tree_timer = self.create_timer(
            0.1,
            self.tick_behavior_tree,
        )

    def decision_callback(self, msg):
        # 每次收到新消息，都保存最新的一份
        self.latest_decision = msg

    def enemy_callback(self, msg):
        # 保存最新收到的敌人数据
        self.latest_enemy = msg

    def tick_behavior_tree(self):
        # 每次 tick 都会重新检查优先级和当前条件
        self.behavior_tree.tick()
    
    def create_navigation_pose(self, position):
        # 创建 map 坐标系下的导航目标
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = 0.0

        # 暂时不要求底盘到达后朝向特定方向
        pose.pose.orientation.w = 1.0

        return pose
    
    def start_occupy(self):
        # 把占点坐标包装成 Nav2 目标消息
        target_pose = self.create_navigation_pose(
            self.center_position
        )

        # 向 Nav2 发送占点目标
        if not self.navigator.goToPose(target_pose):
            return False

        # 当前导航任务属于占点
        self.navigation_owner = "occupy"
        return True

    def start_resupply(self):
        # 把补给点坐标包装成 Nav2 目标消息
        target_pose = self.create_navigation_pose(
            self.home_position
        )

        # 向 Nav2 发送补给目标
        if not self.navigator.goToPose(target_pose):
            return False

        # 当前导航任务属于回补给
        self.navigation_owner = "resupply"
        return True

    def stop_navigation(self, owner):
        # 只允许当前导航的所属行为取消它
        if self.navigation_owner != owner:
            return

        self.navigator.cancelTask()
        self.navigation_owner = None

    def stop_occupy(self):
        # 只取消属于占点的导航
        self.stop_navigation("occupy")

    def stop_resupply(self):
        # 只取消属于回补给的导航
        self.stop_navigation("resupply")

    def start_spin(self):
        # 1 表示开启小陀螺
        self.gimbal_mode_publisher.publish(
            Int32(data=1)
        )
        self.get_logger().info("已开启小陀螺")
        return True

    def stop_spin(self):
        # 0 表示关闭小陀螺
        self.gimbal_mode_publisher.publish(
            Int32(data=0)
        )
        self.get_logger().info("已关闭小陀螺")    

    def start_chase(self):
        # 追击服务尚未启动时，暂时不能开始追击
        if not self.chase_action_client.server_is_ready():
            self.get_logger().warn("追击 Action 服务尚未就绪")
            return False

        # 创建追击请求
        goal = Chase.Goal()

        # 0.0 表示使用 chase_client YAML 中的默认偏移距离
        goal.offset_distance = 0.0

        # 上一次追击请求还没有结束时，不重复发送
        if self.chase_goal_pending or self.chase_goal_handle is not None:
            self.get_logger().warn("上一追击请求尚未结束")
            return False

        # 清空上一次追击状态
        self.chase_result = None
        self.chase_cancel_pending = False

        # 从现在开始等待服务端返回 GoalHandle
        self.chase_goal_pending = True

        # 异步发送请求，收到服务端答复后调用回调
        send_future = self.chase_action_client.send_goal_async(goal)
        send_future.add_done_callback(
            self.chase_goal_response_callback
        )

        # 表示请求已成功发出
        return True
    
    def chase_goal_response_callback(self, future):
        # 服务端已经答复，不再处于等待 GoalHandle 状态
        self.chase_goal_pending = False

        # 获取追击服务对 Goal 的答复
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(
                f"发送追击请求失败: {error}"
            )
            self.chase_result = False
            self.chase_cancel_pending = False
            return

        # 服务拒绝请求
        if not goal_handle.accepted:
            self.get_logger().warn("追击请求被拒绝")
            self.chase_result = False
            self.chase_cancel_pending = False
            return

        # 保存请求句柄，之后取消追击时使用
        self.chase_goal_handle = goal_handle
        self.get_logger().info("追击请求已接受")

        # 异步等待追击结束
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.chase_result_callback
        )

        # 等待 GoalHandle 期间已经要求停止，现在立即取消
        if self.chase_cancel_pending:
            goal_handle.cancel_goal_async()
            self.get_logger().info(
                "追击请求刚被接受，立即请求取消"
            )

    def chase_result_callback(self, future):
        # 获取 Chase Action 最终结果
        try:
            result = future.result().result
            self.chase_result = bool(result.success)
            self.get_logger().info(
                f"追击结束: {result.message}"
            )
        except Exception as error:
            self.get_logger().error(
                f"获取追击结果失败: {error}"
            )
            self.chase_result = False

        self.chase_goal_handle = None
        self.chase_cancel_pending = False
    
    def get_chase_result(self):
        # None 表示仍在追击，True/False 表示已经结束
        return self.chase_result

    def stop_chase(self):
        # 无论是否已经拿到 GoalHandle，都先记录取消要求
        self.chase_cancel_pending = True

        # GoalHandle 尚未返回，等回调收到后再取消
        if self.chase_goal_handle is None:
            if self.chase_goal_pending:
                self.get_logger().info(
                    "追击请求仍在等待接受，已记录取消要求"
                )
            return

        # 已经拿到 GoalHandle，立即请求取消
        self.chase_goal_handle.cancel_goal_async()
        self.get_logger().info("已请求停止追击")

    def get_match_progress(self):
        # 未收到数据时，按比赛未开始处理
        if self.latest_decision is None:
            return 0

        return self.latest_decision.match_progress

    def get_hp(self):
        # 未收到数据时，暂时返回 0
        if self.latest_decision is None:
            return 0

        # 返回最新收到的哨兵血量
        return self.latest_decision.self_sentry_hp
    
    def get_enemy_position(self):
        # 自瞄没有有效目标时，不使用缓存中的敌人坐标
        if self.latest_enemy is None or not self.latest_enemy.tracked:
            return None

        try:
            # 查询敌人相对于地图的位置，Time() 表示最新可用变换
            transform = self.tf_buffer.lookup_transform(
                "map",
                "enemy",
                Time(),
            )
        except TransformException:
            # TF 尚未连通或暂时不可用
            return None

        position = transform.transform.translation
        return position.x, position.y
    
    def get_robot_position(self):
        # 获取底盘在地图中的位置
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                Time(),
            )
        except TransformException:
            # 尚未获得有效定位
            return None

        position = transform.transform.translation
        return position.x, position.y

    def get_control_zone(self):
        # 提供控制区配置，边界判断由条件逻辑完成
        return {
            "enabled": self.control_zone_enabled,
            "x_min": self.control_zone_x_min,
            "x_max": self.control_zone_x_max,
            "y_min": self.control_zone_y_min,
            "y_max": self.control_zone_y_max,
        }
    def get_chase_boundary(self):
        # 提供最大追击区域配置，供追击边界判断使用
        return {
            "enabled": self.chase_boundary_enabled,
            "x_min": self.chase_boundary_x_min,
            "x_max": self.chase_boundary_x_max,
            "y_min": self.chase_boundary_y_min,
            "y_max": self.chase_boundary_y_max,
        }
    
def main(args=None):
    rclpy.init(args=args)

    node = BehaviorTreeNode()

    try:
        # 等待 Nav2 启动完成后再运行行为树
        node.navigator.waitUntilNav2Active()
        node.get_logger().info(
            "Nav2 已就绪，行为树开始运行"
        )

        # 处理订阅、Action 回调和行为树定时器
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("行为树节点正在关闭")

    finally:
        # 停止正在运行的追击、导航和小陀螺
        node.behavior_tree.halt()

        # 销毁 BehaviorTreeNode 内部使用的 Nav2 节点
        node.navigator.destroy_node()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()