from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from rm_interfaces.msg import Decision, EnemyCenter
from std_msgs.msg import Int32
from rclpy.action import ActionClient
from rm_interfaces.action import Chase
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

class RosInterface:
    """行为树与 ROS 系统之间的统一通信接口。"""

    def __init__(self, node, blackboard):
        self.node = node
        self.blackboard = blackboard

        # 声明占点和补给点的 map 坐标
        # 启动时加载的 YAML 会覆盖默认值
        self.node.declare_parameter(
            "center",
            [4.86, -2.39],
        )
        self.node.declare_parameter(
            "home",
            [-0.233, 0.833],
        )

        # 保存占点和补给点坐标
        self.center_position = self.node.get_parameter(
            "center"
        ).value
        self.home_position = self.node.get_parameter(
            "home"
        ).value

        # 占点和回补给共用的 Nav2 导航接口
        self.navigator = BasicNavigator(
            node_name="behavior_tree_navigator"
        )

        # 记录当前 Nav2 导航属于哪个行为
        self.navigation_owner = None

        # 接收裁判系统和下位机提供的决策数据
        self.decision_subscription = self.node.create_subscription(
            Decision,
            "/nav/decision",
            self.decision_callback,
            10,
        )
        # 发布小陀螺模式，由 serial_node 转发给下位机
        self.gimbal_mode_publisher = self.node.create_publisher(
            Int32,
            "/gimbal_mode",
            1,
        )
        # 调用现有追击功能提供的 /tracker/chase Action
        self.chase_action_client = ActionClient(
            self.node,
            Chase,
            "/tracker/chase",
        )

        # 保存追击请求句柄和最终结果
        self.chase_goal_handle = None
        self.chase_result = None

        # 请求已经发出，但服务端还没有返回 GoalHandle
        self.chase_goal_pending = False

        # 在 GoalHandle 返回前是否已经要求停止追击
        self.chase_cancel_pending = False
    
        # 尚未收到敌人数据
        self.latest_enemy = None

        # 最后一次有效跟踪到敌人的时间
        self.last_enemy_tracked_time = None

        # 缓存 TF 数据，用于查询敌人和哨兵的地图位置
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self.node,
        )

        # 与自瞄发布端使用兼容的通信配置
        enemy_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 订阅自瞄提供的敌人坐标和跟踪状态
        self.enemy_subscription = self.node.create_subscription(
            EnemyCenter,
            "/tracker/enemy_datas",
            self.enemy_callback,
            enemy_qos,
        )

    def decision_callback(self, msg):
        # 只把当前行为树已经使用的数据写入共享黑板
        self.blackboard.match_progress = msg.match_progress
        self.blackboard.self_hp = msg.self_sentry_hp

    def enemy_callback(self, msg):
        # 保存最新收到的敌人数据
        self.latest_enemy = msg

        # 只在有效跟踪敌人时更新时间
        if msg.tracked:
            self.last_enemy_tracked_time = (
                self.node.get_clock().now()
            )
            
    def is_enemy_within_lost_grace(self, grace_time):
        # 从未跟踪到敌人时，不能使用等待时间
        if self.last_enemy_tracked_time is None:
            return False

        # 计算距离最后一次有效跟踪经过了多久
        elapsed = (
            self.node.get_clock().now()
            - self.last_enemy_tracked_time
        ).nanoseconds / 1e9

        return elapsed <= float(grace_time)

    def get_enemy_position(self, grace_time):
        # 自瞄没有有效目标时，不使用缓存中的敌人坐标
        if (
            self.latest_enemy is None
            or not self.latest_enemy.tracked
            or not self.is_enemy_within_lost_grace(grace_time)
        ):
            return None

        try:
            # 查询敌人相对于地图的位置
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

    def create_navigation_pose(self, position):
        # 创建 map 坐标系下的导航目标
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.node.get_clock().now().to_msg()

        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = 0.0

        # 暂时不要求底盘到达后朝向特定方向
        pose.pose.orientation.w = 1.0

        return pose

    def start_occupy(self):
        # 把中央占点坐标包装成 Nav2 目标消息
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
    
    def wait_until_navigation_active(self):
        # 等待 Nav2 完成启动
        self.navigator.waitUntilNav2Active()

    def destroy(self):
        # 销毁通信接口内部使用的 Nav2 节点
        self.navigator.destroy_node()

    def start_spin(self):
        # 1 表示开启小陀螺
        self.gimbal_mode_publisher.publish(
            Int32(data=1)
        )
        self.node.get_logger().info("已开启小陀螺")
        return True

    def stop_spin(self):
        # 0 表示关闭小陀螺
        self.gimbal_mode_publisher.publish(
            Int32(data=0)
        )
        self.node.get_logger().info("已关闭小陀螺")
    
    def start_chase(self):
        # 追击服务尚未启动时，暂时不能开始追击
        if not self.chase_action_client.server_is_ready():
            self.node.get_logger().warn(
                "追击 Action 服务尚未就绪"
            )
            return False

        # 创建追击请求
        goal = Chase.Goal()

        # 0.0 表示使用 chase_client YAML 中的默认偏移距离
        goal.offset_distance = 0.0

        # 上一次追击请求还没有结束时，不重复发送
        if (
            self.chase_goal_pending
            or self.chase_goal_handle is not None
        ):
            self.node.get_logger().warn(
                "上一追击请求尚未结束"
            )
            return False

        # 清空上一次追击状态
        self.chase_result = None
        self.chase_cancel_pending = False

        # 从现在开始等待服务端返回 GoalHandle
        self.chase_goal_pending = True

        # 异步发送请求
        send_future = self.chase_action_client.send_goal_async(goal)
        send_future.add_done_callback(
            self.chase_goal_response_callback
        )

        # 表示请求已成功发出
        return True

    def chase_goal_response_callback(self, future):
        # 服务端已经答复，不再等待 GoalHandle
        self.chase_goal_pending = False

        # 获取追击服务对请求的答复
        try:
            goal_handle = future.result()
        except Exception as error:
            self.node.get_logger().error(
                f"发送追击请求失败: {error}"
            )
            self.chase_result = False
            self.chase_cancel_pending = False
            return

        # 服务端拒绝请求
        if not goal_handle.accepted:
            self.node.get_logger().warn(
                "追击请求被拒绝"
            )
            self.chase_result = False
            self.chase_cancel_pending = False
            return

        # 保存请求句柄，之后取消追击时使用
        self.chase_goal_handle = goal_handle
        self.node.get_logger().info(
            "追击请求已接受"
        )

        # 异步等待追击结束
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.chase_result_callback
        )

        # 等待 GoalHandle 时已经要求停止，现在立即取消
        if self.chase_cancel_pending:
            goal_handle.cancel_goal_async()
            self.node.get_logger().info(
                "追击请求刚被接受，立即请求取消"
            )
    
    def chase_result_callback(self, future):
        # 获取 Chase Action 的最终结果
        try:
            result = future.result().result
            self.chase_result = bool(result.success)
            self.node.get_logger().info(
                f"追击结束: {result.message}"
            )
        except Exception as error:
            self.node.get_logger().error(
                f"获取追击结果失败: {error}"
            )
            self.chase_result = False

        # 本次追击已经结束，清理运行状态
        self.chase_goal_handle = None
        self.chase_cancel_pending = False

    def get_chase_result(self):
        # None 表示仍在追击，True/False 表示已经结束
        return self.chase_result

    def stop_chase(self):
        # 无论是否拿到 GoalHandle，都先记录取消要求
        self.chase_cancel_pending = True

        # GoalHandle 尚未返回，等回调收到后再取消
        if self.chase_goal_handle is None:
            if self.chase_goal_pending:
                self.node.get_logger().info(
                    "追击请求仍在等待接受，已记录取消要求"
                )
            return

        # 已经拿到 GoalHandle，立即请求取消
        self.chase_goal_handle.cancel_goal_async()
        self.node.get_logger().info(
            "已请求停止追击"
        )