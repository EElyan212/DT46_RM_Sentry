import time
from pathlib import Path

import rclpy
import py_trees
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from rm_application.behavior_tree.ros_interface import RosInterface
from rm_application.behavior_tree.blackboard import Blackboard
from rm_application.behavior_tree.xml_loader import load_behavior_tree

class BehaviorTreeNode(Node):
    def __init__(self):
        super().__init__("behavior_tree_node")

        # 创建整棵行为树共用的唯一黑板
        self.blackboard = Blackboard()

        # 创建行为树与 ROS 系统之间的统一通信接口
        self.ros_interface = RosInterface(self,self.blackboard,)

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
        # 追击过程中允许短暂丢失目标的时间
        self.declare_parameter("target_lost_grace_time", 0.5)
        # 追击恢复耗尽后，多久允许再次选择追击，单位：秒
        self.declare_parameter("chase_failure_cooldown_seconds", 3.0)
        
        self.declare_parameter("hp_limit", 150)
        self.declare_parameter("hp_up", 400)

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

        self.target_lost_grace_time = float(
            self.get_parameter("target_lost_grace_time").value
        )

        # 读取回血阈值，供节点注册和行为判断使用
        self.hp_limit = self.get_parameter("hp_limit").value
        self.hp_up = self.get_parameter("hp_up").value

        self.patrol_min_goal_distance = (
            self.ros_interface.patrol_min_goal_distance
        )
        # 提供恢复参数，供节点注册表创建恢复节点
        self.recovery_wait_seconds = (
            self.ros_interface.recovery_wait_seconds
        )
        self.recovery_max_retries = (
            self.ros_interface.recovery_max_retries
        )
        self.recovery_min_goal_distance = (
            self.ros_interface.recovery_min_goal_distance
        )
        self.recovery_sample_attempts = (
            self.ros_interface.recovery_sample_attempts
        )
        # 从 ROS 包安装目录加载主树，子树由 XML 内部引用
        tree_path = (
            Path(get_package_share_directory("rm_application"))
            / "tree"
            / "main_tree.xml"
        )
        self.behavior_tree = load_behavior_tree(self, tree_path)

        # 每 0.1 秒运行一次行为树，即 10 Hz
        self.behavior_tree_timer = self.create_timer(
            0.1,
            self.tick_behavior_tree,
        )

    def tick_behavior_tree(self):
        # 执行完整一轮 tick，重新检查优先级和当前条件
        self.behavior_tree.tick_once()
    
    def set_navigation_task(self, task):
        # 任务选择树把当前导航任务写入共享黑板
        self.blackboard.navigation_task = task

    def get_navigation_task(self):
        # 导航执行树从共享黑板读取当前导航任务
        return self.blackboard.navigation_task

    def is_chase_ready(self):
        # 冷却结束后，还需要由追击条件节点检查敌人和边界
        return time.monotonic() >= self.blackboard.chase_retry_after

    def report_chase_exhausted(self):
        # 追击恢复耗尽时，暂时关闭追击资格
        cooldown_seconds = float(
            self.get_parameter(
                "chase_failure_cooldown_seconds"
            ).value
        )

        self.blackboard.chase_retry_after = (
            time.monotonic() + cooldown_seconds
        )

        self.get_logger().warn(
            "追击恢复耗尽，暂时禁止选择追击"
        )    
    
    def set_tactical_mode(self, tactical_mode):
        # 战术选择结果写入共享黑板
        self.blackboard.tactical_mode = tactical_mode

    def get_tactical_mode(self):
        # 行为树读取当前采用的战术模式
        return self.blackboard.tactical_mode


    def set_patrol_region(self, patrol_region):
        # 战术选择树把巡逻区域写入共享黑板
        self.blackboard.patrol_region = patrol_region

    def get_patrol_region(self):
        # 巡逻行为从共享黑板读取当前巡逻区域
        return self.blackboard.patrol_region

    def start_occupy(self):
        # 占点行为不直接操作 Nav2
        # 把开始占点的请求交给统一通信接口
        return self.ros_interface.start_occupy()

    def start_resupply(self):
        # 回血行为不直接操作 Nav2
        # 把开始回补给的请求交给统一通信接口
        return self.ros_interface.start_resupply()

    def get_occupy_result(self):
        # 从统一通信接口读取占点导航结果
        return self.ros_interface.get_occupy_result()

    def get_resupply_result(self):
        # 从统一通信接口读取回补给导航结果
        return self.ros_interface.get_resupply_result()


    def stop_occupy(self):
        # 把停止占点导航的请求交给统一通信接口
        self.ros_interface.stop_occupy()

    def stop_resupply(self):
        # 把停止回补给导航的请求交给统一通信接口
        self.ros_interface.stop_resupply()

    def get_patrol_area(self, patrol_region):
        return self.ros_interface.get_patrol_area(
            patrol_region
        )

    def start_patrol(self, target_position):
        # 只转发选好的坐标
        return self.ros_interface.start_patrol(
            target_position
        )

    def get_patrol_result(self):
        # 从统一通信接口读取当前巡逻目标的导航结果
        return self.ros_interface.get_patrol_result()

    def stop_patrol(self):
        # 把停止巡逻导航的请求交给统一通信接口
        self.ros_interface.stop_patrol()

    def get_last_navigation_target(self, owner):
        # 读取指定行为最近尝试的目标，供恢复选点避开失败点
        return self.ros_interface.get_last_navigation_target(
            owner
        )

    def is_navigation_stopped(self):
        # 普通 Nav2 导航必须已经结束并释放
        if not self.ros_interface.is_navigation_stopped():
            return False

        # 追击服务也必须已结束，才能启动新的平移任务
        return self.ros_interface.is_chase_stopped()

    def start_spin(self):
        # 把开启小陀螺的请求交给统一通信接口
        return self.ros_interface.start_spin()

    def stop_spin(self):
        # 把关闭小陀螺的请求交给统一通信接口
        self.ros_interface.stop_spin()
    
    def start_chase(self):
        # 把开始追击的请求交给统一通信接口
        return self.ros_interface.start_chase()

    def get_chase_result(self):
        # 从统一通信接口读取追击执行结果
        return self.ros_interface.get_chase_result()

    def stop_chase(self):
        # 把停止追击的请求交给统一通信接口
        self.ros_interface.stop_chase()

    def get_match_progress(self):
        # 裁判数据由通信接口写入黑板
        # 这里读取最新比赛阶段，供行为树判断比赛是否正在进行
        return self.blackboard.match_progress

    def get_hp(self):
        # 裁判数据由通信接口写入黑板
        # 这里读取最新哨兵血量，供行为树判断是否需要回补给
        return self.blackboard.self_hp
    
    def is_enemy_within_lost_grace(self):
        # 由通信接口判断目标是否仍在短暂丢失宽限时间内
        return self.ros_interface.is_enemy_within_lost_grace(
            self.target_lost_grace_time
        )

    def get_enemy_position(self):
        # 从统一通信接口读取有效敌人位置
        return self.ros_interface.get_enemy_position(
            self.target_lost_grace_time
        )
    
    def get_robot_position(self):
        # 从统一通信接口读取哨兵的地图位置
        return self.ros_interface.get_robot_position()

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
        node.ros_interface.wait_until_navigation_active()
        node.get_logger().info(
            "Nav2 已就绪，行为树开始运行"
        )

        # 处理订阅、Action 回调和行为树定时器
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("行为树节点正在关闭")

    finally:
        # 打断整棵树，触发活动节点的取消与停止逻辑
        node.behavior_tree.stop(py_trees.common.Status.INVALID)

        # 销毁 BehaviorTreeNode 内部使用的 Nav2 节点
        node.ros_interface.destroy()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
