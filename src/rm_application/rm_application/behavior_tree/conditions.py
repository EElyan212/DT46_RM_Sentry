import math

from rm_application.behavior_tree.core import Node, Status

# 判断是否需要回补给
class NeedResupply(Node):
    def __init__(self, name, get_hp, hp_limit, hp_up):
        super().__init__(name)

        # 获取当前血量的方法
        self.get_hp = get_hp

        # 开始和结束回补给的血量
        self.hp_limit = hp_limit
        self.hp_up = hp_up

        # 是否已经进入回补给状态
        self.resupplying = False

    def tick(self):
        current_hp = self.get_hp()

        # 血量过低，开始回补给
        if current_hp <= self.hp_limit:
            self.resupplying = True

        # 血量恢复，结束回补给
        elif current_hp >= self.hp_up:
            self.resupplying = False

        if self.resupplying:
            return Status.SUCCESS

        return Status.FAILURE

# 判断是否需要追击敌人
class CanChase(Node):
    def __init__(
        self,
        name,
        get_enemy_position,
        get_robot_position,
        get_control_zone,
        is_chasing,
        attack_range,
    ):
        super().__init__(name)

        # 获取敌人和底盘的地图坐标
        self.get_enemy_position = get_enemy_position
        self.get_robot_position = get_robot_position

        # 获取控制区开关和边界参数
        self.get_control_zone = get_control_zone

        # 获取追击状态和攻击距离阈值
        self.is_chasing = is_chasing
        self.attack_range = attack_range

    def tick(self):
        enemy_position = self.get_enemy_position()
        robot_position = self.get_robot_position()

        # 缺少有效位置时，不允许追击
        if enemy_position is None or robot_position is None:
            return Status.FAILURE

        enemy_x, enemy_y = enemy_position
        robot_x, robot_y = robot_position

        # 两者都在 map 坐标系中，计算水平直线距离
        distance = math.hypot(
            enemy_x - robot_x,
            enemy_y - robot_y,
        )

        # 坐标无效或已经进入攻击范围，停止追击
        if not math.isfinite(distance) or distance <= self.attack_range:
            return Status.FAILURE

        # 已经开始追击时，允许离开控制区继续追
        # 最大追击边界仍需后续在追击功能中接入
        if self.is_chasing():
            return Status.SUCCESS

        zone = self.get_control_zone()

        # 调试时关闭限制，允许从控制区外开始追击
        if not zone["enabled"]:
            return Status.SUCCESS

        # 启用限制时，只允许从控制区内开始追击
        inside_zone = (
            zone["x_min"] <= robot_x <= zone["x_max"]
            and zone["y_min"] <= robot_y <= zone["y_max"]
        )

        return Status.SUCCESS if inside_zone else Status.FAILURE

# 判断比赛是否正在进行
class IsMatchRunning(Node):
    def __init__(self, name, get_match_progress):
        super().__init__(name)

        # 获取当前比赛阶段的方法
        self.get_match_progress = get_match_progress

    def tick(self):
        # match_progress 等于 4 表示比赛正在进行
        if self.get_match_progress() == 4:
            return Status.SUCCESS

        return Status.FAILURE