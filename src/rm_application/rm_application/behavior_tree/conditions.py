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


# 判断是否正在跟踪敌人
class HasTrackedEnemy(Node):
    def __init__(self, name, get_tracked):
        super().__init__(name)
        self.get_tracked = get_tracked

    def tick(self):
        if self.get_tracked():
            return Status.SUCCESS

        return Status.FAILURE


# 守点优先的保守追击条件。位置均使用同一坐标系；没有可用目标时返回 None。
class CanChase(Node):
    def __init__(self, name, get_robot_position, get_enemy_position,
                 control_center, max_distance_from_control, attack_range):
        super().__init__(name)
        self.get_robot_position = get_robot_position
        self.get_enemy_position = get_enemy_position
        self.control_center = control_center
        self.max_distance_from_control = max_distance_from_control
        self.attack_range = attack_range

    def tick(self):
        enemy_position = self.get_enemy_position()
        if enemy_position is None:
            return Status.FAILURE

        robot_position = self.get_robot_position()
        robot_to_enemy = math.dist(robot_position, enemy_position)
        robot_to_control = math.dist(robot_position, self.control_center)
        enemy_to_control = math.dist(enemy_position, self.control_center)

        if (robot_to_enemy <= self.attack_range
                or robot_to_control >= self.max_distance_from_control
                or enemy_to_control >= self.max_distance_from_control):
            return Status.FAILURE

        return Status.SUCCESS
