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
        get_chase_boundary,
        is_enemy_within_lost_grace,
        is_chasing,
    ):
        super().__init__(name)

        # 获取敌人和底盘的地图坐标
        self.get_enemy_position = get_enemy_position
        self.get_robot_position = get_robot_position

        # 获取控制区开关和边界参数
        self.get_control_zone = get_control_zone

        # 获取最大追击区域开关和边界参数
        # 边界判断使用哨兵自身的地图坐标
        self.get_chase_boundary = get_chase_boundary

        # 判断短暂丢失目标是否仍在允许等待的时间内
        self.is_enemy_within_lost_grace = is_enemy_within_lost_grace

        # 获取追击动作当前是否正在运行
        self.is_chasing = is_chasing

    def tick(self):
        enemy_position = self.get_enemy_position()
        robot_position = self.get_robot_position()

        # 没有底盘位置时无法判断追击边界
        if robot_position is None:
            return Status.FAILURE

        # 最大追击边界判断使用哨兵自身的 map 坐标
        robot_x, robot_y = robot_position

        # 启用最大追击区域后，检查哨兵自身是否到达边界
        boundary = self.get_chase_boundary()

        if boundary["enabled"]:
            reached_boundary = (
                robot_x <= boundary["x_min"]
                or robot_x >= boundary["x_max"]
                or robot_y <= boundary["y_min"]
                or robot_y >= boundary["y_max"]
            )

            # 哨兵到达或超过边界时停止追击
            if reached_boundary:
                return Status.FAILURE

        # 追击过程中短暂丢失敌人时，暂时保持追击状态
        if enemy_position is None:
            if (
                self.is_chasing()
                and self.is_enemy_within_lost_grace()
            ):
                return Status.SUCCESS

            return Status.FAILURE

        # 已经开始追击时，可以离开控制区继续追，
        # 直到哨兵自身到达最大追击边界
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

# 判断占点期间是否需要执行受击闪避
class CanEvade(Node):
    def __init__(
        self,
        name,
        get_evade_enabled,
        is_evading,
        is_occupying,
        get_pending_hit,
        is_inside_control_zone,
    ):
        super().__init__(name)

        # 动态读取闪避开关，之后可以通过 RQT 修改
        self.get_evade_enabled = get_evade_enabled

        # 获取闪避动作和占点动作当前是否正在运行
        self.is_evading = is_evading
        self.is_occupying = is_occupying

        # 获取尚未处理的受击事件
        # 没有待处理事件时应返回 None
        self.get_pending_hit = get_pending_hit

        # 判断机器人当前位置是否位于有效控制区内
        self.is_inside_control_zone = is_inside_control_zone

    def tick(self):
        # 关闭闪避开关后立即停止闪避。
        if not self.get_evade_enabled():
            return Status.FAILURE

        # 闪避已经开始后，不再要求占点动作继续运行，
        # 但机器人必须仍然位于控制区内。
        if self.is_evading():
            if self.is_inside_control_zone():
                return Status.SUCCESS

            return Status.FAILURE

        # 只允许从占点动作进入闪避。
        if not self.is_occupying():
            return Status.FAILURE

        hit_event = self.get_pending_hit()
        if hit_event is None:
            return Status.FAILURE

        # 只有弹丸命中才触发，撞击等原因不触发。
        if hit_event.armor_reason != 0:
            return Status.FAILURE

        # 启动闪避前确认机器人位于控制区内。
        if not self.is_inside_control_zone():
            return Status.FAILURE

        return Status.SUCCESS
