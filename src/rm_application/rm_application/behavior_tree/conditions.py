from rm_application.behavior_tree.core import Node, Status

# 判断共享黑板中的导航任务是否为指定任务
class IsNavigationTask(Node):
    def __init__(self, name, get_navigation_task, expected_task):
        super().__init__(name)

        self.get_navigation_task = get_navigation_task
        self.expected_task = expected_task

    def tick(self):
        if self.get_navigation_task() == self.expected_task:
            return Status.SUCCESS

        return Status.FAILURE

# 判断共享黑板中的战术模式是否为指定模式
class IsTacticalMode(Node):
    def __init__(self, name, get_tactical_mode, expected_mode):
        super().__init__(name)

        self.get_tactical_mode = get_tactical_mode
        self.expected_mode = expected_mode

    def tick(self):
        if self.get_tactical_mode() == self.expected_mode:
            return Status.SUCCESS

        return Status.FAILURE

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

# 判断哨兵是否需要进入中央控制区
class NeedEnterCenterZone(Node):
    def __init__(self, name, get_robot_position, get_control_zone):
        super().__init__(name)
        self.get_robot_position = get_robot_position
        self.get_control_zone = get_control_zone

    def tick(self):
        zone = self.get_control_zone()

        # 边界未配置或定位不可用时，暂不允许巡逻
        if not zone["enabled"]:
            return Status.SUCCESS

        robot_position = self.get_robot_position()
        if robot_position is None:
            return Status.SUCCESS

        robot_x, robot_y = robot_position

        inside_zone = (
            zone["x_min"] <= robot_x <= zone["x_max"]
            and zone["y_min"] <= robot_y <= zone["y_max"]
        )

        return Status.FAILURE if inside_zone else Status.SUCCESS

# 判断是否允许在中央控制区内受限追击
class CanChaseInsideCenterZone(Node):
    def __init__(
        self,
        name,
        get_enemy_position,
        get_robot_position,
        get_control_zone,
        is_enemy_within_lost_grace,
        is_chasing,
    ):
        super().__init__(name)

        self.get_enemy_position = get_enemy_position
        self.get_robot_position = get_robot_position
        self.get_control_zone = get_control_zone
        self.is_enemy_within_lost_grace = is_enemy_within_lost_grace
        self.is_chasing = is_chasing

    def tick(self):
        zone = self.get_control_zone()

        # 没有配置中央控制区边界时，不启动受限追击
        if not zone["enabled"]:
            return Status.FAILURE

        robot_position = self.get_robot_position()
        if robot_position is None:
            return Status.FAILURE

        robot_x, robot_y = robot_position
        robot_inside = (
            zone["x_min"] <= robot_x <= zone["x_max"]
            and zone["y_min"] <= robot_y <= zone["y_max"]
        )

        # 哨兵离开中央控制区后立即停止追击
        if not robot_inside:
            return Status.FAILURE

        enemy_position = self.get_enemy_position()

        # 追击中短暂丢失目标时，允许使用现有宽限时间
        if enemy_position is None:
            if (
                self.is_chasing()
                and self.is_enemy_within_lost_grace()
            ):
                return Status.SUCCESS

            return Status.FAILURE

        enemy_x, enemy_y = enemy_position
        enemy_inside = (
            zone["x_min"] <= enemy_x <= zone["x_max"]
            and zone["y_min"] <= enemy_y <= zone["y_max"]
        )

        # 只有哨兵和敌人都在中央控制区内才允许追击
        if enemy_inside:
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

        zone = self.get_control_zone()
        boundary = self.get_chase_boundary()

        # 中央控制区或最大追击边界未配置时，
        # 不允许启动前压离区追击
        if not zone["enabled"] or not boundary["enabled"]:
            return Status.FAILURE

        robot_x, robot_y = robot_position

        # 哨兵到达最大追击边界后立即停止追击
        reached_boundary = (
            robot_x <= boundary["x_min"]
            or robot_x >= boundary["x_max"]
            or robot_y <= boundary["y_min"]
            or robot_y >= boundary["y_max"]
        )

        if reached_boundary:
            return Status.FAILURE

        robot_inside_zone = (
            zone["x_min"] <= robot_x <= zone["x_max"]
            and zone["y_min"] <= robot_y <= zone["y_max"]
        )

        chasing = self.is_chasing()

        # 只能从中央控制区内开始追击
        if not chasing and not robot_inside_zone:
            return Status.FAILURE

        # 追击过程中短暂丢失敌人时，允许等待一小段时间
        if enemy_position is None:
            if chasing and self.is_enemy_within_lost_grace():
                return Status.SUCCESS

            return Status.FAILURE

        # 已经开始追击后，可以离开中央控制区，
        # 直到到达最大追击边界
        return Status.SUCCESS

# 判断比赛是否未在进行
class IsMatchStopped(Node):
    def __init__(self, name, get_match_progress):
        super().__init__(name)

        self.get_match_progress = get_match_progress

    def tick(self):
        # 未收到比赛数据、比赛尚未开始或比赛已经结束时都应停止
        if self.get_match_progress() != 4:
            return Status.SUCCESS

        return Status.FAILURE

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
