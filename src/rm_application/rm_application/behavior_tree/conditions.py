import py_trees
from py_trees.ports import BehaviourWithPorts

# 判断共享黑板中的战术模式是否为指定模式
class IsTacticalMode(BehaviourWithPorts):
    # 通过 Python 回调读取战术模式，不使用 XML 数据端口
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(
        self,
        name,
        get_tactical_mode,
        expected_mode,
        **kwargs,
    ):
        super().__init__(name=name, **kwargs)

        # 获取当前战术模式的方法
        self.get_tactical_mode = get_tactical_mode

        # 当前分支要求的战术模式
        self.expected_mode = expected_mode

    def update(self):
        if self.get_tactical_mode() == self.expected_mode:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE
# 判断是否需要回补给
class NeedResupply(BehaviourWithPorts):
    # 通过 Python 回调读取血量，不使用 XML 数据端口
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(self, name, get_hp, hp_limit, hp_up, **kwargs):
        super().__init__(name=name, **kwargs)

        # 获取当前血量的方法
        self.get_hp = get_hp

        # 开始和结束回补给的血量
        self.hp_limit = hp_limit
        self.hp_up = hp_up

        # 保存回血状态，避免血量在两个阈值之间时反复切换
        self.resupplying = False

    def update(self):
        current_hp = self.get_hp()

        # 血量过低，开始回补给
        if current_hp <= self.hp_limit:
            self.resupplying = True

        # 血量恢复，结束回补给
        elif current_hp >= self.hp_up:
            self.resupplying = False

        if self.resupplying:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE

# 判断哨兵是否需要进入中央控制区
class NeedEnterCenterZone(BehaviourWithPorts):
    # 通过 Python 回调读取位置和区域，不使用 XML 数据端口
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(
        self,
        name,
        get_robot_position,
        get_control_zone,
        **kwargs,
    ):
        super().__init__(name=name, **kwargs)

        # 获取哨兵位置和控制区配置的方法
        self.get_robot_position = get_robot_position
        self.get_control_zone = get_control_zone

    def update(self):
        zone = self.get_control_zone()

        # 边界未配置时，继续选择进入控制区任务
        if not zone["enabled"]:
            return py_trees.common.Status.SUCCESS

        robot_position = self.get_robot_position()

        # 定位不可用时，暂不切换为区内巡逻
        if robot_position is None:
            return py_trees.common.Status.SUCCESS

        robot_x, robot_y = robot_position

        inside_zone = (
            zone["x_min"] <= robot_x <= zone["x_max"]
            and zone["y_min"] <= robot_y <= zone["y_max"]
        )

        # 已在区内，交给后续分支选择巡逻或受限追击
        if inside_zone:
            return py_trees.common.Status.FAILURE

        # 位于区外，需要进入或返回控制区
        return py_trees.common.Status.SUCCESS

# 判断是否允许在中央控制区内受限追击
class CanChaseInsideCenterZone(BehaviourWithPorts):
    # 通过 Python 回调读取追击条件，不使用 XML 数据端口
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(
        self,
        name,
        get_enemy_position,
        get_robot_position,
        get_control_zone,
        is_enemy_within_lost_grace,
        is_chasing,
        **kwargs,
    ):
        super().__init__(name=name, **kwargs)

        # 获取敌人、哨兵位置和控制区配置的方法
        self.get_enemy_position = get_enemy_position
        self.get_robot_position = get_robot_position
        self.get_control_zone = get_control_zone

        # 检查目标丢失宽限和当前追击状态的方法
        self.is_enemy_within_lost_grace = is_enemy_within_lost_grace
        self.is_chasing = is_chasing

    def update(self):
        zone = self.get_control_zone()

        # 没有配置中央控制区边界时，不启动受限追击
        if not zone["enabled"]:
            return py_trees.common.Status.FAILURE

        robot_position = self.get_robot_position()
        if robot_position is None:
            return py_trees.common.Status.FAILURE

        robot_x, robot_y = robot_position
        robot_inside = (
            zone["x_min"] <= robot_x <= zone["x_max"]
            and zone["y_min"] <= robot_y <= zone["y_max"]
        )

        # 哨兵离开中央控制区后立即停止追击
        if not robot_inside:
            return py_trees.common.Status.FAILURE

        enemy_position = self.get_enemy_position()

        # 追击中短暂丢失目标时，允许使用现有宽限时间
        if enemy_position is None:
            if (
                self.is_chasing()
                and self.is_enemy_within_lost_grace()
            ):
                return py_trees.common.Status.SUCCESS

            return py_trees.common.Status.FAILURE

        enemy_x, enemy_y = enemy_position
        enemy_inside = (
            zone["x_min"] <= enemy_x <= zone["x_max"]
            and zone["y_min"] <= enemy_y <= zone["y_max"]
        )

        # 只有哨兵和敌人都在中央控制区内才允许追击
        if enemy_inside:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE


# 判断前压模式下是否允许追击敌人
class CanChase(BehaviourWithPorts):
    # 通过 Python 回调读取追击条件，不使用 XML 数据端口
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(
        self,
        name,
        get_enemy_position,
        get_robot_position,
        get_control_zone,
        get_chase_boundary,
        is_enemy_within_lost_grace,
        is_chasing,
        **kwargs,
    ):
        super().__init__(name=name, **kwargs)

        # 获取敌人和底盘的地图坐标
        self.get_enemy_position = get_enemy_position
        self.get_robot_position = get_robot_position

        # 获取控制区开关和边界参数
        self.get_control_zone = get_control_zone

        # 获取最大追击区域配置，使用哨兵位置判断边界
        self.get_chase_boundary = get_chase_boundary

        # 检查目标丢失宽限和当前追击任务状态
        self.is_enemy_within_lost_grace = is_enemy_within_lost_grace
        self.is_chasing = is_chasing

    def update(self):
        enemy_position = self.get_enemy_position()
        robot_position = self.get_robot_position()

        # 没有底盘位置时无法判断追击边界
        if robot_position is None:
            return py_trees.common.Status.FAILURE

        zone = self.get_control_zone()
        boundary = self.get_chase_boundary()

        # 中央控制区或最大追击边界未配置时，不允许追击
        if not zone["enabled"] or not boundary["enabled"]:
            return py_trees.common.Status.FAILURE

        robot_x, robot_y = robot_position

        # 哨兵到达最大追击边界后立即停止追击
        reached_boundary = (
            robot_x <= boundary["x_min"]
            or robot_x >= boundary["x_max"]
            or robot_y <= boundary["y_min"]
            or robot_y >= boundary["y_max"]
        )

        if reached_boundary:
            return py_trees.common.Status.FAILURE

        robot_inside_zone = (
            zone["x_min"] <= robot_x <= zone["x_max"]
            and zone["y_min"] <= robot_y <= zone["y_max"]
        )

        chasing = self.is_chasing()

        # 只能从中央控制区内开始追击
        if not chasing and not robot_inside_zone:
            return py_trees.common.Status.FAILURE

        # 追击中短暂丢失敌人时，沿用现有宽限时间
        if enemy_position is None:
            if chasing and self.is_enemy_within_lost_grace():
                return py_trees.common.Status.SUCCESS

            return py_trees.common.Status.FAILURE

        # 追击开始后，沿用最大追击边界内允许离区的规则
        return py_trees.common.Status.SUCCESS

# 判断比赛是否未在进行
class IsMatchStopped(BehaviourWithPorts):
    # 通过 Python 回调读取比赛阶段，不使用 XML 数据端口
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(self, name, get_match_progress, **kwargs):
        super().__init__(name=name, **kwargs)

        # 获取当前比赛阶段的方法
        self.get_match_progress = get_match_progress

    def update(self):
        # 未收到比赛数据、比赛尚未开始或已经结束时停止
        if self.get_match_progress() != 4:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE

# 判断比赛是否正在进行
class IsMatchRunning(BehaviourWithPorts):
    # 通过 Python 回调读取比赛阶段，不使用 XML 数据端口
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(self, name, get_match_progress, **kwargs):
        super().__init__(name=name, **kwargs)

        # 获取当前比赛阶段的方法
        self.get_match_progress = get_match_progress

    def update(self):
        # match_progress 等于 4 表示比赛正在进行
        if self.get_match_progress() == 4:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE

# 判断追击失败后的冷却是否已经结束
class IsChaseReady(BehaviourWithPorts):
    # 通过 Python 回调检查冷却状态，不使用 XML 数据端口
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(self, name, is_chase_ready, **kwargs):
        super().__init__(name=name, **kwargs)

        # 检查是否已经允许再次追击的方法
        self.is_chase_ready = is_chase_ready

    def update(self):
        # 冷却结束，允许继续检查其他追击条件
        if self.is_chase_ready():
            return py_trees.common.Status.SUCCESS

        # 冷却期间，让任务选择树继续检查占点或巡逻
        return py_trees.common.Status.FAILURE