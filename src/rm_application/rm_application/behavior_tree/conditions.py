import math
import time
import py_trees
from py_trees.ports import BehaviourWithPorts
from rm_application.behavior_tree.nav_zone import NavigationTask, TacticalMode

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

# 判断是否需要切换战术模式，成立时发布待应用候选
class ShouldSwitchTacticalMode(BehaviourWithPorts):
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(
        self,
        name,
        get_tactical_mode,
        get_tactical_mode_since,
        set_tactical_candidate,
        get_navigation_task,
        get_tactical_inputs,
        mode_source,
        confirm_seconds,
        min_hold_seconds,
        score_config,
        **kwargs,
    ):
        super().__init__(name=name, **kwargs)

        # 读取正式模式及其开始时间，只发布候选，不修改正式模式
        self.get_tactical_mode = get_tactical_mode
        self.get_tactical_mode_since = get_tactical_mode_since
        self.set_tactical_candidate = set_tactical_candidate
        self.get_navigation_task = get_navigation_task
        self.get_tactical_inputs = get_tactical_inputs
        self.mode_source = mode_source
        self.confirm_seconds = float(confirm_seconds)
        self.min_hold_seconds = float(min_hold_seconds)

        # 因子、权重和归一化上限按相同顺序对应
        self.score_factors = list(score_config["factors"])
        self.score_weights = list(map(float, score_config["weights"]))
        self.score_maxima = list(map(float, score_config["maxima"]))

        # 掉血统计窗口、满压力速率和消息接收超时
        self.damage_window_seconds = float(
            score_config.get("damage_window_seconds", 2.0)
        )
        self.damage_full_rate = float(
            score_config.get("damage_full_rate", 50.0)
        )
        self.damage_input_timeout = float(
            score_config.get("damage_input_timeout", 1.0)
        )

        # 防守和前压采用不同的进入、退出阈值
        self.defense_enter = float(score_config["defense_enter"])
        self.defense_exit = float(score_config["defense_exit"])
        self.pressure_exit = float(score_config["pressure_exit"])
        self.pressure_enter = float(score_config["pressure_enter"])
        self.validate_config()

        # 候选确认计时，以及最近观察到的正式模式
        self.candidate_mode = None
        self.candidate_since = None
        self.observed_mode = None
        self.observed_mode_since = None

        # 保存最近一次计算结果，None 表示本轮无法评分
        self.last_score = None

    def validate_config(self):
        # 时间必须是有限的非负数
        for value in (self.confirm_seconds, self.min_hold_seconds):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("战术确认时间和保持时间必须是有限非负数")

        # 每个因子只能出现一次，且三个配置列表长度一致
        if (
            not self.score_factors
            or not all(isinstance(key, str) and key for key in self.score_factors)
            or len(set(self.score_factors)) != len(self.score_factors)
            or len(self.score_factors) != len(self.score_weights)
            or len(self.score_factors) != len(self.score_maxima)
        ):
            raise ValueError("评分因子须非空且不重复，因子、权重、上限数量须一致")

        for weight, maximum in zip(self.score_weights, self.score_maxima):
            if not math.isfinite(weight) or weight < 0.0:
                raise ValueError("评分权重必须是有限非负数")
            if not math.isfinite(maximum) or maximum < 0.0:
                raise ValueError("归一化上限必须是有限非负数")
            if weight > 0.0 and maximum <= 0.0:
                raise ValueError("参与评分的因子必须配置正数上限")

        # 掉血统计参数必须是有限正数
        for value in (
            self.damage_window_seconds,
            self.damage_full_rate,
            self.damage_input_timeout,
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    "掉血统计窗口、满压力速率和消息超时必须为有限正数"
                )

        self.total_weight = sum(self.score_weights)
        if not math.isfinite(self.total_weight):
            raise ValueError("评分权重总和必须是有限数")

        if not (
            0.0 <= self.defense_enter < self.defense_exit
            <= self.pressure_exit < self.pressure_enter <= 1.0
        ):
            raise ValueError(
                "阈值须满足 0 <= 防守进入 < 防守退出"
                " <= 前压退出 < 前压进入 <= 1"
            )

    def clear_candidate(self):
        # 只清除候选，不修改正在采用的战术模式
        self.candidate_mode = None
        self.candidate_since = None

    def calculate_damage_safety(self, samples):
        # 首次接收前或样本不足时，保留未知
        if not isinstance(samples, (tuple, list)) or len(samples) < 2:
            return None

        now = time.monotonic()
        cutoff = now - self.damage_window_seconds
        latest_time = samples[-1][0]

        # 接收中断或时间异常时，不把旧血量当成当前安全
        if not 0.0 <= now - latest_time <= self.damage_input_timeout:
            return None

        # 寻找窗口开始前最近的一个样本，作为计算掉血的基准
        start = None
        for index, (sample_time, _, _) in enumerate(samples):
            if sample_time > now:
                return None
            if sample_time <= cutoff:
                start = index
        if start is None:
            return None

        recent = samples[start:]
        lost_hp = 0.0

        # 只使用比赛进行中、血量为正的连续记录
        # 0 血在这里表示不能继续比较，不据此判定战亡
        for sample_time, hp, progress in recent:
            if (
                not math.isfinite(sample_time)
                or not math.isfinite(hp)
                or hp <= 0
                or progress != 4
            ):
                return None

        for previous, current in zip(recent, recent[1:]):
            interval = current[0] - previous[0]
            if not 0.0 < interval <= self.damage_input_timeout:
                return None

            # 累计每次下降量，期间回血不会抵消已观察到的掉血
            lost_hp += max(previous[1] - current[1], 0.0)

        loss_rate = lost_hp / self.damage_window_seconds
        pressure = min(loss_rate / self.damage_full_rate, 1.0)
        return 1.0 - pressure

    def calculate_score(self, inputs):
        # 全部权重为 0 时，不产生评分
        if self.total_weight == 0.0:
            return None

        weighted_sum = 0.0
        for key, weight, maximum in zip(
            self.score_factors, self.score_weights, self.score_maxima
        ):
            # 权重为 0 的输入不参与，也不要求它已有数据
            if weight == 0.0:
                continue

            # 掉血越快越危险，先转换成越大越安全的 0～1 分数
            if key == "damage_safety":
                value = self.calculate_damage_safety(
                    inputs.get("hp_samples")
                )
            else:
                value = inputs.get(key)
                
            if value is None or isinstance(value, bool):
                return None
            try:
                value = float(value)
            except (TypeError, ValueError, OverflowError):
                return None
            if not math.isfinite(value):
                return None

            # 先换算到 0～1，再按权重求平均
            normalized = min(1.0, max(0.0, value / maximum))
            weighted_sum += weight * normalized

        return weighted_sum / self.total_weight

    def evaluate_candidate(self, current_mode, inputs):
        self.last_score = self.calculate_score(inputs)
        if self.last_score is None:
            self.feedback_message = "暂无评分：权重未配置或启用项缺少有效数值"
            return None

        score = self.last_score
        self.feedback_message = f"战术评分={score:.3f}"

        # 超过两端进入阈值时，直接产生对应候选
        if score < self.defense_enter:
            return TacticalMode.DEFENSE
        if score > self.pressure_enter:
            return TacticalMode.PRESSURE

        # 处于迟滞区间时保留原模式，避免边界附近反复切换
        if current_mode == TacticalMode.DEFENSE and score <= self.defense_exit:
            return TacticalMode.DEFENSE
        if current_mode == TacticalMode.PRESSURE and score >= self.pressure_exit:
            return TacticalMode.PRESSURE

        return TacticalMode.NEUTRAL

    def update(self):
        # 每轮先清除旧的待应用结果，避免使用过期候选
        self.set_tactical_candidate(None)
        now = time.monotonic()
        current_mode = self.get_tactical_mode()
        mode_since = self.get_tactical_mode_since()

        # 正式模式或其时间戳改变后，重新确认候选
        if (
            current_mode != self.observed_mode
            or mode_since != self.observed_mode_since
        ):
            self.observed_mode = current_mode
            self.observed_mode_since = mode_since
            self.clear_candidate()

        # 手动模式不产生自动切换请求
        if self.mode_source != "auto":
            self.clear_candidate()
            self.last_score = None
            return py_trees.common.Status.FAILURE

        # 从停止或回血任务恢复后，重新累计确认时间
        if self.get_navigation_task() in (
            NavigationTask.STOP, NavigationTask.RESUPPLY,
        ):
            self.clear_candidate()

        candidate = self.evaluate_candidate(
            current_mode, self.get_tactical_inputs()
        )
        if not isinstance(candidate, TacticalMode) or candidate == current_mode:
            self.clear_candidate()
            return py_trees.common.Status.FAILURE

        if candidate != self.candidate_mode:
            self.candidate_mode = candidate
            self.candidate_since = now

        confirmed = now - self.candidate_since >= self.confirm_seconds
        held_long_enough = now - mode_since >= self.min_hold_seconds
        if confirmed and held_long_enough:
            # 条件成立，只交出候选；正式模式由后续 Action 修改
            self.set_tactical_candidate(candidate)
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE