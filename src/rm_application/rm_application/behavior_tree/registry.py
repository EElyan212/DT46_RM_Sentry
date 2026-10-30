import py_trees
from py_trees.ports import BehaviourWithPorts, PortsMixin
from rm_application.behavior_tree.core import Status
from functools import partial
from rm_application.behavior_tree.actions import (
    SetNavigationTask,
    SetPatrolRegion,
    OccupyZone,
    GoResupply,
    ChaseEnemy,
    WaitBeforeRetry,
    PatrolZone,
    SelectAlternativeGoal,
    SpinChassis,
)
from rm_application.behavior_tree.conditions import (
    IsMatchStopped,
    IsMatchRunning,    
    NeedResupply,
    IsTacticalMode,
    NeedEnterCenterZone,
    IsChaseReady,
    CanChase,
    CanChaseInsideCenterZone,
)
from rm_application.behavior_tree.nav_zone import (
    NavigationTask,
    TacticalMode,
    PatrolRegion,
)
from rm_application.behavior_tree.recovery import RecoveryNode

# 将现有 Python 节点接入公共行为树库
class NodeAdapter(BehaviourWithPorts):
    # 当前通过 Python 回调读取共享数据，暂不使用 XML 数据端口
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(self, name, node_factory, **kwargs):
        super().__init__(name=name, **kwargs)

        # 使用传入的创建方法，生成实际执行的原有节点
        self.node = node_factory(name)

    def update(self):
        # 执行原有节点，并转换为公共库的节点状态
        status = self.node.tick()

        return {
            Status.SUCCESS: py_trees.common.Status.SUCCESS,
            Status.FAILURE: py_trees.common.Status.FAILURE,
            Status.RUNNING: py_trees.common.Status.RUNNING,
        }[status]

    def terminate(self, new_status):
        # 公共库打断节点时，调用原有停止逻辑
        if new_status == py_trees.common.Status.INVALID:
            self.node.halt()

# 将 Groot 的反应式顺序节点接到公共库的顺序节点
class ReactiveSequence(PortsMixin, py_trees.composites.Sequence):
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(self, name, children, **kwargs):
        # 每轮从第一个子节点重新检查
        super().__init__(
            name=name,
            memory=False,
            children=children,
            **kwargs,
        )


# 将 Groot 的反应式选择节点接到公共库的选择节点
class ReactiveFallback(PortsMixin, py_trees.composites.Selector):
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(self, name, children, **kwargs):
        # 每轮从最高优先级分支重新检查
        super().__init__(
            name=name,
            memory=False,
            children=children,
            **kwargs,
        )

# 根据导航任务牌切换执行分支，并等待旧导航释放
class NavigationSwitch(PortsMixin, py_trees.composites.Composite):
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(
        self,
        name,
        children,
        get_task,
        can_start,
        cases,
        **kwargs,
    ):
        super().__init__(
            name=name,
            children=children,
            **kwargs,
        )

        # 读取当前导航任务牌
        self.get_task = get_task

        # 判断旧导航是否已经结束，允许启动新任务
        self.can_start = can_start

        # XML 中 cases 的任务顺序，与子节点顺序一一对应
        tasks = [
            NavigationTask(value.strip())
            for value in cases.split(",")
        ]

        if len(tasks) != len(children):
            raise ValueError("任务数量必须与执行分支数量一致")

        if len(set(tasks)) != len(tasks):
            raise ValueError("任务名称不能重复")

        self.task_children = dict(zip(tasks, children))

    def tick(self):
        status = py_trees.common.Status

        if self.status != status.RUNNING:
            self.initialise()

        # 等待期间也重新读取任务牌，以最新任务为准
        child = self.task_children.get(self.get_task())

        # 任务改变时，先打断旧分支，触发原有取消逻辑
        if child is not self.current_child:
            if self.current_child is not None:
                self.current_child.stop(status.INVALID)

            self.current_child = None

        # 只有进入新分支前才等待导航释放
        # 正在执行的分支继续正常 tick
        if self.current_child is None:
            if not self.can_start():
                self.status = status.RUNNING
                yield self
                return

            self.current_child = child

        # 停止任务没有执行分支，确认停止后返回 FAILURE
        # 保持原 Switch 的返回约定
        if child is None:
            self.stop(status.FAILURE)
            yield self
            return

        # 执行选中的分支，并向上传递它的状态
        for visited in child.tick():
            if visited is child:
                if child.status == status.RUNNING:
                    self.status = status.RUNNING
                else:
                    self.stop(child.status)

            yield visited

        yield self

# 每轮执行所有分支，保留原主树的持续并行规则
class ContinuousParallel(PortsMixin, py_trees.composites.Composite):
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(self, name, children, **kwargs):
        super().__init__(name=name, children=children, **kwargs)

    def tick(self):
        status = py_trees.common.Status

        if self.status != status.RUNNING:
            self.initialise()

        # 每个分支每轮都执行，不跳过上轮成功或失败的分支
        for child in self.children:
            yield from child.tick()

        self.current_child = next(
            (child for child in self.children
             if child.status == status.RUNNING),
            None,
        )

        # 任意分支仍在运行，主树就继续运行
        if self.current_child is not None:
            self.status = status.RUNNING

        elif all(child.status == status.SUCCESS for child in self.children):
            self.stop(status.SUCCESS)

        else:
            self.stop(status.FAILURE)

        yield self

# 创建 XML 节点名称与 Python 节点之间的对应关系
def create_node_registry(node):
    # 为现有节点准备创建方法，并接入公共库
    def adapt(node_class, *args):
        return partial(
            NodeAdapter,
            node_factory=lambda name: node_class(name, *args),
        )

    # 本次加载共用一个巡逻实例，备用点交回该实例保存
    patrol_action = PatrolZone(
        "执行区域巡逻",
        node.get_patrol_region,
        node.start_patrol,
        node.get_patrol_result,
        node.stop_patrol,
        get_patrol_area=node.get_patrol_area,
        get_robot_position=node.get_robot_position,
        patrol_min_goal_distance=node.patrol_min_goal_distance,
        is_navigation_stopped=node.is_navigation_stopped,
    )

    return {
        # 正常回血动作，恢复流程由 XML 中的父节点管理
        "GoResupply": adapt(
            GoResupply,
            node.start_resupply,
            node.get_resupply_result,
            node.stop_resupply,
        ),

        # 追击恢复控制：额外重试一次，耗尽后通知上层进入冷却
        "ChaseRecovery": partial(
            RecoveryNode,
            max_retries=1,
            on_exhausted=node.report_chase_exhausted,
        ),

        # 正常追击动作，继续调用已有的追击服务接口
        "ChaseEnemy": adapt(
            ChaseEnemy,
            node.start_chase,
            node.get_chase_result,
            node.stop_chase,
        ),

        # 占点、回血、巡逻共用的恢复控制
        "NavigationRecovery": partial(
            RecoveryNode,
            max_retries=node.recovery_max_retries,
            # 读取正常动作的到达计数，用于清零连续失败次数
            read_progress=lambda action: action.node.goal_reached_count,
        ),

        # 正常占点动作，恢复流程交给 XML 中的父节点管理
        "OccupyZone": adapt(
            OccupyZone,
            node.start_occupy,
            node.get_occupy_result,
            node.stop_occupy,
        ),

        # 导航失败后的等待动作，等待时间沿用配置
        "WaitBeforeRetry": adapt(
            WaitBeforeRetry,
            node.recovery_wait_seconds,
        ),

        # 正常巡逻动作，与备用点选择节点使用同一个巡逻实例
        "PatrolZone": partial(
            NodeAdapter,
            node_factory=lambda name: patrol_action,
        ),

        # 准备备用点，只保存坐标，由原巡逻动作发送导航
        "SelectAlternativeGoal": partial(
            NodeAdapter,
            node_factory=lambda name: SelectAlternativeGoal(
                name,
                get_area=lambda: node.get_patrol_area(
                    node.get_patrol_region()
                ),
                get_failed_position=lambda: (
                    node.get_last_navigation_target("patrol")
                ),
                save_target=patrol_action.save_recovery_target,
                recovery_min_goal_distance=(
                    node.recovery_min_goal_distance
                ),
                recovery_sample_attempts=(
                    node.recovery_sample_attempts
                ),
            ),
        ),

        # Groot 控制节点名称与公共库实现的对应关系
        "ReactiveSequence": ReactiveSequence,
        "ReactiveFallback": ReactiveFallback,
        "ContinuousParallel": ContinuousParallel,

        # 按任务牌切换导航分支，启动前等待旧导航释放
        "NavigationSwitch": partial(
            NavigationSwitch,
            get_task=node.get_navigation_task,
            can_start=node.is_navigation_stopped,
        ),

        # 判断比赛是否停止
        "IsMatchStopped": adapt(
            IsMatchStopped,
            node.get_match_progress,
        ),

        # 判断是否需要回血，沿用原来的开始和结束阈值
        "NeedResupply": adapt(
            NeedResupply,
            node.get_hp,
            node.hp_limit,
            node.hp_up,
        ),

        # 将导航任务牌设置为停止
        "SelectStop": adapt(
            SetNavigationTask,
            node.set_navigation_task,
            NavigationTask.STOP,
        ),

        # 将导航任务牌设置为回血
        "SelectResupply": adapt(
            SetNavigationTask,
            node.set_navigation_task,
            NavigationTask.RESUPPLY,
        ),

        # 将导航任务牌设置为占点
        "SelectOccupy": adapt(
            SetNavigationTask,
            node.set_navigation_task,
            NavigationTask.OCCUPY_CENTER,
        ),

        # 判断当前是否使用防守模式
        "IsDefenseMode": adapt(
            IsTacticalMode,
            node.get_tactical_mode,
            TacticalMode.DEFENSE,
        ),

        # 判断当前是否使用中立模式
        "IsNeutralMode": adapt(
            IsTacticalMode,
            node.get_tactical_mode,
            TacticalMode.NEUTRAL,
        ),

        # 判断当前是否使用前压模式
        "IsPressureMode": adapt(
            IsTacticalMode,
            node.get_tactical_mode,
            TacticalMode.PRESSURE,
        ),

        # 判断是否需要返回中央控制区
        "NeedEnterCenterZone": adapt(
            NeedEnterCenterZone,
            node.get_robot_position,
            node.get_control_zone,
        ),

        # 判断追击失败后的冷却是否已经结束
        "IsChaseReady": adapt(
            IsChaseReady,
            node.is_chase_ready,
        ),

        # 中立模式：判断是否允许在中央控制区内追击
        "CanChaseInsideCenterZone": adapt(
            CanChaseInsideCenterZone,
            node.get_enemy_position,
            node.get_robot_position,
            node.get_control_zone,
            node.is_enemy_within_lost_grace,
            lambda: (
                node.get_navigation_task()
                == NavigationTask.CHASE
            ),
        ),

        # 前压模式：沿用原有追击开启条件和最大追击边界
        "CanChase": adapt(
            CanChase,
            node.get_enemy_position,
            node.get_robot_position,
            node.get_control_zone,
            node.get_chase_boundary,
            node.is_enemy_within_lost_grace,
            lambda: (
                node.get_navigation_task()
                == NavigationTask.CHASE
            ),
        ),

        # 将导航任务牌设置为追击
        "SelectChase": adapt(
            SetNavigationTask,
            node.set_navigation_task,
            NavigationTask.CHASE,
        ),

        # 将导航任务牌设置为巡逻
        "SelectPatrol": adapt(
            SetNavigationTask,
            node.set_navigation_task,
            NavigationTask.PATROL,
        ),

        # 防守模式使用中央控制区内偏己方的巡逻区域
        "SetHomePatrolRegion": adapt(
            SetPatrolRegion,
            node.set_patrol_region,
            PatrolRegion.HOME,
        ),

        # 中立模式使用中央控制区内居中的巡逻区域
        "SetCenterPatrolRegion": adapt(
            SetPatrolRegion,
            node.set_patrol_region,
            PatrolRegion.CENTER,
        ),

        # 前压模式使用中央控制区内偏敌方的巡逻区域
        "SetEnemyPatrolRegion": adapt(
            SetPatrolRegion,
            node.set_patrol_region,
            PatrolRegion.ENEMY,
        ),

        # 判断比赛是否正在进行
        "IsMatchRunning": adapt(
            IsMatchRunning,
            node.get_match_progress,
        ),

        # 比赛进行时持续开启小陀螺，被打断时关闭
        "SpinChassis": adapt(
            SpinChassis,
            node.start_spin,
            node.stop_spin,
        ),
    }
