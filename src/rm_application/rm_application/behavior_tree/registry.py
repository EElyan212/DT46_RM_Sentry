from functools import partial
import py_trees
from py_trees.ports import PortsMixin
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
    # 保存本棵树实际创建的巡逻实例，备用点回调使用同一实例
    patrol_action = {}

    return {
        # 正常回血动作，恢复流程由 XML 中的父节点管理
        "GoResupply": partial(
            GoResupply,
            start_resupply=node.start_resupply,
            get_resupply_result=node.get_resupply_result,
            stop_resupply=node.stop_resupply,
        ),

        # 追击恢复控制：额外重试一次，耗尽后通知上层进入冷却
        "ChaseRecovery": partial(
            RecoveryNode,
            max_retries=1,
            on_exhausted=node.report_chase_exhausted,
        ),

        # 正常追击动作，继续调用已有的追击服务接口
        "ChaseEnemy": partial(
            ChaseEnemy,
            start_chase=node.start_chase,
            get_chase_result=node.get_chase_result,
            stop_chase=node.stop_chase,
        ),

        # 占点、回血、巡逻共用的恢复控制
        "NavigationRecovery": partial(
            RecoveryNode,
            max_retries=node.recovery_max_retries,
            # 三个动作都已迁移，直接读取动作自己的到达计数
            read_progress=lambda action: action.goal_reached_count,
        ),

        # 正常占点动作，恢复流程交给 XML 中的父节点管理
        "OccupyZone": partial(
            OccupyZone,
            start_occupy=node.start_occupy,
            get_occupy_result=node.get_occupy_result,
            stop_occupy=node.stop_occupy,
        ),

        # 导航失败后的等待动作，等待时间沿用配置
        "WaitBeforeRetry": partial(
            WaitBeforeRetry,
            wait_seconds=node.recovery_wait_seconds,
        ),

        # 创建巡逻节点，并记录实际运行的实例
        "PatrolZone": partial(
            PatrolZone,
            get_patrol_region=node.get_patrol_region,
            start_patrol=node.start_patrol,
            get_patrol_result=node.get_patrol_result,
            stop_patrol=node.stop_patrol,
            get_patrol_area=node.get_patrol_area,
            get_robot_position=node.get_robot_position,
            patrol_min_goal_distance=node.patrol_min_goal_distance,
            is_navigation_stopped=node.is_navigation_stopped,
            on_created=lambda action: patrol_action.update(node=action),
        ),

        # 准备备用点，只保存坐标，由原巡逻动作发送导航
        "SelectAlternativeGoal": partial(
            SelectAlternativeGoal,
            get_area=lambda: node.get_patrol_area(
                node.get_patrol_region()
            ),
            get_failed_position=lambda: (
                node.get_last_navigation_target("patrol")
            ),
            save_target=lambda target: (
                patrol_action["node"].save_recovery_target(target)
            ),
            recovery_min_goal_distance=node.recovery_min_goal_distance,
            recovery_sample_attempts=node.recovery_sample_attempts,
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

        # 直接创建公共库节点，并提供读取比赛阶段的方法
        "IsMatchStopped": partial(
            IsMatchStopped,
            get_match_progress=node.get_match_progress,
        ),

        # 直接创建公共库回血条件，沿用原来的双阈值
        "NeedResupply": partial(
            NeedResupply,
            get_hp=node.get_hp,
            hp_limit=node.hp_limit,
            hp_up=node.hp_up,
        ),

        # 将导航任务牌设置为停止
        "SelectStop": partial(
            SetNavigationTask,
            set_navigation_task=node.set_navigation_task,
            task=NavigationTask.STOP,
        ),

        # 将导航任务牌设置为回血
        "SelectResupply": partial(
            SetNavigationTask,
            set_navigation_task=node.set_navigation_task,
            task=NavigationTask.RESUPPLY,
        ),

        # 将导航任务牌设置为占点
        "SelectOccupy": partial(
            SetNavigationTask,
            set_navigation_task=node.set_navigation_task,
            task=NavigationTask.OCCUPY_CENTER,
        ),

        # 判断当前是否使用防守模式
        "IsDefenseMode": partial(
            IsTacticalMode,
            get_tactical_mode=node.get_tactical_mode,
            expected_mode=TacticalMode.DEFENSE,
        ),

        # 判断当前是否使用中立模式
        "IsNeutralMode": partial(
            IsTacticalMode,
            get_tactical_mode=node.get_tactical_mode,
            expected_mode=TacticalMode.NEUTRAL,
        ),

        # 判断当前是否使用前压模式
        "IsPressureMode": partial(
            IsTacticalMode,
            get_tactical_mode=node.get_tactical_mode,
            expected_mode=TacticalMode.PRESSURE,
        ),

        # 直接创建公共库条件，判断是否需要返回控制区
        "NeedEnterCenterZone": partial(
            NeedEnterCenterZone,
            get_robot_position=node.get_robot_position,
            get_control_zone=node.get_control_zone,
        ),

        # 直接创建公共库条件，判断追击冷却是否结束
        "IsChaseReady": partial(
            IsChaseReady,
            is_chase_ready=node.is_chase_ready,
        ),
        # 中立模式：判断是否允许在中央控制区内追击
        "CanChaseInsideCenterZone": partial(
            CanChaseInsideCenterZone,
            get_enemy_position=node.get_enemy_position,
            get_robot_position=node.get_robot_position,
            get_control_zone=node.get_control_zone,
            is_enemy_within_lost_grace=node.is_enemy_within_lost_grace,
            is_chasing=lambda: (
                node.get_navigation_task()
                == NavigationTask.CHASE
            ),
        ),

        # 前压模式：沿用原有追击开启条件和最大追击边界
        "CanChase": partial(
            CanChase,
            get_enemy_position=node.get_enemy_position,
            get_robot_position=node.get_robot_position,
            get_control_zone=node.get_control_zone,
            get_chase_boundary=node.get_chase_boundary,
            is_enemy_within_lost_grace=node.is_enemy_within_lost_grace,
            is_chasing=lambda: (
                node.get_navigation_task()
                == NavigationTask.CHASE
            ),
        ),

        # 将导航任务牌设置为追击
        "SelectChase": partial(
            SetNavigationTask,
            set_navigation_task=node.set_navigation_task,
            task=NavigationTask.CHASE,
        ),

        # 将导航任务牌设置为巡逻
        "SelectPatrol": partial(
            SetNavigationTask,
            set_navigation_task=node.set_navigation_task,
            task=NavigationTask.PATROL,
        ),

        # 防守模式使用中央控制区内偏己方的巡逻区域
        "SetHomePatrolRegion": partial(
            SetPatrolRegion,
            set_patrol_region=node.set_patrol_region,
            patrol_region=PatrolRegion.HOME,
        ),

        # 中立模式使用中央控制区内中部的巡逻区域
        "SetCenterPatrolRegion": partial(
            SetPatrolRegion,
            set_patrol_region=node.set_patrol_region,
            patrol_region=PatrolRegion.CENTER,
        ),

        # 前压模式使用中央控制区内偏敌方的巡逻区域
        "SetEnemyPatrolRegion": partial(
            SetPatrolRegion,
            set_patrol_region=node.set_patrol_region,
            patrol_region=PatrolRegion.ENEMY,
        ),

        # 直接创建公共库节点，并提供读取比赛阶段的方法
        "IsMatchRunning": partial(
            IsMatchRunning,
            get_match_progress=node.get_match_progress,
        ),

        # 比赛进行时持续开启小陀螺，被打断时关闭
        "SpinChassis": partial(
            SpinChassis,
            start_spin=node.start_spin,
            stop_spin=node.stop_spin,
        ),
    }
