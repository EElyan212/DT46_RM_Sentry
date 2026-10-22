from rm_application.behavior_tree.core import (
    Sequence,
    Selector,
    Parallel,
)
from rm_application.behavior_tree.conditions import (
    NeedResupply,
    CanChase,
    IsMatchRunning,
    IsNavigationTask,
)
from rm_application.behavior_tree.actions import (
    GoResupply,
    ChaseEnemy,
    OccupyZone,
    SpinChassis,
    SetNavigationTask,
)
from rm_application.behavior_tree.nav_zone import NavigationTask


# 负责创建和组装整棵哨兵行为树
class BehaviorTreeBuilder:
    def __init__(self, node):
        # ROS 主节点，提供参数、实时数据和动作方法
        self.node = node

    def build(self):
        # 创建底盘平移和底盘旋转两条并行分支
        translation_behavior = self._build_translation_behavior()
        spin_behavior = self._build_spin_behavior()

        # 根节点同时运行平移和小陀螺
        return Parallel(
            "哨兵行为树",
            [
                translation_behavior,
                spin_behavior,
            ],
        )

    def _build_translation_behavior(self):
        # 底盘平移行为按优先级选择
        translation_selector = Selector(
            "底盘平移选择",
            [
                self._build_resupply_behavior(),
                self._build_chase_behavior(),
                self._build_occupy_behavior(),
            ],
        )

        # 比赛进行时才允许底盘移动
        return Sequence(
            "底盘平移行为",
            [
                IsMatchRunning(
                    "比赛进行中（平移）",
                    self.node.get_match_progress,
                ),
                translation_selector,
            ],
        )

    def _build_resupply_task_selection(self):
        # 血量满足回补给条件时，把唯一导航任务牌改为回补给
        return Sequence(
            "选择回补给任务",
            [
                NeedResupply(
                    "判断是否需要回补给",
                    self.node.get_hp,
                    self.node.hp_limit,
                    self.node.hp_up,
                ),
                SetNavigationTask(
                    "任务牌改为回补给",
                    self.node.set_navigation_task,
                    NavigationTask.RESUPPLY,
                ),
            ],
        )

    def _build_chase_task_selection(self):
        # 满足追击条件时，把唯一导航任务牌改为追击
        can_chase = CanChase(
            "判断是否允许追击",
            self.node.get_enemy_position,
            self.node.get_robot_position,
            self.node.get_control_zone,
            self.node.get_chase_boundary,
            self.node.is_enemy_within_lost_grace,
            lambda: (
                self.node.get_navigation_task()
                == NavigationTask.CHASE
            ),
        )

        return Sequence(
            "选择追击任务",
            [
                can_chase,
                SetNavigationTask(
                    "任务牌改为追击",
                    self.node.set_navigation_task,
                    NavigationTask.CHASE,
                ),
            ],
        )
    
    def _build_occupy_task_selection(self):
        # 没有更高优先级任务时，默认把任务牌改为中央占点
        return SetNavigationTask(
            "任务牌改为中央占点",
            self.node.set_navigation_task,
            NavigationTask.OCCUPY_CENTER,
        )

    def _build_resupply_task_execution(self):
        # 任务牌为“回补给”时，调用已有的回补给行为
        return Sequence(
            "执行回补给任务",
            [
                IsNavigationTask(
                    "当前任务是回补给",
                    self.node.get_navigation_task,
                    NavigationTask.RESUPPLY,
                ),
                GoResupply(
                    "执行前往补给区",
                    self.node.start_resupply,
                    self.node.stop_resupply,
                ),
            ],
        )

    def _build_occupy_task_execution(self):
        # 任务牌为“中央占点”时，调用已有的占点行为
        return Sequence(
            "执行中央占点任务",
            [
                IsNavigationTask(
                    "当前任务是中央占点",
                    self.node.get_navigation_task,
                    NavigationTask.OCCUPY_CENTER,
                ),
                OccupyZone(
                    "执行中央占点",
                    self.node.start_occupy,
                    self.node.stop_occupy,
                ),
            ],
        )

    def _build_chase_task_execution(self):
        # 任务牌为“追击”时，调用已有的追击行为
        return Sequence(
            "执行追击任务",
            [
                IsNavigationTask(
                    "当前任务是追击",
                    self.node.get_navigation_task,
                    NavigationTask.CHASE,
                ),
                ChaseEnemy(
                    "执行任务牌指定的追击",
                    self.node.start_chase,
                    self.node.get_chase_result,
                    self.node.stop_chase,
                ),
            ],
        )

    def _build_navigation_executor(self):
        # 根据唯一的导航任务牌，选择对应的平移行为执行
        return Selector(
            "公共导航执行树",
            [
                self._build_resupply_task_execution(),
                self._build_chase_task_execution(),
                self._build_occupy_task_execution(),
            ],
        )

    def _build_resupply_behavior(self):
        # 血量过低时前往补给区
        return Sequence(
            "回补给行为",
            [
                NeedResupply(
                    "是否需要回补给",
                    self.node.get_hp,
                    self.node.hp_limit,
                    self.node.hp_up,
                ),
                GoResupply(
                    "前往补给区",
                    self.node.start_resupply,
                    self.node.stop_resupply,
                ),
            ],
        )

    def _build_chase_behavior(self):
        # 先创建追击动作，供 CanChase 判断当前是否正在追击
        chase_action = ChaseEnemy(
            "执行追击",
            self.node.start_chase,
            self.node.get_chase_result,
            self.node.stop_chase,
        )

        # 判断是否可以开始或继续追击
        # 将通信节点的位置和范围读取方法传给追击条件
        can_chase = CanChase(
            "是否需要追击",
            self.node.get_enemy_position,
            self.node.get_robot_position,
            self.node.get_control_zone,
            self.node.get_chase_boundary,
            self.node.is_enemy_within_lost_grace,
            lambda: chase_action.running,
        )
        return Sequence(
            "追击行为",
            [
                can_chase,
                chase_action,
            ],
        )

    def _build_occupy_behavior(self):
        # 回补给和追击都不执行时，默认占点
        return OccupyZone(
            "占领控制区",
            self.node.start_occupy,
            self.node.stop_occupy,
        )

    def _build_spin_behavior(self):
        # 比赛进行期间持续开启小陀螺
        return Sequence(
            "小陀螺行为",
            [
                IsMatchRunning(
                    "比赛进行中（旋转）",
                    self.node.get_match_progress,
                ),
                SpinChassis(
                    "开启小陀螺",
                    self.node.start_spin,
                    self.node.stop_spin,
                ),
            ],
        )