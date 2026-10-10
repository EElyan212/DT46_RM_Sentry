from rm_application.behavior_tree.core import (
    Sequence,
    Selector,
    Parallel,
)
from rm_application.behavior_tree.conditions import (
    NeedResupply,
    CanChase,
    IsMatchRunning,
)
from rm_application.behavior_tree.actions import (
    GoResupply,
    ChaseEnemy,
    OccupyZone,
    SpinChassis,
)


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