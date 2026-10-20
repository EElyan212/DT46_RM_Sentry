from dataclasses import dataclass
from rm_application.behavior_tree.nav_zone import (
    NavigationTask,
    PatrolRegion,
    TacticalMode,
)


@dataclass
class Blackboard:
    """行为树各部分共享的状态。"""

    # 当前比赛阶段；0 表示尚未确认比赛正在进行
    match_progress: int = 0

    # 当前哨兵血量；尚未收到裁判数据时为 0
    self_hp: int = 0

    # 当前采用的全局策略
    tactical_mode: TacticalMode = TacticalMode.NEUTRAL

    # 当前策略允许巡逻的区域
    patrol_region: PatrolRegion = PatrolRegion.CENTER

    # 导航树当前应当执行的任务
    navigation_task: NavigationTask = NavigationTask.STOP