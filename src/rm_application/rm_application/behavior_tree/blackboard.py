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

    # 我方重装血量；尚未收到数据时为 None
    heavy_hp: int | None = None

    # 我方步兵血量
    infantry_hp: int | None = None

    # 哨兵剩余允许发弹量
    remain_bullet: int | None = None

    # 当前采用的全局策略
    tactical_mode: TacticalMode = TacticalMode.NEUTRAL

    # 条件节点确认后，交给动作节点应用的战术候选
    pending_tactical_mode: TacticalMode | None = None

    # 当前正式战术模式的开始时间，使用单调时钟
    tactical_mode_since: float = 0.0

    # 当前策略允许巡逻的区域
    patrol_region: PatrolRegion = PatrolRegion.CENTER

    # 导航树当前应当执行的任务
    navigation_task: NavigationTask = NavigationTask.STOP

    # 追击失败冷却结束的时间；0 表示尚未进入冷却
    chase_retry_after: float = 0.0