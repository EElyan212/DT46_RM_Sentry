from rm_application.behavior_tree.core import (
    Sequence,
    Selector,
    Parallel,
    Switch,
)
from rm_application.behavior_tree.conditions import (
    NeedResupply,
    CanChase,
    CanChaseInsideCenterZone,
    IsMatchRunning,
    IsMatchStopped,
    IsNavigationTask,
    IsTacticalMode,
    NeedEnterCenterZone,
    IsChaseReady,
)
from rm_application.behavior_tree.actions import (
    SelectAlternativeGoal,
    WaitBeforeRetry,
    GoResupply,
    ChaseEnemy,
    OccupyZone,
    PatrolZone,
    SpinChassis,
    SetNavigationTask,
    SetPatrolRegion,
)
from rm_application.behavior_tree.nav_zone import (
    NavigationTask,
    PatrolRegion,
    TacticalMode,
)
from rm_application.behavior_tree.recovery import RecoveryControl

# 负责创建和组装整棵哨兵行为树
class BehaviorTreeBuilder:
    def __init__(self, node):
        # ROS 主节点，提供参数、实时数据和动作方法
        self.node = node

    def build(self):
        # 创建底盘平移和底盘旋转两条并行分支
        translation_behavior = self._build_navigation_task_behavior(
            self._build_navigation_task_selection()
        )
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

    def _build_stop_task_selection(self):
        # 比赛未进行时，把导航任务牌改为停止
        return Sequence(
            "选择停止导航任务",
            [
                IsMatchStopped(
                    "判断比赛是否停止",
                    self.node.get_match_progress,
                ),
                SetNavigationTask(
                    "任务牌改为停止",
                    self.node.set_navigation_task,
                    NavigationTask.STOP,
                ),
            ],
        )

    def _build_inside_zone_chase_task_selection(self):
        # 第一版只允许哨兵和敌人都在中央控制区内时追击
        can_chase = CanChaseInsideCenterZone(
            "判断是否允许区内追击",
            self.node.get_enemy_position,
            self.node.get_robot_position,
            self.node.get_control_zone,
            self.node.is_enemy_within_lost_grace,
            lambda: (
                self.node.get_navigation_task()
                == NavigationTask.CHASE
            ),
        )

        return Sequence(
            "选择区内追击任务",
            [
                IsChaseReady(
                    "追击失败冷却已结束",
                    self.node.is_chase_ready,
                ),
                can_chase,
                SetNavigationTask(
                    "任务牌改为区内追击",
                    self.node.set_navigation_task,
                    NavigationTask.CHASE,
                ),
            ],
        )

    def _build_defense_task_selection(self):
        # 防守：优先回到控制区，区内在偏己方区域巡逻
        return Selector(
            "防守策略任务选择",
            [
                self._build_occupy_task_selection(),
                self._build_patrol_task_selection(
                    PatrolRegion.HOME
                ),
            ],
        )

    def _build_neutral_task_selection(self):
        # 中立：优先回到控制区，区内允许有限追击
        return Selector(
            "中立策略任务选择",
            [
                self._build_occupy_task_selection(),
                self._build_inside_zone_chase_task_selection(),
                self._build_patrol_task_selection(
                    PatrolRegion.CENTER
                ),
            ],
        )
    
    def _build_pressure_task_selection(self):
        # 前压：允许从控制区内开始追击，
        # 追击开始后可以离开控制区，直到到达最大追击边界
        return Selector(
            "前压策略任务选择",
            [
                self._build_chase_task_selection(),
                self._build_occupy_task_selection(),
                self._build_patrol_task_selection(
                    PatrolRegion.ENEMY
                ),
            ],
        )

    def _build_tactical_task_selection(self):
        # 根据黑板中的战术模式选择对应的任务选择子树
        return Selector(
            "根据战术模式选择任务",
            [
                Sequence(
                    "使用防守策略",
                    [
                        IsTacticalMode(
                            "当前模式是防守",
                            self.node.get_tactical_mode,
                            TacticalMode.DEFENSE,
                        ),
                        self._build_defense_task_selection(),
                    ],
                ),
                Sequence(
                    "使用中立策略",
                    [
                        IsTacticalMode(
                            "当前模式是中立",
                            self.node.get_tactical_mode,
                            TacticalMode.NEUTRAL,
                        ),
                        self._build_neutral_task_selection(),
                    ],
                ),
                Sequence(
                    "使用前压策略",
                    [
                        IsTacticalMode(
                            "当前模式是前压",
                            self.node.get_tactical_mode,
                            TacticalMode.PRESSURE,
                        ),
                        self._build_pressure_task_selection(),
                    ],
                ),
            ],
        )

    def _build_navigation_task_selection(self):
        # 按公共优先级选择本轮唯一的导航任务
        return Selector(
            "公共导航任务选择",
            [
                self._build_stop_task_selection(),
                self._build_resupply_task_selection(),
                self._build_tactical_task_selection(),
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
                IsChaseReady(
                    "追击失败冷却已结束",
                    self.node.is_chase_ready,
                ),
                can_chase,
                SetNavigationTask(
                    "任务牌改为追击",
                    self.node.set_navigation_task,
                    NavigationTask.CHASE,
                ),
            ],
        )
    
    def _build_occupy_task_selection(self):
        # 位于控制区外时，选择进入中央控制区任务
        return Sequence(
            "选择进入中央控制区任务",
            [
                NeedEnterCenterZone(
                    "是否需要进入中央控制区",
                    self.node.get_robot_position,
                    self.node.get_control_zone,
                ),
                SetNavigationTask(
                    "任务牌改为中央占点",
                    self.node.set_navigation_task,
                    NavigationTask.OCCUPY_CENTER,
                ),
            ],
        )
    
    def _build_patrol_task_selection(self, patrol_region):
        # 写入巡逻区域，并把唯一导航任务牌改为巡逻
        return Sequence(
            "选择区域巡逻任务",
            [
                SetPatrolRegion(
                    "设置巡逻区域",
                    self.node.set_patrol_region,
                    patrol_region,
                ),
                SetNavigationTask(
                    "任务牌改为区域巡逻",
                    self.node.set_navigation_task,
                    NavigationTask.PATROL,
                ),
            ],
        )


    def _build_occupy_or_patrol_task_selection(
        self,
        patrol_region,
    ):
        # 区外优先进入中央控制区，区内持续巡逻
        return Selector(
            "选择占点或区域巡逻",
            [
                self._build_occupy_task_selection(),
                self._build_patrol_task_selection(
                    patrol_region
                ),
            ],
        )

    def _build_resupply_task_execution(self):
        # 创建正常的回补给动作
        resupply_action = GoResupply(
            "执行前往补给区",
            self.node.start_resupply,
            self.node.get_resupply_result,
            self.node.stop_resupply,
        )

        # 回补给失败后，先等待，再重试原来的补给目标
        resupply_recovery = RecoveryControl(
            "回补给恢复控制",
            action=resupply_action,
            recovery_action=WaitBeforeRetry(
                "回补给失败后等待",
                self.node.recovery_wait_seconds,
            ),
            max_retries=self.node.recovery_max_retries,
            get_progress=lambda: resupply_action.goal_reached_count,
        )

        # 任务牌改变时，退出回补给动作或恢复等待
        return Sequence(
            "执行回补给任务",
            [
                IsNavigationTask(
                    "当前任务是回补给",
                    self.node.get_navigation_task,
                    NavigationTask.RESUPPLY,
                ),
                resupply_recovery,
            ],
        )

    def _build_occupy_task_execution(self):
        # 创建正常的占点动作
        occupy_action = OccupyZone(
            "执行中央占点",
            self.node.start_occupy,
            self.node.get_occupy_result,
            self.node.stop_occupy,
        )

        # 占点失败后，先等待，再重试原来的占点目标
        occupy_recovery = RecoveryControl(
            "中央占点恢复控制",
            action=occupy_action,
            recovery_action=WaitBeforeRetry(
                "占点失败后等待",
                self.node.recovery_wait_seconds,
            ),
            max_retries=self.node.recovery_max_retries,
            get_progress=lambda: occupy_action.goal_reached_count,
        )

        # 每次 tick 先检查任务牌
        # 其他任务可以抢占正常占点，也可以抢占恢复等待
        return Sequence(
            "执行中央占点任务",
            [
                IsNavigationTask(
                    "当前任务是中央占点",
                    self.node.get_navigation_task,
                    NavigationTask.OCCUPY_CENTER,
                ),
                occupy_recovery,
            ],
        )    

    def _build_chase_task_execution(self):
        # 创建追击动作，继续调用已有的追击服务
        chase_action = ChaseEnemy(
            "执行任务牌指定的追击",
            self.node.start_chase,
            self.node.get_chase_result,
            self.node.stop_chase,
        )

        # 追击失败后先等待，再重新启动一次追击
        # 额外重试次数由恢复控制管理，追击服务内部不重复计数
        chase_recovery = RecoveryControl(
            "追击恢复控制",
            action=chase_action,
            recovery_action=WaitBeforeRetry(
                "追击失败后等待",
                self.node.recovery_wait_seconds,
            ),
            max_retries=1,
            on_exhausted=self.node.report_chase_exhausted,
        )

        # 每次 tick 先检查任务牌
        # 回补给、停止或其他任务可以打断追击和恢复等待
        return Sequence(
            "执行追击任务",
            [
                IsNavigationTask(
                    "当前任务是追击",
                    self.node.get_navigation_task,
                    NavigationTask.CHASE,
                ),
                chase_recovery,
            ],
        )

    def _build_patrol_task_execution(self):
        # 创建巡逻动作，负责正常选点、发送目标和使用备用点
        patrol_action = PatrolZone(
            "执行区域巡逻",
            self.node.get_patrol_region,
            self.node.start_patrol,
            self.node.get_patrol_result,
            self.node.stop_patrol,
            get_patrol_area=self.node.get_patrol_area,
            get_robot_position=self.node.get_robot_position,
            patrol_min_goal_distance=self.node.patrol_min_goal_distance,
            is_navigation_stopped=self.node.is_navigation_stopped,
        )

        # 巡逻失败后先等待，再为当前巡逻区域准备备用点
        patrol_recovery_steps = Sequence(
            "巡逻恢复步骤",
            [
                WaitBeforeRetry(
                    "巡逻失败后等待",
                    self.node.recovery_wait_seconds,
                ),
                SelectAlternativeGoal(
                    "准备巡逻备用点",
                    get_area=lambda: self.node.get_patrol_area(
                        self.node.get_patrol_region()
                    ),
                    get_failed_position=lambda: (
                        self.node.get_last_navigation_target("patrol")
                    ),
                    save_target=patrol_action.save_recovery_target,
                    recovery_min_goal_distance=(
                        self.node.recovery_min_goal_distance
                    ),
                    recovery_sample_attempts=(
                        self.node.recovery_sample_attempts
                    ),
                ),
            ],
        )

        # 恢复步骤完成后，再由原巡逻动作发送备用目标
        patrol_recovery = RecoveryControl(
            "区域巡逻恢复控制",
            action=patrol_action,
            recovery_action=patrol_recovery_steps,
            max_retries=self.node.recovery_max_retries,
            get_progress=lambda: patrol_action.goal_reached_count,
        )

        # 每轮先检查任务牌，允许其他任务抢占巡逻和恢复
        return Sequence(
            "执行区域巡逻任务",
            [
                IsNavigationTask(
                    "当前任务是区域巡逻",
                    self.node.get_navigation_task,
                    NavigationTask.PATROL,
                ),
                patrol_recovery,
            ],
        )

    def _build_navigation_executor(self):
        # 任务选择树已经确定优先级，这里只执行任务牌指定的分支
        return Switch(
            "公共导航执行树",
            get_key=self.node.get_navigation_task,
            can_start=self.node.is_navigation_stopped,
            children={
                NavigationTask.RESUPPLY: (
                    self._build_resupply_task_execution()
                ),
                NavigationTask.CHASE: (
                    self._build_chase_task_execution()
                ),
                NavigationTask.OCCUPY_CENTER: (
                    self._build_occupy_task_execution()
                ),
                NavigationTask.PATROL: (
                    self._build_patrol_task_execution()
                ),
            },
        )

    def _build_navigation_task_behavior(
        self,
        task_selection,
    ):
        # 每轮先更新任务牌，再由公共执行树执行对应行为
        return Sequence(
            "导航任务选择与执行",
            [
                task_selection,
                self._build_navigation_executor(),
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
                    self.node.get_resupply_result,
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
            self.node.get_occupy_result,
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