from rm_application.behavior_tree.core import Node, Status

# 把任务选择结果写入共享黑板
class SetNavigationTask(Node):
    def __init__(self, name, set_navigation_task, task):
        super().__init__(name)

        self.set_navigation_task = set_navigation_task
        self.task = task

    def tick(self):
        self.set_navigation_task(self.task)
        return Status.SUCCESS

# 把战术选择的巡逻区域写入共享黑板
class SetPatrolRegion(Node):
    def __init__(self, name, set_patrol_region, patrol_region):
        super().__init__(name)

        self.set_patrol_region = set_patrol_region
        self.patrol_region = patrol_region

    def tick(self):
        self.set_patrol_region(self.patrol_region)
        return Status.SUCCESS

# 前往并守住控制区
class OccupyZone(Node):
    def __init__(self, name, start_occupy, stop_occupy):
        super().__init__(name)

        # 开始前往控制区的方法
        self.start_occupy = start_occupy

        # 停止占点导航的方法
        self.stop_occupy = stop_occupy

        # 是否已经启动占点导航
        self.running = False

    def tick(self):
        # 第一次进入占点行为时，只发送一次导航目标
        if not self.running:
            if not self.start_occupy():
                return Status.FAILURE

            self.running = True

        # 到达控制区后继续保持占点状态
        return Status.RUNNING

    def halt(self):
        # 被回补给或追击打断时，停止当前占点导航
        if self.running:
            self.stop_occupy()
            self.running = False

# 在指定区域内持续巡逻
class PatrolZone(Node):
    def __init__(
        self,
        name,
        get_patrol_region,
        start_patrol,
        get_patrol_result,
        stop_patrol,
    ):
        super().__init__(name)

        self.get_patrol_region = get_patrol_region
        self.start_patrol = start_patrol
        self.get_patrol_result = get_patrol_result
        self.stop_patrol = stop_patrol

        # 是否正在前往一个巡逻目标
        self.running = False

        # 记录当前目标所属的巡逻区域
        self.active_region = None

    def tick(self):
        patrol_region = self.get_patrol_region()

        # 战术切换导致巡逻区域改变时，取消旧区域的目标
        if (
            self.running
            and patrol_region != self.active_region
        ):
            self.stop_patrol()
            self.running = False
            self.active_region = None

        # 尚未运行时，生成并发送一个巡逻目标
        if not self.running:
            if not self.start_patrol(patrol_region):
                return Status.FAILURE

            self.running = True
            self.active_region = patrol_region
            return Status.RUNNING

        # 查询当前随机目标的导航结果
        result = self.get_patrol_result()
        if result is None:
            return Status.RUNNING

        self.running = False

        # 当前目标不可达时交给上层恢复逻辑
        if not result:
            self.active_region = None
            return Status.FAILURE

        # 成功到达后，在同一区域生成下一个目标
        if not self.start_patrol(patrol_region):
            self.active_region = None
            return Status.FAILURE

        self.running = True
        self.active_region = patrol_region
        return Status.RUNNING

    def halt(self):
        # 被回血、占点或追击打断时取消巡逻
        if self.running:
            self.stop_patrol()

        self.running = False
        self.active_region = None

# 前往补给区
class GoResupply(Node):
    def __init__(self, name, start_resupply, stop_resupply):
        super().__init__(name)

        # 开始前往补给区的方法
        self.start_resupply = start_resupply

        # 停止前往补给区的方法
        self.stop_resupply = stop_resupply

        # 是否已经启动补给导航
        self.running = False

    def tick(self):
        # 第一次进入时，只发送一次补给区导航目标
        if not self.running:
            if not self.start_resupply():
                return Status.FAILURE

            self.running = True

        # 到达补给区后仍等待回血，由 NeedResupply 判断何时结束
        return Status.RUNNING

    def halt(self):
        # 血量恢复到目标值，或行为树停止时取消补给导航
        if self.running:
            self.stop_resupply()
            self.running = False

# 执行追击；具体通信方法以后接入
class ChaseEnemy(Node):
    def __init__(self, name, start_chase, get_chase_result, stop_chase):
        super().__init__(name)

        self.start_chase = start_chase
        self.get_chase_result = get_chase_result
        self.stop_chase = stop_chase
        self.running = False

    def tick(self):
        # 只在刚进入追击行为时启动一次
        if not self.running:
            if not self.start_chase():
                return Status.FAILURE
            self.running = True

        # None 表示仍在追；True/False 表示追击已经结束
        result = self.get_chase_result()
        if result is None:
            return Status.RUNNING

        self.running = False
        return Status.SUCCESS if result else Status.FAILURE

    def halt(self):
        # 被回补给抢占，或追击条件失效时停止
        if self.running:
            self.stop_chase()
            self.running = False

# 持续开启底盘小陀螺
class SpinChassis(Node):
    def __init__(self, name, start_spin, stop_spin):
        super().__init__(name)

        # 开启小陀螺的方法
        self.start_spin = start_spin

        # 关闭小陀螺的方法
        self.stop_spin = stop_spin

        # 是否已经开启小陀螺
        self.running = False

    def tick(self):
        # 第一次进入时开启小陀螺
        if not self.running:
            if not self.start_spin():
                return Status.FAILURE

            self.running = True

        # 比赛进行期间持续运行
        return Status.RUNNING

    def halt(self):
        # 比赛结束或整棵树停止时关闭小陀螺
        if self.running:
            self.stop_spin()
            self.running = False
