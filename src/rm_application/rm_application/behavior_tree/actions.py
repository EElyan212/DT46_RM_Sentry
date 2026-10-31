import random
import time
import py_trees
from py_trees.ports import BehaviourWithPorts
from rm_application.behavior_tree.nav_zone import NavigationResult


# 把任务选择结果写入共享黑板
class SetNavigationTask(BehaviourWithPorts):
    # 通过 Python 回调写入任务，不使用 XML 数据端口
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(self, name, set_navigation_task, task, **kwargs):
        super().__init__(name=name, **kwargs)

        # 写入导航任务的方法和当前节点要设置的任务
        self.set_navigation_task = set_navigation_task
        self.task = task

    def update(self):
        self.set_navigation_task(self.task)
        return py_trees.common.Status.SUCCESS


# 把战术选择的巡逻区域写入共享黑板
class SetPatrolRegion(BehaviourWithPorts):
    # 通过 Python 回调写入巡逻区域，不使用 XML 数据端口
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(
        self,
        name,
        set_patrol_region,
        patrol_region,
        **kwargs,
    ):
        super().__init__(name=name, **kwargs)

        # 写入巡逻区域的方法和当前节点要设置的区域
        self.set_patrol_region = set_patrol_region
        self.patrol_region = patrol_region

    def update(self):
        self.set_patrol_region(self.patrol_region)
        return py_trees.common.Status.SUCCESS

# 前往并守住控制区
class OccupyZone(BehaviourWithPorts):
    # 通过 Python 回调执行导航，不使用 XML 数据端口
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(
        self,
        name,
        start_occupy,
        get_occupy_result,
        stop_occupy,
        **kwargs,
    ):
        super().__init__(name=name, **kwargs)

        # 启动、查询结果和停止占点导航的方法
        self.start_occupy = start_occupy
        self.get_occupy_result = get_occupy_result
        self.stop_occupy = stop_occupy

        # 占点行为是否已经启动
        self.running = False

        # 是否已经成功到达控制区目标
        self.arrived = False

        # 到达计数供恢复控制判断是否取得进展
        self.goal_reached_count = 0

    def update(self):
        # 第一次进入时发送一次占点导航目标
        if not self.running:
            if not self.start_occupy():
                return py_trees.common.Status.FAILURE

            self.running = True
            self.arrived = False

        # 已经到达后继续保持占点，不重复发送目标
        if self.arrived:
            return py_trees.common.Status.RUNNING

        result = self.get_occupy_result()

        if result == NavigationResult.RUNNING:
            return py_trees.common.Status.RUNNING

        if result == NavigationResult.SUCCEEDED:
            self.arrived = True
            self.goal_reached_count += 1
            return py_trees.common.Status.RUNNING

        self.running = False
        self.arrived = False

        # 只有真正失败才交给上层恢复
        if result == NavigationResult.FAILED:
            return py_trees.common.Status.FAILURE

        # IDLE 或 CANCELED 表示正常退出
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        # 被回血、追击或停止任务打断时取消占点导航
        if self.running:
            self.stop_occupy()

        self.running = False
        self.arrived = False

# 在指定区域内持续巡逻
class PatrolZone(BehaviourWithPorts):
    # 通过 Python 回调执行巡逻，不使用 XML 数据端口
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(
        self,
        name,
        get_patrol_region,
        start_patrol,
        get_patrol_result,
        stop_patrol,
        get_patrol_area,
        get_robot_position,
        patrol_min_goal_distance=0.5,
        is_navigation_stopped=None,
        on_created=None,
        **kwargs,
    ):
        super().__init__(name=name, **kwargs)

        # 读取巡逻区域、启动导航、查询结果和取消导航的方法
        self.get_patrol_region = get_patrol_region
        self.start_patrol = start_patrol
        self.get_patrol_result = get_patrol_result
        self.stop_patrol = stop_patrol

        # 查询旧导航是否已经结束，换巡逻区域时使用
        self.is_navigation_stopped = is_navigation_stopped

        # 读取巡逻范围和哨兵当前位置的方法
        self.get_patrol_area = get_patrol_area
        self.get_robot_position = get_robot_position
        self.patrol_min_goal_distance = patrol_min_goal_distance

        # 是否正在前往一个巡逻目标
        self.running = False

        # 记录当前目标所属的巡逻区域
        self.active_region = None
        self.goal_reached_count = 0

        # 保存恢复节点选好的备用目标及其所属区域
        self.recovery_target = None
        self.recovery_region = None

        # 将实际创建的巡逻实例交给注册表，供备用点回调使用
        if on_created is not None:
            on_created(self)

    def sample_patrol_target(self, patrol_region):
        area = self.get_patrol_area(patrol_region)
        if area is None or len(area) != 4:
            return None

        x_min, x_max, y_min, y_max = map(float, area)

        # 未配置有效区域时，不生成目标
        if x_min >= x_max or y_min >= y_max:
            return None

        robot_position = self.get_robot_position()
        if robot_position is None:
            return None

        robot_x, robot_y = robot_position
        min_distance_squared = (
            float(self.patrol_min_goal_distance) ** 2
        )

        # 限制尝试次数，避免选不到点时一直循环
        for _ in range(20):
            target_x = random.uniform(x_min, x_max)
            target_y = random.uniform(y_min, y_max)

            distance_squared = (
                (target_x - robot_x) ** 2
                + (target_y - robot_y) ** 2
            )

            if distance_squared >= min_distance_squared:
                return target_x, target_y

        return None

    def save_recovery_target(self, target_position):
        # 保存备用坐标，不在这里发送导航
        self.recovery_target = tuple(target_position)

        # 备用点只允许用于准备它时对应的巡逻区域
        self.recovery_region = self.get_patrol_region()

    def _start_patrol_goal(self, patrol_region):
        # 备用点仍属于当前巡逻区域时，优先使用
        if self.recovery_region == patrol_region:
            target_position = self.recovery_target
        else:
            target_position = None

        # 备用点取出后就清空，避免下一次重复使用
        # 若本次发送失败，交给恢复流程重新准备目标
        self.recovery_target = None
        self.recovery_region = None

        # 没有可用备用点时，进行正常巡逻选点
        if target_position is None:
            target_position = self.sample_patrol_target(
                patrol_region
            )

        # 没有合法目标时，不发送导航
        if target_position is None:
            return False

        return self.start_patrol(target_position)

    def update(self):
        patrol_region = self.get_patrol_region()

        # 战术切换导致巡逻区域改变时，取消旧区域的目标
        if (
            self.running
            and patrol_region != self.active_region
        ):
            self.stop_patrol()
            self.running = False
            self.active_region = None

        # 尚未运行时，先确认旧导航已经结束
        if not self.running:
            if (
                self.is_navigation_stopped is not None
                and not self.is_navigation_stopped()
            ):
                # 等待取消完成，不把正常换区当成导航失败
                return py_trees.common.Status.RUNNING

            # 使用本次读取到的最新巡逻区域
            if not self._start_patrol_goal(patrol_region):
                return py_trees.common.Status.FAILURE

            self.running = True
            self.active_region = patrol_region
            return py_trees.common.Status.RUNNING

        result = self.get_patrol_result()

        if result == NavigationResult.RUNNING:
            return py_trees.common.Status.RUNNING

        # 主动取消不属于导航故障，不启动恢复
        if result in (
            NavigationResult.IDLE,
            NavigationResult.CANCELED,
        ):
            self.running = False
            self.active_region = None
            return py_trees.common.Status.SUCCESS

        self.running = False

        # 只有真正的导航失败才向上返回 FAILURE
        if result == NavigationResult.FAILED:
            self.active_region = None
            return py_trees.common.Status.FAILURE

        self.goal_reached_count += 1

        # 成功到达后，在同一区域生成下一个目标
        if not self._start_patrol_goal(patrol_region):
            self.active_region = None
            return py_trees.common.Status.FAILURE

        self.running = True
        self.active_region = patrol_region
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # 被回血、占点、追击或停止任务打断时取消巡逻
        if self.running:
            self.stop_patrol()

        self.running = False
        self.active_region = None

        # 当前巡逻任务退出后，不保留尚未使用的备用点
        self.recovery_target = None
        self.recovery_region = None

# 前往补给区
class GoResupply(BehaviourWithPorts):
    # 通过 Python 回调执行导航，不使用 XML 数据端口
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(
        self,
        name,
        start_resupply,
        get_resupply_result,
        stop_resupply,
        **kwargs,
    ):
        super().__init__(name=name, **kwargs)

        # 启动、查询结果和停止补给导航的方法
        self.start_resupply = start_resupply
        self.get_resupply_result = get_resupply_result
        self.stop_resupply = stop_resupply

        # 回血行为是否已经启动
        self.running = False

        # 是否已经成功到达补给区目标
        self.arrived = False

        # 到达计数供恢复控制判断是否取得进展
        self.goal_reached_count = 0

    def update(self):
        # 第一次进入时发送一次补给区导航目标
        if not self.running:
            if not self.start_resupply():
                return py_trees.common.Status.FAILURE

            self.running = True
            self.arrived = False

        # 已经到达后继续等待回血，不重复发送目标
        if self.arrived:
            return py_trees.common.Status.RUNNING

        result = self.get_resupply_result()

        if result == NavigationResult.RUNNING:
            return py_trees.common.Status.RUNNING

        if result == NavigationResult.SUCCEEDED:
            self.arrived = True
            self.goal_reached_count += 1
            return py_trees.common.Status.RUNNING

        self.running = False
        self.arrived = False

        # 只有真正失败才交给上层恢复
        if result == NavigationResult.FAILED:
            return py_trees.common.Status.FAILURE

        # IDLE 或 CANCELED 表示正常退出
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        # 血量恢复或任务被打断时取消补给导航
        if self.running:
            self.stop_resupply()

        self.running = False
        self.arrived = False

# 执行追击，调用已有追击接口
class ChaseEnemy(BehaviourWithPorts):
    # 通过 Python 回调执行追击，不使用 XML 数据端口
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(
        self,
        name,
        start_chase,
        get_chase_result,
        stop_chase,
        **kwargs,
    ):
        super().__init__(name=name, **kwargs)

        # 启动、查询结果和停止追击的方法
        self.start_chase = start_chase
        self.get_chase_result = get_chase_result
        self.stop_chase = stop_chase

        # 是否已经启动追击
        self.running = False

    def update(self):
        # 只在刚进入追击行为时启动一次
        if not self.running:
            if not self.start_chase():
                return py_trees.common.Status.FAILURE

            self.running = True

        # None 表示仍在追；True/False 表示追击已经结束
        result = self.get_chase_result()
        if result is None:
            return py_trees.common.Status.RUNNING

        self.running = False

        if result:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        # 被回血抢占、追击条件失效或整棵树停止时取消追击
        if self.running:
            self.stop_chase()
            self.running = False

# 持续开启底盘小陀螺
class SpinChassis(BehaviourWithPorts):
    # 通过 Python 回调控制小陀螺，不使用 XML 数据端口
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(self, name, start_spin, stop_spin, **kwargs):
        super().__init__(name=name, **kwargs)

        # 开启小陀螺的方法
        self.start_spin = start_spin

        # 关闭小陀螺的方法
        self.stop_spin = stop_spin

        # 是否已经开启小陀螺
        self.running = False

    def update(self):
        # 第一次进入时开启小陀螺
        if not self.running:
            if not self.start_spin():
                return py_trees.common.Status.FAILURE

            self.running = True

        # 比赛进行期间持续运行
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # 比赛结束、节点退出或整棵树被打断时关闭小陀螺
        if self.running:
            self.stop_spin()
            self.running = False


# 导航失败后等待，为后续重试做准备
class WaitBeforeRetry(BehaviourWithPorts):
    # 等待时长由 Python 构造参数提供，不使用 XML 数据端口
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(
        self,
        name,
        wait_seconds,
        now_fn=time.monotonic,
        **kwargs,
    ):
        super().__init__(name=name, **kwargs)

        # 每次恢复需要等待的时长
        self.wait_seconds = wait_seconds

        # 获取当前时间的方法，测试时可以提供模拟时间
        self.now_fn = now_fn

        # None 表示尚未开始等待
        self.start_time = None

    def update(self):
        now = self.now_fn()

        # 第一次执行时记录开始时间
        if self.start_time is None:
            self.start_time = now

        # 等待时返回 RUNNING，不阻塞整棵树
        if now - self.start_time < self.wait_seconds:
            return py_trees.common.Status.RUNNING

        # 等待结束，允许恢复流程继续执行
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        # 等待完成或被打断后清除计时，下次进入时重新开始
        self.start_time = None


# 为当前导航任务准备一个备用目标
class SelectAlternativeGoal(BehaviourWithPorts):
    # 通过 Python 回调读取区域和保存目标，不使用 XML 数据端口
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(
        self,
        name,
        get_area,
        get_failed_position,
        save_target,
        recovery_min_goal_distance=0.5,
        recovery_sample_attempts=20,
        **kwargs,
    ):
        super().__init__(name=name, **kwargs)

        # 获取当前任务允许选点的矩形区域
        # 区域格式：[x_min, x_max, y_min, y_max]
        self.get_area = get_area

        # 获取刚刚导航失败的目标坐标
        # 坐标格式：(x, y)
        self.get_failed_position = get_failed_position

        # 保存选好的备用目标，供后续导航使用
        # 这里只保存坐标，不发送导航请求
        self.save_target = save_target

        # 备用目标与失败目标之间的最小距离，单位：米
        self.recovery_min_goal_distance = recovery_min_goal_distance

        # 最多随机采样多少次，避免选不到点时一直循环
        self.recovery_sample_attempts = recovery_sample_attempts

    def sample_recovery_target(self, area, failed_position):
        # 必须提供完整的矩形边界
        if area is None or len(area) != 4:
            return None

        # 必须知道刚刚失败的目标位置
        if failed_position is None or len(failed_position) != 2:
            return None

        # 读取区域边界和失败目标坐标
        x_min, x_max, y_min, y_max = map(float, area)
        failed_x, failed_y = map(float, failed_position)

        # 全零占位区域或边界顺序错误时，不生成备用点
        if not (x_min < x_max and y_min < y_max):
            return None

        # 读取最小距离和采样次数
        min_distance = float(self.recovery_min_goal_distance)
        attempts = int(self.recovery_sample_attempts)

        # 参数无效时，不进行采样
        if min_distance <= 0.0 or attempts <= 0:
            return None

        # 在当前任务允许的区域内，有限次数随机选点
        for _ in range(attempts):
            target_x = random.uniform(x_min, x_max)
            target_y = random.uniform(y_min, y_max)

            # 计算候选点与失败目标之间的平方距离
            distance_squared = (
                (target_x - failed_x) ** 2
                + (target_y - failed_y) ** 2
            )

            # 与失败目标距离足够远时，返回这个备用点
            if distance_squared >= min_distance ** 2:
                return target_x, target_y

        # 尝试次数耗尽，仍未找到符合条件的备用点
        return None

    def update(self):
        # 首次选点就失败时，可能还没有发出过任何导航目标
        failed_position = self.get_failed_position()

        # 没有失败目标时，交回原动作重新尝试正常选点
        if failed_position is None:
            return py_trees.common.Status.SUCCESS

        # 在当前区域内，选择远离失败目标的备用点
        target = self.sample_recovery_target(
            self.get_area(),
            failed_position,
        )

        # 已有失败目标但选不到合法备用点时，报告恢复失败
        if target is None:
            return py_trees.common.Status.FAILURE

        # 只保存坐标，具体导航由原任务的执行动作发起
        self.save_target(target)
        return py_trees.common.Status.SUCCESS
