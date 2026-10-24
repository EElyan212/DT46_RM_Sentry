from rm_application.behavior_tree.core import (
    Node,
    Status,
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
    PatrolZone,
    SpinChassis,
)
from rm_application.behavior_tree.builder import BehaviorTreeBuilder
from rm_application.behavior_tree.nav_zone import (
    NavigationTask,
    PatrolRegion,
    TacticalMode,
)
from rm_application.behavior_tree.ros_interface import RosInterface
from rm_interfaces.msg import TacticalModeCommand


# 可指定返回状态的假节点
class FakeNode(Node):
    def __init__(self, name, result):
        super().__init__(name)
        self.result = result
        self.tick_count = 0
        self.halt_count = 0

    def tick(self):
        self.tick_count += 1
        print(f"{self.name}: {self.result.name}")
        return self.result

    def halt(self):
        self.halt_count += 1
        print(f"停止: {self.name}")


def test_sequence():
    first = FakeNode("第一个", Status.SUCCESS)
    second = FakeNode("第二个", Status.FAILURE)
    third = FakeNode("第三个", Status.SUCCESS)

    sequence = Sequence("测试Sequence", [first, second, third])
    result = sequence.tick()

    assert result == Status.FAILURE
    assert third.tick_count == 0

    print("Sequence 测试通过\n")


def test_selector():
    first = FakeNode("第一个", Status.FAILURE)
    second = FakeNode("第二个", Status.SUCCESS)
    third = FakeNode("第三个", Status.SUCCESS)

    selector = Selector("测试Selector", [first, second, third])
    result = selector.tick()

    assert result == Status.SUCCESS
    assert third.tick_count == 0

    print("Selector 测试通过\n")


def test_parallel():
    first = FakeNode("第一个", Status.FAILURE)
    second = FakeNode("第二个", Status.RUNNING)
    third = FakeNode("第三个", Status.SUCCESS)

    parallel = Parallel("测试Parallel", [first, second, third])
    result = parallel.tick()

    assert result == Status.RUNNING
    assert first.tick_count == 1
    assert second.tick_count == 1
    assert third.tick_count == 1

    print("Parallel 测试通过\n")


def test_selector_interrupt():
    resupply = FakeNode("回补给", Status.FAILURE)
    chase = FakeNode("追击", Status.RUNNING)

    selector = Selector("底盘选择", [resupply, chase])

    # 第一次：回补给不满足，开始追击
    first_result = selector.tick()

    assert first_result == Status.RUNNING
    assert selector.running_child is chase
    assert chase.halt_count == 0

    # 第二次：血量变低，回补给开始运行
    resupply.result = Status.RUNNING
    second_result = selector.tick()

    assert second_result == Status.RUNNING
    assert selector.running_child is resupply
    assert chase.halt_count == 1

    print("Selector 打断测试通过\n")


def test_sequence_interrupt():
    can_chase = FakeNode("是否值得追", Status.SUCCESS)
    chase = FakeNode("执行追击", Status.RUNNING)

    sequence = Sequence("追击行为", [can_chase, chase])

    # 第一次：条件成立，开始追击
    first_result = sequence.tick()

    assert first_result == Status.RUNNING
    assert sequence.running_child is chase
    assert chase.halt_count == 0

    # 第二次：条件失效，停止追击
    can_chase.result = Status.FAILURE
    second_result = sequence.tick()

    assert second_result == Status.FAILURE
    assert sequence.running_child is None
    assert chase.halt_count == 1

    print("Sequence 打断测试通过\n")


def test_need_resupply():
    hp = [400]
    condition = NeedResupply("需要回补给", lambda: hp[0], 150, 380)

    assert condition.tick() == Status.FAILURE

    # 低血开始回补给
    hp[0] = 150
    assert condition.tick() == Status.SUCCESS

    # 回血途中不提前结束
    hp[0] = 200
    assert condition.tick() == Status.SUCCESS

    hp[0] = 379
    assert condition.tick() == Status.SUCCESS

    # 恢复到目标血量才结束
    hp[0] = 380
    assert condition.tick() == Status.FAILURE

    # 结束回补给后，血量仍高于启动阈值时不重新进入。
    hp[0] = 200
    assert condition.tick() == Status.FAILURE

    print("回补给条件测试通过\n")


def test_can_chase():
    enemy_position = [None]
    robot_position = [(2.0, 0.0)]
    control_zone = [{
        "enabled": True,
        "x_min": -1.0,
        "x_max": 1.0,
        "y_min": -1.0,
        "y_max": 1.0,
    }]
    chase_boundary = {
        "enabled": True,
        "x_min": -3.0,
        "x_max": 3.0,
        "y_min": -3.0,
        "y_max": 3.0,
    }
    chasing = [False]
    enemy_within_lost_grace = [False]
    condition = CanChase(
        "是否需要追击",
        lambda: enemy_position[0],
        lambda: robot_position[0],
        lambda: control_zone[0],
        lambda: chase_boundary,
        lambda: enemy_within_lost_grace[0],
        lambda: chasing[0],
    )

    # 没有有效目标时不追击。
    assert condition.tick() == Status.FAILURE

    # 尚未开始追击且不在控制区内时，不能发起追击。
    robot_position[0] = (2.0, 0.0)
    enemy_position[0] = (5.0, 0.0)
    assert condition.tick() == Status.FAILURE

    # 进入控制区后，可以发起追击。
    robot_position[0] = (0.0, 0.0)
    enemy_position[0] = (3.0, 0.0)
    assert condition.tick() == Status.SUCCESS

    # 追击开始后，允许离开控制区继续追。
    robot_position[0] = (2.0, 0.0)
    enemy_position[0] = (5.0, 0.0)
    chasing[0] = True
    assert condition.tick() == Status.SUCCESS

    # 追击过程中短暂丢失目标时，保持追击。
    enemy_position[0] = None
    enemy_within_lost_grace[0] = True
    assert condition.tick() == Status.SUCCESS

    # 超过等待时间后停止追击。
    enemy_within_lost_grace[0] = False
    assert condition.tick() == Status.FAILURE

    # 哨兵自身到达最大追击边界时停止。
    enemy_position[0] = (5.0, 0.0)
    robot_position[0] = (3.0, 0.0)
    assert condition.tick() == Status.FAILURE

    print("追击条件测试通过\n")


def test_can_chase_interrupt():
    enemy_position = [(3.0, 0.0)]
    robot_position = [(0.0, 0.0)]
    control_zone = {
        "enabled": True,
        "x_min": -1.0,
        "x_max": 1.0,
        "y_min": -1.0,
        "y_max": 1.0,
    }
    chase_boundary = {
        "enabled": True,
        "x_min": -2.0,
        "x_max": 2.0,
        "y_min": -2.0,
        "y_max": 2.0,
    }
    condition = CanChase(
        "是否需要追击",
        lambda: enemy_position[0],
        lambda: robot_position[0],
        lambda: control_zone,
        lambda: chase_boundary,
        lambda: False,
        lambda: False,
    )
    chase = FakeNode("执行追击", Status.RUNNING)
    sequence = Sequence("追击行为", [condition, chase])

    assert sequence.tick() == Status.RUNNING
    assert chase.halt_count == 0

    # 追击中哨兵到达最大边界，Sequence 应停止追击动作。
    robot_position[0] = (2.0, 0.0)
    assert sequence.tick() == Status.FAILURE
    assert chase.halt_count == 1

    print("追击条件打断测试通过\n")


def test_go_resupply():
    hp = [100]
    calls = {"start": 0, "stop": 0}

    def start_resupply():
        calls["start"] += 1
        return True

    def stop_resupply():
        calls["stop"] += 1

    condition = NeedResupply("需要回补给", lambda: hp[0], 150, 380)
    action = GoResupply("前往补给区", start_resupply, stop_resupply)
    sequence = Sequence("回补给行为", [condition, action])

    # 低血时启动一次，连续 tick 不重复发送导航目标。
    assert sequence.tick() == Status.RUNNING
    assert sequence.tick() == Status.RUNNING
    assert calls["start"] == 1
    assert calls["stop"] == 0

    # 血量恢复后，条件失效并停止补给动作。
    hp[0] = 380
    assert sequence.tick() == Status.FAILURE
    assert calls["stop"] == 1
    assert action.running is False

    print("回补给动作测试通过\n")


def test_chase_enemy():
    enemy_position = [(3.0, 0.0)]
    robot_position = [(0.0, 0.0)]
    control_zone = {
        "enabled": True,
        "x_min": -1.0,
        "x_max": 1.0,
        "y_min": -1.0,
        "y_max": 1.0,
    }
    chase_boundary = {
        "enabled": True,
        "x_min": -3.0,
        "x_max": 3.0,
        "y_min": -3.0,
        "y_max": 3.0,
    }
    chase_result = [None]
    calls = {"start": 0, "stop": 0}

    def start_chase():
        calls["start"] += 1
        return True

    def stop_chase():
        calls["stop"] += 1

    action = ChaseEnemy(
        "执行追击",
        start_chase,
        lambda: chase_result[0],
        stop_chase,
    )
    condition = CanChase(
        "是否需要追击",
        lambda: enemy_position[0],
        lambda: robot_position[0],
        lambda: control_zone,
        lambda: chase_boundary,
        lambda: False,
        lambda: action.running,
    )
    sequence = Sequence("追击行为", [condition, action])

    # 追击期间只启动一次，并持续返回 RUNNING。
    assert sequence.tick() == Status.RUNNING
    assert sequence.tick() == Status.RUNNING
    assert calls["start"] == 1

    # 已经开始追击后，离开控制区仍继续运行。
    robot_position[0] = (2.0, 0.0)
    enemy_position[0] = (5.0, 0.0)
    assert sequence.tick() == Status.RUNNING
    assert calls["start"] == 1

    # 到达最大追击边界后，条件失效并停止追击。
    robot_position[0] = (3.0, 0.0)
    assert sequence.tick() == Status.FAILURE
    assert calls["stop"] == 1
    assert action.running is False

    # 停止后仍在控制区外，不能立即重新启动。
    robot_position[0] = (2.0, 0.0)
    enemy_position[0] = (5.0, 0.0)
    assert sequence.tick() == Status.FAILURE
    assert calls["start"] == 1

    # 返回控制区后可以重新启动，并能接收完成结果。
    robot_position[0] = (0.0, 0.0)
    enemy_position[0] = (3.0, 0.0)
    chase_result[0] = True
    assert sequence.tick() == Status.SUCCESS
    assert calls["start"] == 2
    assert action.running is False

    print("追击动作测试通过\n")


def test_occupy_zone():
    calls = {"start": 0, "stop": 0}

    def start_occupy():
        calls["start"] += 1
        return True

    def stop_occupy():
        calls["stop"] += 1

    action = OccupyZone("占领控制区", start_occupy, stop_occupy)

    # 占点持续运行，但只发送一次导航目标。
    assert action.tick() == Status.RUNNING
    assert action.tick() == Status.RUNNING
    assert calls["start"] == 1

    # 被高优先级行为打断后停止，再次进入时可以重新启动。
    action.halt()
    assert calls["stop"] == 1
    assert action.running is False
    assert action.tick() == Status.RUNNING
    assert calls["start"] == 2

    print("占点动作测试通过\n")


def test_patrol_zone():
    patrol_region = [PatrolRegion.HOME]
    patrol_result = [None]
    started_regions = []
    calls = {"stop": 0}

    def start_patrol(region):
        started_regions.append(region)
        return True

    def stop_patrol():
        calls["stop"] += 1

    action = PatrolZone(
        "区域巡逻",
        lambda: patrol_region[0],
        start_patrol,
        lambda: patrol_result[0],
        stop_patrol,
    )

    # 第一次进入时发送一个目标，后续 tick 不重复发送。
    assert action.tick() == Status.RUNNING
    assert action.tick() == Status.RUNNING
    assert started_regions == [PatrolRegion.HOME]

    # 到达当前目标后，在同一区域继续发送下一个目标。
    patrol_result[0] = True
    assert action.tick() == Status.RUNNING
    assert started_regions == [
        PatrolRegion.HOME,
        PatrolRegion.HOME,
    ]

    # 巡逻区域改变时，取消旧目标并在新区域重新开始。
    patrol_result[0] = None
    patrol_region[0] = PatrolRegion.ENEMY
    assert action.tick() == Status.RUNNING
    assert calls["stop"] == 1
    assert started_regions[-1] == PatrolRegion.ENEMY

    # Nav2 报告失败时，向上层返回 FAILURE。
    patrol_result[0] = False
    assert action.tick() == Status.FAILURE
    assert action.running is False
    assert action.active_region is None

    # 被其他行为打断时取消正在运行的巡逻目标。
    patrol_result[0] = None
    assert action.tick() == Status.RUNNING
    action.halt()
    assert calls["stop"] == 2
    assert action.running is False

    print("连续巡逻动作测试通过\n")


def test_sample_patrol_target():
    # 跳过真实 ROS 初始化，只验证巡逻选点的纯计算逻辑。
    interface = RosInterface.__new__(RosInterface)
    interface.patrol_areas = {
        PatrolRegion.HOME: [1.0, 2.0, -1.0, 1.0],
    }
    interface.patrol_min_goal_distance = 0.5
    interface.get_robot_position = lambda: (0.0, 0.0)

    # 有效区域应生成区域内且距离机器人足够远的目标。
    target = interface.sample_patrol_target(PatrolRegion.HOME)
    assert target is not None
    target_x, target_y = target
    assert 1.0 <= target_x <= 2.0
    assert -1.0 <= target_y <= 1.0
    assert target_x ** 2 + target_y ** 2 >= 0.5 ** 2

    # 全零占位区域无效，不能生成导航目标。
    interface.patrol_areas[PatrolRegion.HOME] = [
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    assert interface.sample_patrol_target(PatrolRegion.HOME) is None

    # 没有定位时不能计算目标与机器人的距离。
    interface.patrol_areas[PatrolRegion.HOME] = [
        1.0,
        2.0,
        -1.0,
        1.0,
    ]
    interface.get_robot_position = lambda: None
    assert interface.sample_patrol_target(PatrolRegion.HOME) is None

    # 区域太小且无法满足最小距离时，有限尝试后返回失败。
    interface.patrol_areas[PatrolRegion.HOME] = [
        -0.1,
        0.1,
        -0.1,
        0.1,
    ]
    interface.patrol_min_goal_distance = 1.0
    interface.get_robot_position = lambda: (0.0, 0.0)
    assert interface.sample_patrol_target(PatrolRegion.HOME) is None

    print("巡逻随机选点测试通过\n")


def test_spin_chassis():
    match_progress = [4]
    calls = {"start": 0, "stop": 0}

    def start_spin():
        calls["start"] += 1
        return True

    def stop_spin():
        calls["stop"] += 1

    condition = IsMatchRunning("比赛正在进行", lambda: match_progress[0])
    action = SpinChassis("开启小陀螺", start_spin, stop_spin)
    sequence = Sequence("小陀螺行为", [condition, action])

    # 比赛期间只开启一次并持续运行。
    assert sequence.tick() == Status.RUNNING
    assert sequence.tick() == Status.RUNNING
    assert calls["start"] == 1

    # 比赛结束后关闭小陀螺。
    match_progress[0] = 3
    assert sequence.tick() == Status.FAILURE
    assert calls["stop"] == 1
    assert action.running is False

    print("小陀螺动作测试通过\n")


class FakeBehaviorNode:
    """为 builder 提供参数、状态和动作方法，不包含真实 ROS 通信。"""

    def __init__(self):
        self.hp = 400
        self.enemy_position = None
        self.robot_position = (0.0, 0.0)
        self.control_zone = {
            "enabled": True,
            "x_min": -1.0,
            "x_max": 1.0,
            "y_min": -1.0,
            "y_max": 1.0,
        }
        self.chase_boundary = {
            "enabled": True,
            "x_min": -3.0,
            "x_max": 3.0,
            "y_min": -3.0,
            "y_max": 3.0,
        }
        self.match_progress = 3
        self.chase_result = None
        self.enemy_within_lost_grace = False
        self.navigation_task = NavigationTask.STOP
        self.tactical_mode = TacticalMode.NEUTRAL
        self.patrol_region = PatrolRegion.CENTER
        self.patrol_result = None

        self.hp_limit = 150
        self.hp_up = 380

        self.calls = {
            "start_resupply": 0,
            "stop_resupply": 0,
            "start_chase": 0,
            "stop_chase": 0,
            "start_occupy": 0,
            "stop_occupy": 0,
            "start_patrol": 0,
            "stop_patrol": 0,
            "start_spin": 0,
            "stop_spin": 0,
        }

    def get_hp(self):
        return self.hp

    def get_enemy_position(self):
        return self.enemy_position

    def get_robot_position(self):
        return self.robot_position

    def get_control_zone(self):
        return self.control_zone

    def get_chase_boundary(self):
        return self.chase_boundary

    def is_enemy_within_lost_grace(self):
        return self.enemy_within_lost_grace

    def get_match_progress(self):
        return self.match_progress

    def get_navigation_task(self):
        return self.navigation_task

    def set_navigation_task(self, navigation_task):
        self.navigation_task = navigation_task

    def get_tactical_mode(self):
        return self.tactical_mode

    def set_tactical_mode(self, tactical_mode):
        self.tactical_mode = tactical_mode

    def get_patrol_region(self):
        return self.patrol_region

    def set_patrol_region(self, patrol_region):
        self.patrol_region = patrol_region

    def start_resupply(self):
        self.calls["start_resupply"] += 1
        return True

    def stop_resupply(self):
        self.calls["stop_resupply"] += 1

    def start_chase(self):
        self.calls["start_chase"] += 1
        return True

    def get_chase_result(self):
        return self.chase_result

    def stop_chase(self):
        self.calls["stop_chase"] += 1

    def start_occupy(self):
        self.calls["start_occupy"] += 1
        return True

    def stop_occupy(self):
        self.calls["stop_occupy"] += 1

    def start_patrol(self, patrol_region):
        self.calls["start_patrol"] += 1
        self.patrol_region = patrol_region
        return True

    def get_patrol_result(self):
        return self.patrol_result

    def stop_patrol(self):
        self.calls["stop_patrol"] += 1

    def start_spin(self):
        self.calls["start_spin"] += 1
        return True

    def stop_spin(self):
        self.calls["stop_spin"] += 1


def test_navigation_executor_patrol_switch():
    node = FakeBehaviorNode()
    executor = BehaviorTreeBuilder(
        node
    )._build_navigation_executor()

    # PATROL 任务牌启动巡逻，连续 tick 不重复启动。
    node.navigation_task = NavigationTask.PATROL
    assert executor.tick() == Status.RUNNING
    assert executor.tick() == Status.RUNNING
    assert node.calls["start_patrol"] == 1

    # 巡逻区域变化时取消旧目标，并在新区域重新开始。
    node.patrol_region = PatrolRegion.ENEMY
    assert executor.tick() == Status.RUNNING
    assert node.calls["stop_patrol"] == 1
    assert node.calls["start_patrol"] == 2

    # 任务牌切换为占点时，停止巡逻并启动占点导航。
    node.navigation_task = NavigationTask.OCCUPY_CENTER
    assert executor.tick() == Status.RUNNING
    assert node.calls["stop_patrol"] == 2
    assert node.calls["start_occupy"] == 1

    print("公共导航执行树巡逻切换测试通过\n")


def test_occupy_patrol_minimum_loop():
    node = FakeBehaviorNode()
    builder = BehaviorTreeBuilder(node)
    task_selection = (
        builder._build_occupy_or_patrol_task_selection(
            PatrolRegion.CENTER
        )
    )
    behavior = builder._build_navigation_task_behavior(
        task_selection
    )

    # 控制区外选择占点，并启动中央点导航。
    node.robot_position = (2.0, 0.0)
    assert behavior.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.OCCUPY_CENTER
    assert node.calls["start_occupy"] == 1

    # 进入控制区后改为巡逻，取消占点并启动巡逻目标。
    node.robot_position = (0.0, 0.0)
    assert behavior.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.PATROL
    assert node.patrol_region == PatrolRegion.CENTER
    assert node.calls["stop_occupy"] == 1
    assert node.calls["start_patrol"] == 1

    # 保持在区内时继续同一次巡逻，不重复发送目标。
    assert behavior.tick() == Status.RUNNING
    assert node.calls["start_patrol"] == 1

    # 再次离开控制区后取消巡逻，并重新执行中央占点。
    node.robot_position = (2.0, 0.0)
    assert behavior.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.OCCUPY_CENTER
    assert node.calls["stop_patrol"] == 1
    assert node.calls["start_occupy"] == 2

    print("占点到区内巡逻最小闭环测试通过\n")


def test_tactical_navigation_task_selection():
    node = FakeBehaviorNode()
    builder = BehaviorTreeBuilder(node)
    behavior = builder._build_navigation_task_behavior(
        builder._build_navigation_task_selection()
    )

    # 比赛未进行时保持 STOP，所有平移行为都不启动。
    assert behavior.tick() == Status.FAILURE
    assert node.navigation_task == NavigationTask.STOP
    assert node.calls["start_resupply"] == 0
    assert node.calls["start_chase"] == 0
    assert node.calls["start_occupy"] == 0
    assert node.calls["start_patrol"] == 0

    # 中立模式、位于区内且没有目标时，在中央子区域巡逻。
    node.match_progress = 4
    assert behavior.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.PATROL
    assert node.patrol_region == PatrolRegion.CENTER
    assert node.calls["start_patrol"] == 1

    # 哨兵和敌人都在区内时，区内追击抢占巡逻。
    node.enemy_position = (0.5, 0.0)
    assert behavior.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.CHASE
    assert node.calls["stop_patrol"] == 1
    assert node.calls["start_chase"] == 1

    # 敌人离开控制区后立即停止追击，恢复中立巡逻。
    node.enemy_position = (2.0, 0.0)
    assert behavior.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.PATROL
    assert node.patrol_region == PatrolRegion.CENTER
    assert node.calls["stop_chase"] == 1
    assert node.calls["start_patrol"] == 2

    # 防守模式即使区内存在敌人，也不启动底盘追击。
    node.tactical_mode = TacticalMode.DEFENSE
    node.enemy_position = (0.5, 0.0)
    assert behavior.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.PATROL
    assert node.patrol_region == PatrolRegion.HOME
    assert node.calls["start_chase"] == 1

    # 前压模式没有可追目标时，在偏敌方子区域巡逻。
    node.tactical_mode = TacticalMode.PRESSURE
    node.enemy_position = None
    assert behavior.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.PATROL
    assert node.patrol_region == PatrolRegion.ENEMY

    # 离开中央控制区后，占点任务优先于三种战术内部行为。
    node.robot_position = (2.0, 0.0)
    assert behavior.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.OCCUPY_CENTER
    assert node.calls["start_occupy"] == 1

    # 低血时回补给抢占当前占点任务。
    node.hp = 100
    assert behavior.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.RESUPPLY
    assert node.calls["stop_occupy"] == 1
    assert node.calls["start_resupply"] == 1

    # 比赛结束后改为 STOP，并取消正在运行的补给导航。
    node.match_progress = 3
    assert behavior.tick() == Status.FAILURE
    assert node.navigation_task == NavigationTask.STOP
    assert node.calls["stop_resupply"] == 1

    print("三种战术导航任务选择测试通过\n")


def test_pressure_chase_leave_zone_and_return():
    node = FakeBehaviorNode()
    node.match_progress = 4
    node.tactical_mode = TacticalMode.PRESSURE
    node.enemy_position = (2.0, 0.0)

    builder = BehaviorTreeBuilder(node)
    behavior = builder._build_navigation_task_behavior(
        builder._build_navigation_task_selection()
    )

    # 最大追击边界未配置时，前压模式也只能在区内巡逻。
    node.chase_boundary["enabled"] = False
    assert behavior.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.PATROL
    assert node.patrol_region == PatrolRegion.ENEMY
    assert node.calls["start_chase"] == 0

    # 配置最大追击边界后，可以从中央控制区内开始追击。
    node.chase_boundary["enabled"] = True
    assert behavior.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.CHASE
    assert node.calls["stop_patrol"] == 1
    assert node.calls["start_chase"] == 1

    # 追击开始后，哨兵可以离开中央控制区继续追击。
    node.robot_position = (2.0, 0.0)
    node.enemy_position = (2.5, 0.0)
    assert behavior.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.CHASE
    assert node.calls["stop_chase"] == 0
    assert node.calls["start_occupy"] == 0

    # 哨兵到达最大追击边界后停止追击，并重新执行中央占点。
    node.robot_position = (3.0, 0.0)
    assert behavior.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.OCCUPY_CENTER
    assert node.calls["stop_chase"] == 1
    assert node.calls["start_occupy"] == 1

    print("前压离区追击与回点测试通过\n")


def test_tactical_mode_callback():
    class FakeLogger:
        def __init__(self):
            self.infos = []
            self.warnings = []

        def info(self, message):
            self.infos.append(message)

        def warning(self, message):
            self.warnings.append(message)

    class FakeBlackboard:
        tactical_mode = TacticalMode.NEUTRAL

    class FakeTacticalModeNode:
        def __init__(self, blackboard):
            self.blackboard = blackboard
            self.logger = FakeLogger()

        def set_tactical_mode(self, tactical_mode):
            self.blackboard.tactical_mode = tactical_mode

        def get_logger(self):
            return self.logger

    blackboard = FakeBlackboard()
    node = FakeTacticalModeNode(blackboard)
    interface = RosInterface.__new__(RosInterface)
    interface.node = node
    interface.blackboard = blackboard

    # 单独勾选一个模式时，切换到对应战术。
    interface.tactical_mode_callback(
        TacticalModeCommand(defense=True)
    )
    assert blackboard.tactical_mode == TacticalMode.DEFENSE

    interface.tactical_mode_callback(
        TacticalModeCommand(neutral=True)
    )
    assert blackboard.tactical_mode == TacticalMode.NEUTRAL

    interface.tactical_mode_callback(
        TacticalModeCommand(pressure=True)
    )
    assert blackboard.tactical_mode == TacticalMode.PRESSURE

    # 重复选择当前模式时不重复记录切换日志。
    info_count = len(node.logger.infos)
    interface.tactical_mode_callback(
        TacticalModeCommand(pressure=True)
    )
    assert len(node.logger.infos) == info_count

    # 全不勾或同时勾选多个时拒绝消息，并保持原模式。
    interface.tactical_mode_callback(TacticalModeCommand())
    interface.tactical_mode_callback(
        TacticalModeCommand(defense=True, neutral=True)
    )
    assert blackboard.tactical_mode == TacticalMode.PRESSURE
    assert len(node.logger.warnings) == 2

    print("RQT 战术模式回调测试通过\n")


def test_behavior_tree_builder():
    node = FakeBehaviorNode()
    tree = BehaviorTreeBuilder(node).build()

    # 比赛开始前，平移和小陀螺都不启动。
    assert tree.tick() == Status.FAILURE
    assert node.calls["start_occupy"] == 0
    assert node.calls["start_spin"] == 0

    # 比赛开始：默认中立模式，位于区内时开始中央巡逻并开启小陀螺。
    node.match_progress = 4
    assert tree.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.PATROL
    assert node.patrol_region == PatrolRegion.CENTER
    assert node.calls["start_patrol"] == 1
    assert node.calls["start_spin"] == 1

    # 区内出现敌人：受限追击抢占巡逻，小陀螺不重复开启。
    node.enemy_position = (0.5, 0.0)
    assert tree.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.CHASE
    assert node.calls["start_chase"] == 1
    assert node.calls["stop_patrol"] == 1
    assert node.calls["start_spin"] == 1

    # 哨兵离开控制区后停止追击，重新执行中央占点。
    node.robot_position = (2.0, 0.0)
    node.enemy_position = (5.0, 0.0)
    assert tree.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.OCCUPY_CENTER
    assert node.calls["stop_chase"] == 1
    assert node.calls["start_occupy"] == 1

    # 回到控制区且没有目标时，停止占点并恢复中立巡逻。
    node.robot_position = (0.0, 0.0)
    node.enemy_position = None
    assert tree.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.PATROL
    assert node.calls["stop_occupy"] == 1
    assert node.calls["start_patrol"] == 2

    # 区内再次发现敌人后可以重新开始追击。
    node.enemy_position = (0.5, 0.0)
    assert tree.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.CHASE
    assert node.calls["start_chase"] == 2

    # 短暂丢失目标时保持当前追击，不取消也不重复启动。
    node.enemy_position = None
    node.enemy_within_lost_grace = True
    assert tree.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.CHASE
    assert node.calls["start_chase"] == 2

    # 血量过低：回补给抢占正在运行的追击。
    node.hp = 100
    assert tree.tick() == Status.RUNNING
    assert node.navigation_task == NavigationTask.RESUPPLY
    assert node.calls["start_resupply"] == 1
    assert node.calls["stop_chase"] == 2

    # 比赛结束：任务牌变为 STOP，停止补给导航和小陀螺。
    node.match_progress = 3
    assert tree.tick() == Status.FAILURE
    assert node.navigation_task == NavigationTask.STOP
    assert node.calls["stop_resupply"] == 1
    assert node.calls["stop_spin"] == 1

    print("行为树组装与优先级测试通过\n")




if __name__ == "__main__":
    test_sequence()
    test_selector()
    test_parallel()
    test_selector_interrupt()
    test_sequence_interrupt()
    test_need_resupply()
    test_can_chase()
    test_can_chase_interrupt()
    test_go_resupply()
    test_chase_enemy()
    test_occupy_zone()
    test_patrol_zone()
    test_sample_patrol_target()
    test_spin_chassis()
    test_navigation_executor_patrol_switch()
    test_occupy_patrol_minimum_loop()
    test_tactical_navigation_task_selection()
    test_pressure_chase_leave_zone_and_return()
    test_tactical_mode_callback()
    test_behavior_tree_builder()

    print("所有测试通过")
