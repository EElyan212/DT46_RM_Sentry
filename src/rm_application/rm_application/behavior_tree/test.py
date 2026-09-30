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
    SpinChassis,
)
from rm_application.behavior_tree.builder import BehaviorTreeBuilder


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
    chasing = [False]
    condition = CanChase(
        "是否需要追击",
        lambda: enemy_position[0],
        lambda: robot_position[0],
        lambda: control_zone[0],
        lambda: chasing[0],
        2.0,
    )

    # 没有有效目标时不追击。
    assert condition.tick() == Status.FAILURE

    # 在攻击范围内及边界上时不追击。
    robot_position[0] = (0.0, 0.0)
    enemy_position[0] = (1.9, 0.0)
    assert condition.tick() == Status.FAILURE
    enemy_position[0] = (2.0, 0.0)
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

    # 重新进入攻击范围或丢失目标后停止。
    enemy_position[0] = (3.9, 0.0)
    assert condition.tick() == Status.FAILURE
    enemy_position[0] = None
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
    condition = CanChase(
        "是否需要追击",
        lambda: enemy_position[0],
        lambda: robot_position[0],
        lambda: control_zone,
        lambda: False,
        2.0,
    )
    chase = FakeNode("执行追击", Status.RUNNING)
    sequence = Sequence("追击行为", [condition, chase])

    assert sequence.tick() == Status.RUNNING
    assert chase.halt_count == 0

    # 追击中进入攻击范围，Sequence 应停止追击动作。
    enemy_position[0] = (2.0, 0.0)
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
        lambda: action.running,
        2.0,
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

    # 进入攻击范围后，条件失效并停止追击。
    enemy_position[0] = (4.0, 0.0)
    assert sequence.tick() == Status.FAILURE
    assert calls["stop"] == 1
    assert action.running is False

    # 停止后仍在控制区外，不能立即重新启动。
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
        self.match_progress = 3
        self.chase_result = None

        self.hp_limit = 150
        self.hp_up = 380
        self.attack_range = 2.0

        self.calls = {
            "start_resupply": 0,
            "stop_navigation": 0,
            "start_chase": 0,
            "stop_chase": 0,
            "start_occupy": 0,
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

    def get_match_progress(self):
        return self.match_progress

    def start_resupply(self):
        self.calls["start_resupply"] += 1
        return True

    def stop_navigation(self):
        self.calls["stop_navigation"] += 1

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

    def start_spin(self):
        self.calls["start_spin"] += 1
        return True

    def stop_spin(self):
        self.calls["stop_spin"] += 1


def test_behavior_tree_builder():
    node = FakeBehaviorNode()
    tree = BehaviorTreeBuilder(node).build()

    # 比赛开始前，平移和小陀螺都不启动。
    assert tree.tick() == Status.FAILURE
    assert node.calls["start_occupy"] == 0
    assert node.calls["start_spin"] == 0

    # 比赛开始：没有补给和追击需求，默认占点，同时开启小陀螺。
    node.match_progress = 4
    assert tree.tick() == Status.RUNNING
    assert node.calls["start_occupy"] == 1
    assert node.calls["start_spin"] == 1

    # 到达控制区后出现远目标：追击抢占占点，小陀螺不重复开启。
    node.robot_position = (0.0, 0.0)
    node.enemy_position = (3.0, 0.0)
    assert tree.tick() == Status.RUNNING
    assert node.calls["start_chase"] == 1
    assert node.calls["stop_navigation"] == 1
    assert node.calls["start_spin"] == 1

    # 离开控制区后，已经开始的追击可以继续。
    node.robot_position = (2.0, 0.0)
    node.enemy_position = (5.0, 0.0)
    assert tree.tick() == Status.RUNNING
    assert node.calls["start_chase"] == 1

    # 外层限制触发后追击结束；仍在控制区外时不会立即重新启动。
    node.chase_result = False
    assert tree.tick() == Status.RUNNING
    assert node.calls["start_occupy"] == 2
    assert tree.tick() == Status.RUNNING
    assert node.calls["start_chase"] == 1

    # 返回控制区后可以再次开始追击。
    node.robot_position = (0.0, 0.0)
    node.enemy_position = (3.0, 0.0)
    node.chase_result = None
    assert tree.tick() == Status.RUNNING
    assert node.calls["start_chase"] == 2

    # 血量过低：回补给抢占正在运行的追击。
    node.hp = 100
    assert tree.tick() == Status.RUNNING
    assert node.calls["start_resupply"] == 1
    assert node.calls["stop_chase"] == 1

    # 血量恢复且目标丢失：停止补给导航并重新占点。
    node.hp = 380
    node.enemy_position = None
    assert tree.tick() == Status.RUNNING
    assert node.calls["stop_navigation"] == 3
    assert node.calls["start_occupy"] == 3

    # 比赛结束：停止占点导航和小陀螺。
    node.match_progress = 3
    assert tree.tick() == Status.FAILURE
    assert node.calls["stop_navigation"] == 4
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
    test_spin_chassis()
    test_behavior_tree_builder()

    print("所有测试通过")
