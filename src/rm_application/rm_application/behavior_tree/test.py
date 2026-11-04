"""导航收尾、XML 导航流程和战术模式切换测试。

使用模拟对象调用实际回调，不连接 Nav2，不代替仿真或实车联调。
"""

from collections import deque
from concurrent.futures import Future
from contextlib import contextmanager
from pathlib import Path
from threading import Lock
from types import SimpleNamespace
from unittest.mock import Mock, patch

import py_trees
import yaml

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from rm_interfaces.msg import Decision, SerialDebug

from rm_application.behavior_tree.actions import (
    PatrolZone,
    ApplyTacticalMode,
    WaitBeforeRetry,
)
from rm_application.behavior_tree.blackboard import Blackboard
from rm_application.behavior_tree.conditions import ShouldSwitchTacticalMode
from rm_application.behavior_tree.nav_zone import (
    NavigationResult,
    NavigationTask,
    PatrolRegion,
    TacticalMode,
)
from rm_application.behavior_tree.ros_interface import RosInterface
from rm_application.behavior_tree.xml_loader import load_behavior_tree
from rm_application.behavior_tree_node import BehaviorTreeNode
from rm_application.chase_client import ChaseClient



def _tactical_score_config(**changes):
    # 测试配置不依赖未确认的队友血量上限，默认全部不参与评分。
    config = {
        "factors": ["self_hp", "heavy_hp", "infantry_hp", "remain_bullet"],
        "weights": [0.0, 0.0, 0.0, 0.0],
        "maxima": [400.0, 0.0, 0.0, 750.0],
        "defense_enter": 0.35, "defense_exit": 0.45,
        "pressure_exit": 0.65, "pressure_enter": 0.75,
    }
    config.update(changes)
    return config


def test_chase_cancel_confirmation():
    interface = RosInterface.__new__(RosInterface)
    interface.node = Mock()
    interface.chase_goal_pending = True
    interface.chase_goal_handle = None
    interface.chase_result = None
    interface.chase_cancel_pending = False

    # 请求尚未被接受时要求取消，不能提前判断追击已经结束。
    assert interface.is_chase_stopped() is False
    interface.stop_chase()
    assert interface.chase_cancel_pending is True
    assert interface.is_chase_stopped() is False

    result_future = Future()
    cancel_future = Future()
    handle = Mock()
    handle.accepted = True
    handle.get_result_async.return_value = result_future
    handle.cancel_goal_async.return_value = cancel_future
    goal_future = Future()
    goal_future.set_result(handle)
    interface.chase_goal_response_callback(goal_future)
    handle.cancel_goal_async.assert_called_once()
    assert interface.chase_goal_pending is False
    assert interface.is_chase_stopped() is False

    # 取消请求收到答复，也不代表追击已经返回最终结果。
    cancel_future.set_result(SimpleNamespace(goals_canceling=[1]))
    assert interface.is_chase_stopped() is False
    result_future.set_result(SimpleNamespace(result=SimpleNamespace(
        success=False, message="追击已取消"
    )))
    assert interface.is_chase_stopped() is True
    assert interface.chase_goal_handle is None
    assert interface.chase_cancel_pending is False

    # 读取最终结果异常时无法确认服务结束，不能释放句柄。
    interface.chase_goal_handle = handle
    interface.chase_cancel_pending = True
    failed_future = Future()
    failed_future.set_exception(RuntimeError("结果读取失败"))
    interface.chase_result_callback(failed_future)
    assert interface.is_chase_stopped() is False
    assert interface.chase_goal_handle is handle

    print("追击请求未接受、取消答复和最终结果的结束确认测试通过\n")



def test_chase_server_waits_for_navigation_cancel():
    events = []
    navigator = Mock(spec=["cancelTask", "isTaskComplete", "goToPose"])
    navigator.cancelTask.side_effect = lambda: events.append("cancel")
    completions = iter([False, False, True])

    def is_complete():
        complete = next(completions)
        events.append("done" if complete else "running")
        return complete

    navigator.isTaskComplete.side_effect = is_complete
    logger = Mock()
    service = SimpleNamespace(
        offset_distance=1.5,
        navigator=navigator,
        get_logger=lambda: logger,
        is_enemy_tracked=lambda: True,
        get_target_client=Mock(),
        _start_target_updater=Mock(),
        _stop_target_updater=Mock(
            side_effect=lambda: events.append("updater_stopped")
        ),
    )
    service.get_target_client.wait_for_server.return_value = True
    goal = SimpleNamespace(
        request=SimpleNamespace(offset_distance=0.0),
        is_cancel_requested=True,
        canceled=Mock(side_effect=lambda: events.append("chase_canceled")),
    )
    # 不创建线程或 ROS 节点，直接运行真实服务回调的取消分支。
    with patch("rm_application.chase_client.rclpy.ok", side_effect=[True] * 4 + [False]):
        with patch("rm_application.chase_client.time.sleep", side_effect=lambda _: events.append("wait")):
            result = ChaseClient.execute_callback(service, goal)

    assert result.success is False
    assert result.message == "追击已取消"
    navigator.cancelTask.assert_called_once()
    navigator.goToPose.assert_not_called()
    goal.canceled.assert_called_once()
    assert events == [
        "cancel", "running", "wait", "running", "wait",
        "done", "chase_canceled", "updater_stopped",
    ]
    print("追击服务等待 Nav2 最终结束才报告取消完成的测试通过\n")



def _make_chase_execution_test_case():
    navigator = Mock(spec=["cancelTask", "isTaskComplete", "getResult", "goToPose"])
    navigator.goToPose.return_value = True
    service = SimpleNamespace(
        offset_distance=1.5,
        navigator=navigator,
        get_logger=Mock(return_value=Mock()),
        is_enemy_tracked=Mock(return_value=True),
        get_target_client=Mock(),
        _start_target_updater=Mock(),
        _stop_target_updater=Mock(),
        target_pose_lock=Lock(),
        latest_target_pose=PoseStamped(),
    )
    service.get_target_client.wait_for_server.return_value = True
    service._has_pose_changed = lambda old, new, threshold: ChaseClient._has_pose_changed(
        service, old, new, threshold
    )
    goal = SimpleNamespace(
        request=SimpleNamespace(offset_distance=0.0),
        is_cancel_requested=False,
        publish_feedback=Mock(),
        abort=Mock(),
        succeed=Mock(),
        canceled=Mock(),
    )
    return service, goal



def _run_chase_execution_test(service, goal, cycles=8):
    # 限制循环次数，测试回归时也不会无限运行；没有真实等待或线程。
    with patch("rm_application.chase_client.rclpy.ok", side_effect=[True] * cycles + [False]):
        with patch("rm_application.chase_client.time.sleep"):
            return ChaseClient.execute_callback(service, goal)



def test_chase_reports_navigation_failure():
    for nav_result in (TaskResult.FAILED, TaskResult.UNKNOWN):
        service, goal = _make_chase_execution_test_case()
        service.navigator.isTaskComplete.return_value = True
        service.navigator.getResult.return_value = nav_result
        next_target = PoseStamped()
        next_target.pose.position.x = 1.0

        def send_target(_target):
            service.latest_target_pose = next_target
            return True

        service.navigator.goToPose.side_effect = send_target
        result = _run_chase_execution_test(service, goal)
        # 即使敌人目标已经更新，也必须先处理旧导航的失败。
        assert result is not None
        service.navigator.goToPose.assert_called_once()
        service._stop_target_updater.assert_called_once()
        assert result.success is False
        assert "追击导航未成功" in result.message
        goal.abort.assert_called_once_with()

    # 已发出的导航失败后，即使跟踪数据暂时丢失，也不能跳过结果检查。
    service, goal = _make_chase_execution_test_case()
    service.is_enemy_tracked.side_effect = [True, True] + [False] * 8
    service.navigator.isTaskComplete.return_value = True
    service.navigator.getResult.return_value = TaskResult.FAILED
    result = _run_chase_execution_test(service, goal)
    assert result is not None, "跟踪数据丢失时，导航失败被 continue 跳过"
    assert result.success is False
    goal.abort.assert_called_once_with()
    service.navigator.goToPose.assert_called_once()
    service._stop_target_updater.assert_called_once()

    # 新目标被拒绝时，不能沿用上一目标成功的结果并继续反复发点。
    service, goal = _make_chase_execution_test_case()
    service.navigator.goToPose.return_value = False
    service.navigator.isTaskComplete.return_value = True
    service.navigator.getResult.return_value = TaskResult.SUCCEEDED
    result = _run_chase_execution_test(service, goal)
    assert result is not None, "Nav2 拒绝追击目标后没有返回失败"
    assert result.success is False
    goal.abort.assert_called_once_with()
    service.navigator.goToPose.assert_called_once()
    service._stop_target_updater.assert_called_once()

    # 启动前失败同样使用 ROS 的无参数 abort()，结果通过 return 返回。
    for missing in ("enemy", "target_server"):
        service, goal = _make_chase_execution_test_case()
        if missing == "enemy":
            service.is_enemy_tracked.return_value = False
        else:
            service.get_target_client.wait_for_server.return_value = False
        result = _run_chase_execution_test(service, goal)
        assert result.success is False
        goal.abort.assert_called_once_with()
        service.navigator.goToPose.assert_not_called()

    print("追击导航失败与未知结果上报测试通过\n")



def test_chase_navigation_success_continues():
    service, goal = _make_chase_execution_test_case()
    service.navigator.isTaskComplete.side_effect = [True, True]
    service.navigator.getResult.return_value = TaskResult.SUCCEEDED
    next_target = PoseStamped()
    next_target.pose.position.x = 1.0

    def send_target(_target):
        if service.navigator.goToPose.call_count == 1:
            service.latest_target_pose = next_target
        else:
            goal.is_cancel_requested = True
        return True

    service.navigator.goToPose.side_effect = send_target
    result = _run_chase_execution_test(service, goal)
    # 到达一个追击点后继续追击，直到上层要求退出。
    assert service.navigator.goToPose.call_count == 2
    service.navigator.getResult.assert_called_once()
    goal.succeed.assert_not_called()
    goal.abort.assert_not_called()
    goal.canceled.assert_called_once_with()
    assert result.message == "追击已取消"
    print("追击到达导航点后继续追击测试通过\n")



def test_chase_cooldown_state():
    node = SimpleNamespace(
        blackboard=Blackboard(),
        get_parameter=Mock(return_value=SimpleNamespace(value=3.0)),
        get_logger=Mock(return_value=Mock()),
    )
    with patch("rm_application.behavior_tree_node.time.monotonic") as clock:
        clock.return_value = 100.0
        # 首次选择追击前，冷却状态必须已经初始化。
        assert BehaviorTreeNode.is_chase_ready(node) is True
        BehaviorTreeNode.report_chase_exhausted(node)
        assert node.blackboard.chase_retry_after == 103.0
        assert BehaviorTreeNode.is_chase_ready(node) is False
        clock.return_value = 102.99
        assert BehaviorTreeNode.is_chase_ready(node) is False
        clock.return_value = 103.0
        assert BehaviorTreeNode.is_chase_ready(node) is True
    print("追击冷却初始化、耗尽记录与到期恢复测试通过\n")


def test_chase_selection_respects_cooldown():
    # 加载实际战术 XML，检查中立和前压是否遵守追击冷却。
    blackboard = Blackboard()
    state = SimpleNamespace(ready=False)
    node = Mock(
        tactical_mode_source="manual",
        tactical_score_config=_tactical_score_config(),
        tactical_confirm_seconds=2.0,
        tactical_min_hold_seconds=0.0,
        patrol_min_goal_distance=0.5,
        recovery_wait_seconds=0.5,
        recovery_max_retries=2,
        recovery_min_goal_distance=0.5,
        recovery_sample_attempts=20,
    )
    node.get_tactical_mode.side_effect = lambda: blackboard.tactical_mode
    node.get_tactical_mode_since.side_effect = lambda: blackboard.tactical_mode_since
    node.get_tactical_candidate.side_effect = lambda: blackboard.pending_tactical_mode
    node.set_tactical_candidate.side_effect = lambda candidate: setattr(
        blackboard, "pending_tactical_mode", candidate
    )
    node.is_chase_ready.side_effect = lambda: state.ready
    node.get_navigation_task.side_effect = lambda: blackboard.navigation_task
    node.set_navigation_task.side_effect = lambda task: setattr(
        blackboard, "navigation_task", task
    )
    node.set_patrol_region.side_effect = lambda region: setattr(
        blackboard, "patrol_region", region
    )
    node.get_enemy_position.return_value = (1.0, 1.0)
    node.get_robot_position.return_value = (1.0, 1.0)
    node.get_control_zone.return_value = dict(
        enabled=True, x_min=0.0, x_max=2.0, y_min=0.0, y_max=2.0
    )
    node.get_chase_boundary.return_value = dict(
        enabled=True, x_min=-10.0, x_max=10.0, y_min=-10.0, y_max=10.0
    )
    node.is_enemy_within_lost_grace.return_value = False
    xml_path = Path(__file__).resolve().parent / "tree" / "tactical_tree.xml"

    for mode in (TacticalMode.NEUTRAL, TacticalMode.PRESSURE):
        blackboard.tactical_mode = mode
        blackboard.navigation_task = NavigationTask.CHASE
        selection = load_behavior_tree(node, xml_path)

        # 冷却中选巡逻，冷却结束选追击，再次冷却时退出追击。
        for ready, expected_task in (
            (False, NavigationTask.PATROL),
            (True, NavigationTask.CHASE),
            (False, NavigationTask.PATROL),
        ):
            state.ready = ready
            selection.tick_once()
            assert selection.status == py_trees.common.Status.SUCCESS
            assert blackboard.navigation_task == expected_task, mode.name

        selection.stop(py_trees.common.Status.INVALID)

    print("中立和前压策略遵守追击冷却的测试通过\n")


def test_patrol_region_switch_waits_for_cancel():
    state = SimpleNamespace(region=PatrolRegion.CENTER, stopped=True)
    start = Mock(return_value=True)
    stop = Mock(side_effect=lambda: setattr(state, "stopped", False))
    action = PatrolZone(
        "巡逻换区测试",
        get_patrol_region=lambda: state.region,
        start_patrol=start,
        get_patrol_result=Mock(),
        stop_patrol=stop,
        get_patrol_area=Mock(),
        get_robot_position=Mock(),
        is_navigation_stopped=lambda: state.stopped,
    )
    points = {
        PatrolRegion.CENTER: (1.0, 1.0),
        PatrolRegion.ENEMY: (2.0, 2.0),
        PatrolRegion.HOME: (-1.0, -1.0),
    }
    action.sample_patrol_target = lambda region: points[region]
    action.tick_once()
    assert action.status == py_trees.common.Status.RUNNING
    state.region = PatrolRegion.ENEMY
    action.tick_once()
    assert action.status == py_trees.common.Status.RUNNING
    stop.assert_called_once()
    assert start.call_count == 1
    # 等待期间再次换区，应等旧导航结束后直接使用最新区域。
    state.region = PatrolRegion.HOME
    action.tick_once()
    assert action.status == py_trees.common.Status.RUNNING
    assert start.call_count == 1
    state.stopped = True
    action.tick_once()
    assert action.status == py_trees.common.Status.RUNNING
    stop.assert_called_once()
    assert [call.args[0] for call in start.call_args_list] == [
        points[PatrolRegion.CENTER], points[PatrolRegion.HOME],
    ]
    assert action.active_region == PatrolRegion.HOME
    action.get_patrol_result.assert_not_called()

    # 整个任务退出后，尚未使用的备用点也应丢弃。
    action.save_recovery_target((99.0, 99.0))
    action.stop(py_trees.common.Status.INVALID)
    assert stop.call_count == 2
    state.stopped = True
    action.tick_once()
    assert action.status == py_trees.common.Status.RUNNING
    assert start.call_args.args[0] == points[PatrolRegion.HOME]
    action.stop(py_trees.common.Status.INVALID)
    print("巡逻换区等待取消完成、等待中使用最新区域的测试通过\n")


def test_xml_patrol_recovery_uses_alternative_goal():
    # 用实际导航 XML 验证失败后的备用点确实交给原巡逻动作执行。
    blackboard = Blackboard(
        match_progress=4,
        self_hp=300,
        tactical_mode=TacticalMode.DEFENSE,
    )
    navigation = SimpleNamespace(
        owner=None, target=None, result=NavigationResult.RUNNING,
    )
    node = Mock(
        hp_limit=100,
        hp_up=200,
        tactical_mode_source="manual",
        tactical_score_config=_tactical_score_config(),
        tactical_confirm_seconds=2.0,
        tactical_min_hold_seconds=0.0,
        patrol_min_goal_distance=0.5,
        recovery_wait_seconds=0.0,
        recovery_max_retries=2,
        recovery_min_goal_distance=2.0,
        recovery_sample_attempts=3,
    )
    node.get_match_progress.side_effect = lambda: blackboard.match_progress
    node.get_hp.side_effect = lambda: blackboard.self_hp
    node.get_tactical_mode.side_effect = lambda: blackboard.tactical_mode
    node.get_tactical_mode_since.side_effect = lambda: blackboard.tactical_mode_since
    node.get_tactical_candidate.side_effect = lambda: blackboard.pending_tactical_mode
    node.set_tactical_candidate.side_effect = lambda candidate: setattr(
        blackboard, "pending_tactical_mode", candidate
    )
    node.get_navigation_task.side_effect = lambda: blackboard.navigation_task
    node.set_navigation_task.side_effect = lambda task: setattr(
        blackboard, "navigation_task", task
    )
    node.get_patrol_region.side_effect = lambda: blackboard.patrol_region
    node.set_patrol_region.side_effect = lambda region: setattr(
        blackboard, "patrol_region", region
    )
    node.get_control_zone.return_value = dict(
        enabled=True, x_min=0.0, x_max=4.0, y_min=0.0, y_max=4.0
    )
    node.get_robot_position.return_value = (1.0, 1.0)
    node.get_patrol_area.return_value = (0.0, 4.0, 0.0, 4.0)
    node.is_navigation_stopped.side_effect = lambda: navigation.owner is None
    node.get_last_navigation_target.side_effect = lambda owner: navigation.target
    node.get_patrol_result.side_effect = lambda: navigation.result

    def start(target):
        assert navigation.owner is None
        navigation.owner = "patrol"
        navigation.target = target
        navigation.result = NavigationResult.RUNNING
        return True

    node.start_patrol.side_effect = start
    node.stop_patrol.side_effect = lambda: setattr(navigation, "owner", None)
    xml_path = Path(__file__).resolve().parent / "tree" / "navigation_tree.xml"
    root = load_behavior_tree(node, xml_path)

    # 恢复选点先拒绝不满足配置距离的点，再由原巡逻动作发送备用点。
    with patch(
        "rm_application.behavior_tree.actions.random.uniform",
        side_effect=[2.0, 2.0, 3.0, 3.0, 4.0, 4.0],
    ) as sample:
        root.tick_once()
        assert blackboard.navigation_task == NavigationTask.PATROL
        navigation.owner = None
        navigation.result = NavigationResult.FAILED
        root.tick_once()
        assert node.start_patrol.call_count == 1
        root.tick_once()
        assert root.status == py_trees.common.Status.RUNNING
        assert [call.args[0] for call in node.start_patrol.call_args_list] == [
            (2.0, 2.0), (4.0, 4.0),
        ]
        assert sample.call_count == 6

    # 所有候选点都等于失败点时，有限采样后报告失败，不发送坏目标。
    navigation.owner = None
    navigation.result = NavigationResult.FAILED
    root.tick_once()
    with patch(
        "rm_application.behavior_tree.actions.random.uniform", return_value=4.0
    ) as sample:
        root.tick_once()
        assert root.status == py_trees.common.Status.FAILURE
        assert sample.call_count == 2 * node.recovery_sample_attempts
        assert node.start_patrol.call_count == 2
    root.stop(py_trees.common.Status.INVALID)
    print("XML 巡逻失败后使用备用点、采样耗尽不发送目标的测试通过\n")


def test_retry_wait_restarts_after_interrupt():
    # 用模拟时间验证等待到期与抢占后的重新计时，不实际 sleep。
    clock = SimpleNamespace(now=0.0)
    wait = WaitBeforeRetry(
        "恢复等待测试",
        wait_seconds=0.5,
        now_fn=lambda: clock.now,
    )
    running = py_trees.common.Status.RUNNING
    success = py_trees.common.Status.SUCCESS
    wait.tick_once()
    assert wait.status == running
    clock.now = 0.4
    wait.tick_once()
    assert wait.status == running

    # 等待尚未到期就被高优先级任务抢占，下次进入应完整等待。
    wait.stop(py_trees.common.Status.INVALID)
    clock.now = 10.0
    wait.tick_once()
    assert wait.status == running
    clock.now = 10.4
    wait.tick_once()
    assert wait.status == running
    clock.now = 10.5
    wait.tick_once()
    assert wait.status == success

    # 成功结束后的下一轮恢复也应重新计时。
    clock.now = 20.0
    wait.tick_once()
    assert wait.status == running
    clock.now = 20.5
    wait.tick_once()
    assert wait.status == success
    print("恢复等待到期、抢占与新一轮重新计时测试通过\n")


def test_xml_navigation_arrival_refreshes_retry_budget():
    # 占点和回血都需要：到达后持续等待，并清除之前的连续失败预算。
    xml_path = Path(__file__).resolve().parent / "tree" / "navigation_tree.xml"
    running = py_trees.common.Status.RUNNING
    for owner, hp in (("occupy", 300), ("resupply", 100)):
        blackboard = Blackboard(match_progress=4, self_hp=hp)
        node = Mock(
            hp_limit=100,
            hp_up=200,
            tactical_mode_source="manual",
            tactical_score_config=_tactical_score_config(),
            tactical_confirm_seconds=2.0,
            tactical_min_hold_seconds=0.0,
            recovery_wait_seconds=0.0,
            recovery_max_retries=1,
            recovery_min_goal_distance=0.5,
            recovery_sample_attempts=20,
            patrol_min_goal_distance=0.5,
        )
        node.get_match_progress.side_effect = lambda: blackboard.match_progress
        node.get_hp.side_effect = lambda: blackboard.self_hp
        node.get_tactical_mode.side_effect = lambda: blackboard.tactical_mode
        node.get_tactical_mode_since.side_effect = lambda: blackboard.tactical_mode_since
        node.get_tactical_candidate.side_effect = lambda: blackboard.pending_tactical_mode
        node.set_tactical_candidate.side_effect = lambda candidate: setattr(
            blackboard, "pending_tactical_mode", candidate
        )
        node.get_control_zone.return_value = {"enabled": False}
        node.get_navigation_task.side_effect = lambda: blackboard.navigation_task
        node.set_navigation_task.side_effect = lambda task: setattr(
            blackboard, "navigation_task", task
        )
        node.is_navigation_stopped.return_value = True
        start = getattr(node, f"start_{owner}")
        stop = getattr(node, f"stop_{owner}")
        result = getattr(node, f"get_{owner}_result")
        start.return_value = True
        result.side_effect = [NavigationResult.FAILED, NavigationResult.SUCCEEDED]
        root = load_behavior_tree(node, xml_path)

        # 第一次失败后恢复重试，第二次成功到达。
        root.tick_once()
        assert root.status == running, owner
        root.tick_once()
        assert root.status == running, owner
        for _ in range(3):
            root.tick_once()
            assert root.status == running, owner
        assert start.call_count == 2, owner
        assert result.call_count == 2, owner

        # 到达后被抢占，再次执行时应拥有一轮完整的重试预算。
        root.stop(py_trees.common.Status.INVALID)
        stop.assert_called_once()
        result.side_effect = None
        result.return_value = NavigationResult.FAILED
        root.tick_once()
        assert root.status == running, owner
        assert start.call_count == 3, owner
        root.tick_once()
        assert root.status == py_trees.common.Status.FAILURE, owner
        assert start.call_count == 4, owner

        # 本轮耗尽后仍可再次恢复重试，不永久停在 FAILURE。
        root.tick_once()
        assert root.status == running, owner
        assert start.call_count == 5, owner
        root.stop(py_trees.common.Status.INVALID)
    print("XML 占点/回血到达保持、重试预算恢复和耗尽后下一轮测试通过\n")


def test_xml_navigation_task_selection():
    # 加载实际主 XML，检查导航交接与小陀螺并行，仅模拟外部接口。
    blackboard = Blackboard(self_hp=300)
    navigation = SimpleNamespace(owner=None)
    events = []

    def start(owner):
        assert navigation.owner is None, "旧导航尚未结束就启动了新任务"
        navigation.owner = owner
        events.append(f"启动{owner}")
        return True

    def stop(owner):
        assert navigation.owner == owner
        events.append(f"取消{owner}")
        # 请求取消不等于确认结束，owner 由测试在确认时清除。

    node = Mock(
        hp_limit=100,
        hp_up=200,
        tactical_mode_source="manual",
        tactical_score_config=_tactical_score_config(),
        tactical_confirm_seconds=2.0,
        tactical_min_hold_seconds=0.0,
        recovery_wait_seconds=0.5,
        recovery_max_retries=2,
        recovery_min_goal_distance=0.5,
        recovery_sample_attempts=20,
        patrol_min_goal_distance=0.5,
    )
    node.get_match_progress.side_effect = lambda: blackboard.match_progress
    node.get_hp.side_effect = lambda: blackboard.self_hp
    node.get_tactical_mode.side_effect = lambda: blackboard.tactical_mode
    node.get_tactical_mode_since.side_effect = lambda: blackboard.tactical_mode_since
    node.get_tactical_candidate.side_effect = lambda: blackboard.pending_tactical_mode
    node.set_tactical_candidate.side_effect = lambda candidate: setattr(
        blackboard, "pending_tactical_mode", candidate
    )
    # 本用例专门验证占点与回血交接，未配置控制区时继续选择占点。
    node.get_control_zone.return_value = {"enabled": False}
    node.get_navigation_task.side_effect = lambda: blackboard.navigation_task
    node.set_navigation_task.side_effect = lambda task: setattr(
        blackboard, "navigation_task", task
    )
    node.is_navigation_stopped.side_effect = lambda: navigation.owner is None
    node.start_occupy.side_effect = lambda: start("占点")
    node.stop_occupy.side_effect = lambda: stop("占点")
    node.get_occupy_result.return_value = NavigationResult.RUNNING
    node.start_resupply.side_effect = lambda: start("回血")
    node.stop_resupply.side_effect = lambda: stop("回血")
    node.get_resupply_result.return_value = NavigationResult.RUNNING
    node.start_spin.return_value = True
    xml_path = Path(__file__).resolve().parent / "tree" / "main_tree.xml"
    root = load_behavior_tree(node, xml_path)

    def tick(expected_task, expected_status):
        root.tick_once()
        assert blackboard.navigation_task == expected_task
        assert root.status == expected_status

    running = py_trees.common.Status.RUNNING
    # 已确认停止时，沿用原执行器无活动任务返回 FAILURE 的约定。
    tick(NavigationTask.STOP, py_trees.common.Status.FAILURE)
    assert events == []
    node.start_spin.assert_not_called()

    blackboard.match_progress = 4
    tick(NavigationTask.OCCUPY_CENTER, running)
    tick(NavigationTask.OCCUPY_CENTER, running)
    node.start_occupy.assert_called_once()
    node.start_spin.assert_called_once()

    # 达到回血阈值后取消占点，等待期间不重复取消或启动回血。
    blackboard.self_hp = 100
    tick(NavigationTask.RESUPPLY, running)
    tick(NavigationTask.RESUPPLY, running)
    blackboard.self_hp = 150
    tick(NavigationTask.RESUPPLY, running)
    node.stop_occupy.assert_called_once()
    node.start_resupply.assert_not_called()

    navigation.owner = None  # 模拟旧导航最终结束。
    tick(NavigationTask.RESUPPLY, running)
    node.start_resupply.assert_called_once()

    # 达到回血结束阈值后，同样等回血导航释放再重新占点。
    blackboard.self_hp = 200
    tick(NavigationTask.OCCUPY_CENTER, running)
    node.stop_resupply.assert_called_once()
    assert node.start_occupy.call_count == 1
    navigation.owner = None
    tick(NavigationTask.OCCUPY_CENTER, running)
    assert node.start_occupy.call_count == 2
    # 导航任务切换期间，小陀螺持续运行且不重复开启。
    node.start_spin.assert_called_once()
    node.stop_spin.assert_not_called()

    # 比赛结束优先于低血回血，取消占点并等待最终停止。
    blackboard.match_progress = 5
    blackboard.self_hp = 50
    tick(NavigationTask.STOP, running)
    tick(NavigationTask.STOP, running)
    node.stop_spin.assert_called_once()
    navigation.owner = None
    tick(NavigationTask.STOP, py_trees.common.Status.FAILURE)
    assert events == [
        "启动占点", "取消占点", "启动回血",
        "取消回血", "启动占点", "取消占点",
    ]
    node.start_chase.assert_not_called()
    node.start_patrol.assert_not_called()

    # 整棵主树被主动打断时，导航和小陀螺各停止一次。
    blackboard.match_progress = 4
    blackboard.self_hp = 300
    tick(NavigationTask.OCCUPY_CENTER, running)
    assert node.start_spin.call_count == 2
    root.stop(py_trees.common.Status.INVALID)
    assert node.stop_occupy.call_count == 3
    assert node.stop_spin.call_count == 2
    root.stop(py_trees.common.Status.INVALID)
    assert node.stop_occupy.call_count == 3
    assert node.stop_spin.call_count == 2

    # 追击被低血任务抢占时，也必须等取消确认后才能开始回血。
    navigation.owner = None
    node.get_control_zone.return_value = dict(
        enabled=True, x_min=0.0, x_max=2.0, y_min=0.0, y_max=2.0
    )
    node.get_robot_position.return_value = (1.0, 1.0)
    node.get_enemy_position.return_value = (1.0, 1.0)
    node.is_chase_ready.return_value = True
    node.start_chase.side_effect = lambda: start("追击")
    node.stop_chase.side_effect = lambda: stop("追击")
    node.get_chase_result.return_value = None
    tick(NavigationTask.CHASE, running)
    tick(NavigationTask.CHASE, running)
    node.start_chase.assert_called_once()
    blackboard.self_hp = 100
    tick(NavigationTask.RESUPPLY, running)
    tick(NavigationTask.RESUPPLY, running)
    node.stop_chase.assert_called_once()
    assert node.start_resupply.call_count == 1
    navigation.owner = None
    tick(NavigationTask.RESUPPLY, running)
    assert node.start_resupply.call_count == 2
    root.stop(py_trees.common.Status.INVALID)
    node.stop_chase.assert_called_once()
    print("主 XML 导航交接、回血双阈值、小陀螺并行与停止测试通过\n")


def _create_tactical_test_tree(
    mode_source="auto", min_hold_seconds=0.0, score_config=None,
    hp_limit=100, hp_up=200,
):
    # 加载实际导航 XML，仅模拟外部接口；评分候选由各测试明确提供。
    blackboard = Blackboard(match_progress=4, self_hp=300)
    node = Mock(
        hp_limit=hp_limit,
        hp_up=hp_up,
        tactical_mode_source=mode_source,
        tactical_score_config=(
            _tactical_score_config() if score_config is None else score_config
        ),
        tactical_confirm_seconds=2.0,
        tactical_min_hold_seconds=min_hold_seconds,
        patrol_min_goal_distance=0.5,
        recovery_wait_seconds=0.5,
        recovery_max_retries=2,
        recovery_min_goal_distance=0.5,
        recovery_sample_attempts=20,
    )
    node.blackboard = blackboard
    node.ros_interface.get_hp_samples.return_value = ()
    node.get_tactical_inputs.side_effect = lambda: (
        BehaviorTreeNode.get_tactical_inputs(node)
    )
    node.get_match_progress.side_effect = lambda: blackboard.match_progress
    node.get_hp.side_effect = lambda: blackboard.self_hp
    node.get_tactical_mode.side_effect = lambda: blackboard.tactical_mode
    node.get_tactical_mode_since.side_effect = lambda: blackboard.tactical_mode_since
    node.get_tactical_candidate.side_effect = lambda: blackboard.pending_tactical_mode
    node.set_tactical_candidate.side_effect = lambda candidate: setattr(
        blackboard, "pending_tactical_mode", candidate
    )
    node.set_tactical_mode.side_effect = lambda mode: (
        BehaviorTreeNode.set_tactical_mode(node, mode)
    )
    node.get_navigation_task.side_effect = lambda: blackboard.navigation_task
    node.set_navigation_task.side_effect = lambda task: setattr(
        blackboard, "navigation_task", task
    )
    # 三种策略都选择占点，单独检查模式更新，不引入追击位置条件。
    node.get_control_zone.return_value = {"enabled": False}
    node.is_chase_ready.return_value = False
    node.is_navigation_stopped.return_value = True
    node.start_occupy.return_value = True
    node.get_occupy_result.return_value = NavigationResult.RUNNING
    node.start_resupply.return_value = True
    node.get_resupply_result.return_value = NavigationResult.RUNNING
    xml_path = Path(__file__).resolve().parent / "tree" / "navigation_tree.xml"
    root = load_behavior_tree(node, xml_path)
    updates = [item for item in root.iterate()
               if isinstance(item, ShouldSwitchTacticalMode)]
    assert len(updates) == 1, "战术 XML 必须包含一个实际的切换条件节点"
    return blackboard, node, root, updates[0]


def test_xml_tactical_mode_confirmation():
    blackboard, node, root, update = _create_tactical_test_tree(
        min_hold_seconds=5.0
    )
    candidate = SimpleNamespace(mode=TacticalMode.DEFENSE)
    update.evaluate_candidate = Mock(
        side_effect=lambda current, inputs: candidate.mode
    )
    clock = SimpleNamespace(now=0.0)

    def tick(at, expected):
        clock.now = at
        root.tick_once()
        assert blackboard.tactical_mode == expected, at
        assert blackboard.navigation_task == NavigationTask.OCCUPY_CENTER

    with patch(
        "rm_application.behavior_tree.conditions.time.monotonic",
        side_effect=lambda: clock.now,
    ):
        # 真实 XML 每轮复位成功的顺序节点，但候选时间必须连续累计。
        tick(0.0, TacticalMode.NEUTRAL)
        tick(1.0, TacticalMode.NEUTRAL)
        tick(2.0, TacticalMode.NEUTRAL)  # 候选已确认，最短保持尚未到期。
        tick(5.0, TacticalMode.DEFENSE)

        # 候选变化后重新确认；同时满足确认与最短保持才切换。
        candidate.mode = TacticalMode.PRESSURE
        tick(6.0, TacticalMode.DEFENSE)
        candidate.mode = TacticalMode.NEUTRAL
        tick(6.5, TacticalMode.DEFENSE)
        candidate.mode = TacticalMode.PRESSURE
        tick(7.0, TacticalMode.DEFENSE)
        tick(9.0, TacticalMode.DEFENSE)
        tick(10.0, TacticalMode.PRESSURE)

        # 暂无候选时保持原模式，数据恢复后重新计时。
        candidate.mode = TacticalMode.DEFENSE
        tick(11.0, TacticalMode.PRESSURE)
        candidate.mode = None
        tick(12.0, TacticalMode.PRESSURE)
        candidate.mode = TacticalMode.DEFENSE
        tick(13.0, TacticalMode.PRESSURE)
        tick(14.9, TacticalMode.PRESSURE)
        tick(15.0, TacticalMode.DEFENSE)

        # 返回当前模式或错误类型都不应启动新的模式切换。
        tick(16.0, TacticalMode.DEFENSE)
        candidate.mode = "PRESSURE"
        tick(20.0, TacticalMode.DEFENSE)
        assert update.candidate_mode is None
        assert node.set_tactical_mode.call_count == 3

        # 无确认延时、无最短保持限制时，可在首次得到候选时切换。
        update.confirm_seconds = 0.0
        update.min_hold_seconds = 0.0
        candidate.mode = TacticalMode.NEUTRAL
        tick(21.0, TacticalMode.NEUTRAL)

    root.stop(py_trees.common.Status.INVALID)
    print("战术 XML 连续确认、最短保持、候选变化和缺失输入测试通过\n")


def test_xml_tactical_confirmation_after_preemption():
    blackboard, _, root, update = _create_tactical_test_tree()
    candidate = SimpleNamespace(mode=TacticalMode.DEFENSE)
    update.evaluate_candidate = Mock(
        side_effect=lambda current, inputs: candidate.mode
    )
    clock = SimpleNamespace(now=0.0)

    def tick(at, expected_task, expected_mode):
        clock.now = at
        root.tick_once()
        assert blackboard.navigation_task == expected_task
        assert blackboard.tactical_mode == expected_mode

    with patch(
        "rm_application.behavior_tree.conditions.time.monotonic",
        side_effect=lambda: clock.now,
    ):
        tick(0.0, NavigationTask.OCCUPY_CENTER, TacticalMode.NEUTRAL)
        # 回血期间不评分，等待时间不能算进普通模式的连续确认。
        blackboard.self_hp = 100
        tick(1.0, NavigationTask.RESUPPLY, TacticalMode.NEUTRAL)
        assert update.evaluate_candidate.call_count == 1
        blackboard.self_hp = 200
        tick(20.0, NavigationTask.OCCUPY_CENTER, TacticalMode.NEUTRAL)
        tick(21.9, NavigationTask.OCCUPY_CENTER, TacticalMode.NEUTRAL)
        tick(22.0, NavigationTask.OCCUPY_CENTER, TacticalMode.DEFENSE)

        # 比赛停止同样抢占普通模式选择，恢复后需要重新确认。
        candidate.mode = TacticalMode.PRESSURE
        tick(23.0, NavigationTask.OCCUPY_CENTER, TacticalMode.DEFENSE)
        blackboard.match_progress = 5
        calls_before_stop = update.evaluate_candidate.call_count
        tick(24.0, NavigationTask.STOP, TacticalMode.DEFENSE)
        assert update.evaluate_candidate.call_count == calls_before_stop
        blackboard.match_progress = 4
        tick(50.0, NavigationTask.OCCUPY_CENTER, TacticalMode.DEFENSE)
        tick(51.9, NavigationTask.OCCUPY_CENTER, TacticalMode.DEFENSE)
        tick(52.0, NavigationTask.OCCUPY_CENTER, TacticalMode.PRESSURE)

    root.stop(py_trees.common.Status.INVALID)
    print("回血与比赛停止抢占后，战术候选重新确认测试通过\n")


def test_tactical_manual_and_auto_control():
    blackboard, node, root, update = _create_tactical_test_tree("manual")
    interface = RosInterface.__new__(RosInterface)
    interface.node = node
    interface.blackboard = blackboard
    update.evaluate_candidate = Mock(return_value=TacticalMode.PRESSURE)
    defense = SimpleNamespace(defense=True, neutral=False, pressure=False)

    # 手动模式接受 RQT 选择，普通 tick 不用评分覆盖它。
    interface.tactical_mode_callback(defense)
    root.tick_once()
    assert blackboard.tactical_mode == TacticalMode.DEFENSE
    update.evaluate_candidate.assert_not_called()
    node.get_tactical_inputs.assert_not_called()
    node.set_tactical_mode.assert_called_once_with(TacticalMode.DEFENSE)
    interface.tactical_mode_callback(defense)
    for message in (
        SimpleNamespace(defense=False, neutral=False, pressure=False),
        SimpleNamespace(defense=True, neutral=True, pressure=False),
    ):
        interface.tactical_mode_callback(message)
    assert node.set_tactical_mode.call_count == 1
    root.stop(py_trees.common.Status.INVALID)

    # 自动模式忽略手动选择；默认全部零权重时保持当前模式。
    blackboard, node, root, update = _create_tactical_test_tree("auto")
    interface.node = node
    interface.blackboard = blackboard
    interface.tactical_mode_callback(defense)
    node.set_tactical_mode.assert_not_called()
    root.tick_once()  # 实际零权重评分返回 None，通过保持分支继续任务。
    assert blackboard.tactical_mode == TacticalMode.NEUTRAL
    assert blackboard.navigation_task == NavigationTask.OCCUPY_CENTER
    root.stop(py_trees.common.Status.INVALID)
    print("RQT 手动控制、自动控制隔离和空评分保持模式测试通过\n")


def test_tactical_input_delivery():
    # 验证真实消息回调、黑板读取和实际 XML 节点之间的数据传递。
    blackboard, node, root, update = _create_tactical_test_tree()
    interface = RosInterface.__new__(RosInterface)
    interface.blackboard = blackboard
    interface.hp_samples = deque(maxlen=256)
    node.ros_interface.get_hp_samples.side_effect = interface.get_hp_samples
    update.evaluate_candidate = Mock(return_value=None)
    for message_type in (Decision, SerialDebug):
        fields = message_type.get_fields_and_field_types()
        assert fields["self_heavy_hp"] == "uint16"
        assert "self_hero_hp" not in fields

    root.tick_once()
    first_inputs = update.evaluate_candidate.call_args.args[1]
    assert first_inputs == {
        "self_hp": 300,
        "heavy_hp": None,
        "infantry_hp": None,
        "remain_bullet": None,
        "hp_samples": (),
    }

    # 使用重新生成的真实消息；收到的 0 与尚未收到的 None 区分。
    with patch(
        "rm_application.behavior_tree.ros_interface.time.monotonic",
        return_value=1.0,
    ):
        interface.decision_callback(Decision(
            match_progress=4, self_sentry_hp=400,
            self_heavy_hp=0, self_infantry_hp=210, remain_bullet=0,
        ))
    root.tick_once()
    update.evaluate_candidate.assert_called_with(TacticalMode.NEUTRAL, {
        "self_hp": 400,
        "heavy_hp": 0,
        "infantry_hp": 210,
        "remain_bullet": 0,
        "hp_samples": ((1.0, 400, 4),),
    })
    assert first_inputs["heavy_hp"] is None

    # 后续每轮读取最新数值，不能一直使用节点创建时的输入。
    with patch(
        "rm_application.behavior_tree.ros_interface.time.monotonic",
        return_value=2.0,
    ):
        interface.decision_callback(Decision(
            match_progress=4, self_sentry_hp=350,
            self_heavy_hp=170, self_infantry_hp=199, remain_bullet=640,
        ))
    root.tick_once()
    update.evaluate_candidate.assert_called_with(TacticalMode.NEUTRAL, {
        "self_hp": 350,
        "heavy_hp": 170,
        "infantry_hp": 199,
        "remain_bullet": 640,
        "hp_samples": ((1.0, 400, 4), (2.0, 350, 4)),
    })
    assert blackboard.tactical_mode == TacticalMode.NEUTRAL
    assert first_inputs["hp_samples"] == (), "历史快照不能随后续回调改变"
    root.stop(py_trees.common.Status.INVALID)
    print("裁判消息到实际评分节点的数据传递、缺失值和零值保留测试通过\n")


def test_tactical_score_calculation():
    _, _, root, condition = _create_tactical_test_tree(
        score_config=_tactical_score_config(
            weights=[1.0, 3.0, 0.0, 0.0],
            maxima=[400.0, 200.0, 0.0, 750.0],
        )
    )
    assert condition.calculate_score({"self_hp": 200, "heavy_hp": 200}) == 0.875
    assert condition.calculate_score({"self_hp": 800, "heavy_hp": 400}) == 1.0
    assert condition.calculate_score({"self_hp": -1, "heavy_hp": -1}) == 0.0
    for bad in (None, float("nan"), float("inf"), True, "bad"):
        assert condition.calculate_score({"self_hp": 200, "heavy_hp": bad}) is None
    root.stop(py_trees.common.Status.INVALID)

    _, _, root, condition = _create_tactical_test_tree(
        score_config=_tactical_score_config(weights=[1.0, 0.0, 0.0, 0.0])
    )
    # 关闭的因子无需提供数据；阈值相等时保留迟滞一侧的模式。
    for current, hp, expected in (
        (TacticalMode.NEUTRAL, 136, TacticalMode.DEFENSE),
        (TacticalMode.NEUTRAL, 140, TacticalMode.NEUTRAL),
        (TacticalMode.NEUTRAL, 300, TacticalMode.NEUTRAL),
        (TacticalMode.NEUTRAL, 304, TacticalMode.PRESSURE),
        (TacticalMode.DEFENSE, 180, TacticalMode.DEFENSE),
        (TacticalMode.DEFENSE, 184, TacticalMode.NEUTRAL),
        (TacticalMode.PRESSURE, 260, TacticalMode.PRESSURE),
        (TacticalMode.PRESSURE, 256, TacticalMode.NEUTRAL),
    ):
        assert condition.evaluate_candidate(current, {"self_hp": hp}) == expected
    root.stop(py_trees.common.Status.INVALID)
    print("加权归一化、缺失输入、零权重和迟滞阈值测试通过\n")


def test_tactical_condition_and_action():
    blackboard, node, root, condition = _create_tactical_test_tree(
        score_config=_tactical_score_config(weights=[1.0, 0.0, 0.0, 0.0])
    )
    action = next(item for item in root.iterate()
                  if isinstance(item, ApplyTacticalMode))
    blackboard.navigation_task = NavigationTask.OCCUPY_CENTER
    blackboard.self_hp = 360
    condition.confirm_seconds = 0.0
    with patch(
        "rm_application.behavior_tree.conditions.time.monotonic",
        return_value=5.0,
    ):
        # 仅执行条件：可以发布候选，但不能修改正式模式。
        condition.tick_once()
        assert condition.status == py_trees.common.Status.SUCCESS
        assert blackboard.pending_tactical_mode == TacticalMode.PRESSURE
        assert blackboard.tactical_mode == TacticalMode.NEUTRAL
        node.set_tactical_mode.assert_not_called()

        # 仅执行动作：消费已确认候选，按实际写入时刻记录保持时间。
        action.tick_once()
        assert action.status == py_trees.common.Status.SUCCESS
        assert blackboard.tactical_mode == TacticalMode.PRESSURE
        assert blackboard.tactical_mode_since == 5.0
        assert blackboard.pending_tactical_mode is None

    with patch(
        "rm_application.behavior_tree.conditions.time.monotonic",
        return_value=8.0,
    ):
        BehaviorTreeNode.set_tactical_mode(node, TacticalMode.PRESSURE)
        assert blackboard.tactical_mode_since == 5.0
        calls = node.set_tactical_mode.call_count
        action.tick_once()
        assert action.status == py_trees.common.Status.FAILURE
        assert node.set_tactical_mode.call_count == calls
    root.stop(py_trees.common.Status.INVALID)
    print("条件只确认、动作应用、重复设置保持时间和空候选保护测试通过\n")


def test_xml_weighted_mode_transitions():
    blackboard, _, root, condition = _create_tactical_test_tree(
        score_config=_tactical_score_config(weights=[1.0, 0.0, 0.0, 0.0])
    )
    clock = SimpleNamespace(now=0.0)

    def tick(at, hp, expected):
        clock.now = at
        blackboard.self_hp = hp
        root.tick_once()
        assert blackboard.tactical_mode == expected
        assert blackboard.navigation_task == NavigationTask.OCCUPY_CENTER
        assert blackboard.pending_tactical_mode is None
        assert root.status == py_trees.common.Status.RUNNING

    with patch(
        "rm_application.behavior_tree.conditions.time.monotonic",
        side_effect=lambda: clock.now,
    ):
        tick(0.0, 320, TacticalMode.NEUTRAL)
        tick(1.0, 320, TacticalMode.NEUTRAL)
        tick(2.0, 320, TacticalMode.PRESSURE)
        tick(3.0, 280, TacticalMode.PRESSURE)
        tick(4.0, 256, TacticalMode.PRESSURE)
        tick(6.0, 256, TacticalMode.NEUTRAL)
        tick(7.0, 136, TacticalMode.NEUTRAL)
        tick(9.0, 136, TacticalMode.DEFENSE)
        tick(10.0, 160, TacticalMode.DEFENSE)
        tick(11.0, 184, TacticalMode.DEFENSE)
        tick(13.0, 184, TacticalMode.NEUTRAL)
    root.stop(py_trees.common.Status.INVALID)
    print("实际 XML 的真实评分、条件失败兜底和三档模式切换测试通过\n")


def test_tactical_score_config_validation():
    for changes in (
        {"weights": [-1.0, 0.0, 0.0, 0.0]},
        {"weights": [float("nan"), 0.0, 0.0, 0.0]},
        {"weights": [1.0]},
        {"factors": ["self_hp"] * 4},
        {"weights": [0.0, 1.0, 0.0, 0.0]},
        {"maxima": [float("inf"), 0.0, 0.0, 750.0]},
        {"defense_enter": 0.5},
        {"pressure_enter": float("nan")},
        {"damage_window_seconds": 0.0},
        {"damage_window_seconds": float("inf")},
        {"damage_full_rate": -1.0},
        {"damage_full_rate": float("nan")},
        {"damage_input_timeout": 0.0},
    ):
        try:
            ShouldSwitchTacticalMode(
                name="invalid-config",
                get_tactical_mode=lambda: TacticalMode.NEUTRAL,
                get_tactical_mode_since=lambda: 0.0,
                set_tactical_candidate=Mock(),
                get_navigation_task=lambda: NavigationTask.OCCUPY_CENTER,
                get_tactical_inputs=lambda: {},
                mode_source="auto",
                confirm_seconds=2.0,
                min_hold_seconds=0.0,
                score_config=_tactical_score_config(**changes),
            )
        except ValueError:
            pass
        else:
            raise AssertionError(changes)
    print("评分权重、上限和阈值配置检查测试通过\n")


def _configured_damage_score():
    # 从实际 YAML 读取配置，覆盖本轮使用的权重、上限及统计参数。
    config_path = (
        Path(__file__).resolve().parents[2]
        / "config" / "behavior_tree_params.yaml"
    )
    parameters = yaml.safe_load(config_path.read_text())["behavior_tree_node"]["ros__parameters"]
    score_config = {
        key.removeprefix("tactical_score_"): value
        for key, value in parameters.items()
        if key.startswith("tactical_score_")
    }
    return parameters, score_config


def test_damage_safety_window():
    _, config = _configured_damage_score()
    _, _, root, condition = _create_tactical_test_tree(score_config=config)

    def safety(at, samples):
        with patch(
            "rm_application.behavior_tree.conditions.time.monotonic",
            return_value=at,
        ):
            return condition.calculate_damage_safety(samples)

    steady = tuple((t, 400, 4) for t in (0.0, 0.5, 1.0, 1.5, 2.0))
    assert safety(2.0, steady) == 1.0
    loss = ((0.0, 400, 4), (0.5, 350, 4), (1.0, 325, 4),
            (1.5, 300, 4), (2.0, 300, 4))
    assert safety(2.0, loss) == 0.0

    # 总共下降 75、回复 25：应统计 75，而不是净下降的 50。
    healing = ((0.0, 400, 4), (0.5, 350, 4), (1.0, 375, 4),
               (1.5, 350, 4), (2.0, 350, 4))
    assert safety(2.0, healing) == 0.25
    later = healing + tuple((t, 350, 4) for t in (2.5, 3.0, 3.5, 4.0))
    assert safety(4.0, later) == 1.0

    with patch(
        "rm_application.behavior_tree.conditions.time.monotonic",
        return_value=2.0,
    ):
        score = condition.calculate_score({
            "self_hp": 200, "infantry_hp": 350, "hp_samples": healing,
        })
        assert abs(score - 0.625) < 1e-9
        # 零权重的重装和发弹量无需提供数据。
    root.stop(py_trees.common.Status.INVALID)
    print("实际 YAML 的掉血速率、回血不抵消、窗口过期和 60/30/10 评分测试通过\n")


def test_damage_input_validity():
    _, config = _configured_damage_score()
    _, _, root, condition = _create_tactical_test_tree(score_config=config)
    steady = tuple((t, 400, 4) for t in (0.0, 0.5, 1.0, 1.5, 2.0))
    cases = (
        (2.0, ()),                         # 未收到数据
        (1.0, steady[:3]),                # 未积累完整窗口
        (3.01, steady),                   # 最新消息过期
        (2.0, ((0.0, 400, 4), (2.0, 300, 4))),  # 中间接收中断
        (1.0, steady),                    # 时间异常
        (2.0, ((0.0, 400, 0),) + steady[1:]),   # 窗口包含非比赛阶段
        (2.0, steady[:-1] + ((2.0, 0, 4),)),    # 零血不能当成有效掉血趋势
    )
    for at, samples in cases:
        with patch(
            "rm_application.behavior_tree.conditions.time.monotonic",
            return_value=at,
        ):
            assert condition.calculate_damage_safety(samples) is None
            assert condition.calculate_score({
                "self_hp": 400, "infantry_hp": 350, "hp_samples": samples,
            }) is None

    reconnect = steady + tuple((t, 300, 4) for t in (10.0, 10.5, 11.0, 11.5, 12.0))
    with patch(
        "rm_application.behavior_tree.conditions.time.monotonic",
        return_value=11.0,
    ):
        assert condition.calculate_damage_safety(reconnect[:-2]) is None
    with patch(
        "rm_application.behavior_tree.conditions.time.monotonic",
        return_value=12.0,
    ):
        assert condition.calculate_damage_safety(reconnect) == 1.0
    root.stop(py_trees.common.Status.INVALID)
    print("掉血样本不足、接收中断、阶段变化、零血和恢复后重新累计测试通过\n")


def test_xml_damage_score_and_resupply():
    parameters, config = _configured_damage_score()
    blackboard, node, root, condition = _create_tactical_test_tree(
        score_config=config,
        hp_limit=parameters["hp_limit"], hp_up=parameters["hp_up"],
    )
    interface = RosInterface.__new__(RosInterface)
    interface.blackboard = blackboard
    interface.hp_samples = deque(maxlen=256)
    node.ros_interface.get_hp_samples.side_effect = interface.get_hp_samples
    clock = SimpleNamespace(now=0.0)

    def tick(at, hp, mode, task):
        clock.now = at
        interface.decision_callback(SimpleNamespace(
            match_progress=4, self_sentry_hp=hp, self_heavy_hp=0,
            self_infantry_hp=350, remain_bullet=0,
        ))
        root.tick_once()
        assert blackboard.tactical_mode == mode
        assert blackboard.navigation_task == task
        assert root.status == py_trees.common.Status.RUNNING

    occupy, resupply = NavigationTask.OCCUPY_CENTER, NavigationTask.RESUPPLY
    neutral, pressure = TacticalMode.NEUTRAL, TacticalMode.PRESSURE
    with patch(
        "rm_application.behavior_tree.conditions.time.monotonic",
        side_effect=lambda: clock.now,
    ):
        # 两秒积累样本，再连续确认两秒，才应用前压。
        for at in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5):
            tick(at, 400, neutral, occupy)
        tick(4.0, 400, pressure, occupy)

        # 短时掉血后停在 220：压力消退早于连续确认完成，仍保留前压。
        tick(4.5, 240, pressure, occupy)
        tick(5.0, 220, pressure, occupy)
        for at in (5.5, 6.0, 6.5, 7.0):
            tick(at, 220, pressure, occupy)

        # 160 尚未触发回血，但总分持续低于退出前压阈值，切到中立。
        for at in (7.5, 8.0, 8.5, 9.0):
            tick(at, 160, pressure, occupy)
        tick(9.5, 160, neutral, occupy)

        # 实际 150/400 双阈值：低血抢占，200 血仍继续补给。
        tick(10.0, 150, neutral, resupply)
        tick(10.5, 200, neutral, resupply)
        tick(11.0, 400, neutral, occupy)
        for at in (11.5, 12.0, 12.5):
            tick(at, 400, neutral, occupy)
        tick(13.0, 400, pressure, occupy)

        # 消息超时保留当前策略，同时清除待应用候选。
        clock.now = 14.5
        root.tick_once()
        assert condition.last_score is None
        assert blackboard.tactical_mode == pressure
        assert blackboard.pending_tactical_mode is None
    root.stop(py_trees.common.Status.INVALID)
    print("血量消息到实际 XML 评分、模式切换、150/400 回血抢占与消息超时测试通过\n")


@contextmanager
def _rqt_readonly_types():
    """本次调试进程内锁定字段类型，避免 RQT 的类型编辑异常。"""
    from python_qt_binding.QtCore import Qt
    from rqt_publisher.publisher_tree_model import PublisherTreeModel

    original_flags = PublisherTreeModel.flags
    original_set_data = PublisherTreeModel.setData

    def flags(model, index):
        result = original_flags(model, index)
        if index.column() == model._column_index["type"]:
            result &= ~Qt.ItemIsEditable
        return result

    def set_data(model, index, value, role=Qt.EditRole):
        if role == Qt.EditRole and index.column() == model._column_index["type"]:
            return False
        return original_set_data(model, index, value, role)

    with patch.object(PublisherTreeModel, "flags", flags), patch.object(
        PublisherTreeModel, "setData", set_data
    ):
        yield


def run_rqt():
    """启动本轮调试使用的 RQT，不修改系统安装的插件文件。"""
    from rqt_gui.main import Main

    print("[RQT] 字段类型栏已锁定；请在 expression 栏编辑消息内容。", flush=True)
    with _rqt_readonly_types():
        return Main().main()


def test_rqt_expression_editing():
    from types import MethodType
    from python_qt_binding.QtCore import QCoreApplication, Qt
    from rqt_publisher.publisher import Publisher
    from rqt_publisher.publisher_tree_model import PublisherTreeModel
    from std_msgs.msg import String

    # 只检查 Qt 表格编辑模型，无需桌面窗口和 ROS 网络。
    app = QCoreApplication.instance() or QCoreApplication([])
    model = PublisherTreeModel()
    info = {
        "publisher_id": 0, "topic_name": "/behavior_tree/test/monitor_view",
        "message_instance": String(), "enabled": False,
        "expressions": {}, "rate": 1.0, "counter": 0,
    }
    model.add_publisher(info)
    parent = model.index(0, 0)
    type_index = model.index(0, 1, parent)
    expression_index = model.index(0, 3, parent)
    original_type = model.data(type_index, Qt.DisplayRole)
    publisher = SimpleNamespace(_eval_locals={})
    for name in ("_extract_array_info", "_evaluate_expression", "_fill_message_slots"):
        setattr(publisher, name, MethodType(getattr(Publisher, name), publisher))
    columns = []

    def change(_publisher_id, topic, column, value, setter):
        columns.append(column)
        assert column == "expression", "不能进入会导致崩溃的类型修改处理"
        result = Publisher._change_publisher_expression(publisher, info, topic, value)
        if result is not None:
            setter(result)

    model.item_value_changed.connect(change)
    with _rqt_readonly_types():
        assert not model.flags(type_index) & Qt.ItemIsEditable
        assert model.setData(type_index, "navigation", Qt.EditRole) is False
        assert model.data(type_index, Qt.DisplayRole) == original_type
        assert model.flags(expression_index) & Qt.ItemIsEditable
        assert model.setData(expression_index, "'navigation'", Qt.EditRole)
        assert info["message_instance"].data == "navigation"
        assert columns == ["expression"]
        assert model.setData(expression_index, "'strategy'", Qt.EditRole)
        assert info["message_instance"].data == "strategy"
    assert app is not None
    print("RQT 类型栏保护、实际 Expression 编辑和 String 消息内容更新测试通过\n")


class _GrootMonitor:
    """按 Groot2 协议发布实际运行树，只提供实时观察功能。

    协议依据 BehaviorTree.CPP 4.6.2 的 groot2_protocol.h 和
    groot2_publisher.cpp；不实现断点、节点替换或日志录制。
    """

    def __init__(self, root, port=1667, tree_id="SentryMain", max_depth=None):
        import struct
        import uuid
        import xml.etree.ElementTree as ET
        from threading import Event, Thread

        try:
            import zmq
        except ImportError as exc:
            raise RuntimeError(
                "Groot 监视需要 pyzmq，请先运行："
                "python3 -m pip install --user pyzmq"
            ) from exc

        self._zmq = zmq
        self._struct = struct
        self._nodes = []
        self._codes = []
        self._uuid = uuid.uuid4().bytes
        self._stop = Event()
        self._ready = Event()
        self._error = None
        models = {}
        document = ET.Element("root", BTCPP_format="4",
                              main_tree_to_execute=tree_id)
        tree = ET.SubElement(document, "BehaviorTree",
                             ID=tree_id, _fullpath="")

        def add_node(node, parent, depth=0):
            uid = len(self._nodes) + 1
            if uid > 65535:
                raise ValueError("Groot 节点编号超出协议范围")
            self._nodes.append(node)
            tag = type(node).__name__
            collapsed = bool(node.children) and max_depth is not None and depth >= max_depth
            if collapsed:
                # 总览只折叠显示后续任务，状态仍读取同一个运行实例。
                tag = "CollapsedSubtree"
                kind = "Action"
            elif isinstance(node, py_trees.composites.Composite):
                kind = "Control"
            elif isinstance(node, py_trees.decorators.Decorator):
                kind = "Decorator"
            elif type(node).__module__.endswith(".conditions"):
                kind = "Condition"
            else:
                kind = "Action"
            models[tag] = kind
            element = ET.SubElement(parent, tag, name=node.name.rsplit(".", 1)[-1],
                                    _uid=str(uid))
            if not collapsed:
                for child in node.children:
                    add_node(child, element, depth + 1)

        # 直接遍历正在 tick 的实例，不另建一棵执行树。
        # py_trees 加载 XML 时已展开 SubTree，这里也发送展开后的结构。
        add_node(root, tree)
        model_root = ET.SubElement(document, "TreeNodesModel")
        for tag, kind in sorted(models.items()):
            ET.SubElement(model_root, kind, ID=tag)
        self._xml = ET.tostring(document, encoding="utf-8")
        self._codes = [0] * len(self._nodes)
        self._snapshot = b""
        self.update()
        self._thread = Thread(target=self._serve, args=(port,),
                              name="groot_monitor", daemon=True)
        self._thread.start()
        if not self._ready.wait(5.0):
            self.close()
            raise RuntimeError("Groot 监视端口启动超时")
        if self._error is not None:
            self.close()
            raise RuntimeError(
                f"Groot 监视端口启动失败：{self._error}"
            ) from self._error
        print(
            f"[Groot] 监视地址 127.0.0.1:{port}，"
            f"实际运行节点共 {len(self._nodes)} 个；"
            "完整显示仍受 Groot 当前许可证限制。",
            flush=True,
        )

    def update(self):
        # 显式映射两个库的状态，不能直接使用 py_trees 的枚举数值。
        states = {
            py_trees.common.Status.INVALID: 0,
            py_trees.common.Status.RUNNING: 1,
            py_trees.common.Status.SUCCESS: 2,
            py_trees.common.Status.FAILURE: 3,
        }
        codes = []
        for previous, node in zip(self._codes, self._nodes):
            code = states[node.status]
            if code == 0:
                # Groot 用 11/12/13 表示从运行/成功/失败回到空闲。
                code = previous + 10 if previous in (1, 2, 3) else previous
            codes.append(code)
        snapshot = b"".join(
            self._struct.pack("<HB", uid, code)
            for uid, code in enumerate(codes, start=1)
        )
        self._codes = codes
        # 不可变 bytes 整体替换，服务线程只读取完成的一轮快照。
        self._snapshot = snapshot

    def _serve(self, port):
        zmq = self._zmq
        context = zmq.Context()
        server = context.socket(zmq.REP)
        publisher = context.socket(zmq.PUB)
        try:
            for socket in (server, publisher):
                socket.setsockopt(zmq.LINGER, 0)
                socket.setsockopt(zmq.SNDTIMEO, 250)
            server.bind(f"tcp://127.0.0.1:{port}")
            publisher.bind(f"tcp://127.0.0.1:{port + 1}")
            self._ready.set()
            while not self._stop.is_set():
                if not server.poll(100):
                    continue
                request = server.recv_multipart()
                if not request or len(request[0]) != 6:
                    server.send_multipart([b"error", b"wrong request header"])
                    continue
                protocol, kind, request_id = self._struct.unpack(
                    "<BBI", request[0]
                )
                if protocol != 2:
                    server.send_multipart([b"error", b"unsupported protocol"])
                    continue
                header = self._struct.pack(
                    "<BBI", 2, kind, request_id
                ) + self._uuid
                if kind == ord("T"):
                    reply = [header, self._xml]
                elif kind == ord("S"):
                    reply = [header, self._snapshot]
                elif kind == ord("D"):
                    reply = [header, b"[]"]
                elif kind in (ord("A"), ord("X")):
                    # 没有安装过断点，清空或禁用操作无需改变执行树。
                    reply = [header]
                elif kind == ord("B"):
                    reply = [header, b"{}"]
                else:
                    reply = [
                        b"error",
                        b"read-only live monitor; hooks and recording unsupported",
                    ]
                server.send_multipart(reply)
        except Exception as exc:
            self._error = exc
            self._stop.set()
            print(f"[Groot] 监视服务停止：{exc}", flush=True)
        finally:
            self._ready.set()
            server.close()
            publisher.close()
            context.term()

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            raise RuntimeError("Groot 监视线程尚未退出")


def run_interactive_test(groot=False, groot_navigation=False, groot_view=None):
    import time
    import rm_application.behavior_tree_node as bt_module
    import rm_application.behavior_tree.ros_interface as interface_module
    from rm_application.behavior_tree.registry import NavigationSwitch
    from std_msgs.msg import String

    view_ids = {
        "full": "SentryMain",
        "overview": "SentryOverview",
        "strategy": "TacticalOverview",
        "switching": "TacticalModeSwitching",
        "navigation": "NavigationExecution",
        "stop": "StopSelection",
        "resupply": "ResupplySelection",
        "defense": "DefenseStrategy",
        "neutral": "NeutralStrategy",
        "pressure": "PressureStrategy",
        "stance": "StanceControl",
    }
    view_depths = {"overview": 2, "strategy": 3}
    if groot_view is not None and groot_view not in view_ids:
        raise ValueError(f"未知监视范围 {groot_view!r}，可选：{', '.join(view_ids)}")
    monitoring_enabled = groot or groot_navigation or groot_view is not None
    requested_view = groot_view or ("navigation" if groot_navigation else "full")

    class FakeNavigator:
        def __init__(self):
            self.position = (0.0, 0.0)
            self.target = None
            self.deadline = 0.0
            self.done = True
            self.canceling = False
            self.result = TaskResult.UNKNOWN

        def waitUntilNav2Active(self):
            # 离线测试不等待真实 Nav2。
            pass

        def goToPose(self, pose):
            if not self.done:
                return False
            self.target = (
                pose.pose.position.x,
                pose.pose.position.y,
            )
            self.deadline = time.monotonic() + 2.0
            self.done = False
            self.canceling = False
            self.result = TaskResult.UNKNOWN
            print(f"[假导航] 开始前往 {self.target}", flush=True)
            return True

        def isTaskComplete(self):
            if not self.done and time.monotonic() >= self.deadline:
                self.done = True
                if self.canceling:
                    self.result = TaskResult.CANCELED
                else:
                    self.position = self.target
                    self.result = TaskResult.SUCCEEDED
            return self.done

        def getResult(self):
            return self.result

        def cancelTask(self):
            if not self.done and not self.canceling:
                # 留出取消确认时间，检查任务切换时的等待逻辑。
                self.canceling = True
                self.deadline = time.monotonic() + 0.3
                print("[假导航] 请求取消，等待确认", flush=True)

        def destroy_node(self):
            pass

    navigator = FakeNavigator()
    original_tick = bt_module.BehaviorTreeNode.tick_behavior_tree
    previous_output = None
    monitor = None
    active_view = None
    views = None
    view_subscription = None

    def select_view(message):
        nonlocal requested_view
        if message.data not in view_ids:
            print(
                f"[Groot] 未知监视范围 {message.data!r}，"
                f"可选：{', '.join(view_ids)}",
                flush=True,
            )
            return
        # 只更改监视范围，不修改战术模式、任务牌或树的执行状态。
        requested_view = message.data

    def tick_and_display(node):
        nonlocal previous_output, monitor, active_view, views, view_subscription
        if monitoring_enabled and views is None:
            behaviors = list(node.behavior_tree.iterate())

            def find_view(name):
                matches = [
                    behavior for behavior in behaviors
                    if behavior.name.rsplit(".", 1)[-1] == name
                ]
                if len(matches) != 1:
                    raise RuntimeError(f"监视范围 {name!r} 必须唯一，实际找到 {len(matches)} 个")
                return matches[0]

            switches = [
                behavior for behavior in behaviors
                if isinstance(behavior, NavigationSwitch)
            ]
            if len(switches) != 1:
                raise RuntimeError("导航执行树必须有且只有一个任务切换节点")
            views = {
                "full": node.behavior_tree,
                "overview": node.behavior_tree,
                "strategy": find_view("战术模式更新与任务选择"),
                "switching": find_view("尝试更新战术模式"),
                "navigation": switches[0],
                "stop": find_view("停止分支"),
                "resupply": find_view("回血分支"),
                "defense": find_view("防守模式"),
                "neutral": find_view("中立模式"),
                "pressure": find_view("前压模式"),
                "stance": find_view("姿态控制"),
            }
            view_subscription = node.create_subscription(
                String, "/behavior_tree/test/monitor_view", select_view, 10
            )
            def visible_size(root, limit=None, depth=0):
                if limit is not None and depth >= limit:
                    return 1
                return 1 + sum(visible_size(child, limit, depth + 1) for child in root.children)

            sizes = ", ".join(
                f"{name}={visible_size(root, view_depths.get(name))}"
                for name, root in views.items()
            )
            print(f"[Groot] 各监视范围的节点数：{sizes}", flush=True)
        if monitoring_enabled and active_view != requested_view:
            if monitor is not None:
                monitor.close()
                monitor = None
            # 同一时刻只发布所选范围；全部范围仍属于同一棵执行树。
            monitor = _GrootMonitor(
                views[requested_view], tree_id=view_ids[requested_view],
                max_depth=view_depths.get(requested_view),
            )
            active_view = requested_view
            print(
                f"[Groot] 当前监视范围：{active_view}。"
                "切换后请在 Groot 中重新 Connect，执行树无需重启。",
                flush=True,
            )
        original_tick(node)
        if monitor is not None:
            monitor.update()
        score_condition = next(
            item for item in node.behavior_tree.iterate()
            if isinstance(item, ShouldSwitchTacticalMode)
        )
        if node.get_navigation_task() in (NavigationTask.STOP, NavigationTask.RESUPPLY):
            score_text = "暂停"
        elif score_condition.last_score is None:
            score_text = "暂无有效评分"
        else:
            score_text = f"{score_condition.last_score:.3f}"
        summary = (
            f"比赛阶段={node.get_match_progress()} "
            f"血量={node.get_hp()} "
            f"评分={score_text} "
            f"策略={node.get_tactical_mode().value} "
            f"任务={node.get_navigation_task().value}"
        )
        tree_text = py_trees.display.unicode_tree(
            node.behavior_tree,
            show_status=True,
        )
        output = summary + "\n" + tree_text
        # 状态变化时才打印，避免每秒打印十遍同样的树。
        if output != previous_output:
            print("\n" + output, flush=True)
            previous_output = output

    # 保留真实 ROS 订阅、黑板、XML 和行为节点。
    # 只替换本进程的 Nav2 客户端、位置查询和 tick 显示。
    with patch.object(
        interface_module, "BasicNavigator", return_value=navigator
    ), patch.object(
        interface_module.RosInterface,
        "get_robot_position",
        lambda self: navigator.position,
    ), patch.object(
        bt_module.BehaviorTreeNode,
        "tick_behavior_tree",
        tick_and_display,
    ):
        try:
            bt_module.main()
        finally:
            if monitor is not None:
                monitor.close()


if __name__ == "__main__":
    import sys

    if "--rqt" in sys.argv:
        sys.argv.remove("--rqt")
        raise SystemExit(run_rqt())
    elif "--interactive" in sys.argv:
        # 这是测试入口的选项，不传给 rclpy 的 ROS 参数解析器。
        sys.argv.remove("--interactive")
        groot_enabled = "--groot" in sys.argv
        if groot_enabled:
            sys.argv.remove("--groot")
        groot_navigation = "--groot-navigation" in sys.argv
        if groot_navigation:
            sys.argv.remove("--groot-navigation")
        groot_view = None
        if "--groot-view" in sys.argv:
            option_index = sys.argv.index("--groot-view")
            if option_index + 1 >= len(sys.argv):
                raise SystemExit("--groot-view 后面需要指定监视范围")
            groot_view = sys.argv.pop(option_index + 1)
            sys.argv.pop(option_index)
        run_interactive_test(
            groot=groot_enabled,
            groot_navigation=groot_navigation,
            groot_view=groot_view,
        )
    else:
        test_chase_cancel_confirmation()
        test_chase_server_waits_for_navigation_cancel()
        test_chase_reports_navigation_failure()
        test_chase_navigation_success_continues()
        test_chase_cooldown_state()
        test_chase_selection_respects_cooldown()
        test_patrol_region_switch_waits_for_cancel()
        test_xml_patrol_recovery_uses_alternative_goal()
        test_retry_wait_restarts_after_interrupt()
        test_xml_navigation_arrival_refreshes_retry_budget()
        test_xml_navigation_task_selection()
        test_xml_tactical_mode_confirmation()
        test_xml_tactical_confirmation_after_preemption()
        test_tactical_manual_and_auto_control()
        test_tactical_input_delivery()
        test_tactical_score_calculation()
        test_tactical_condition_and_action()
        test_xml_weighted_mode_transitions()
        test_tactical_score_config_validation()
        test_damage_safety_window()
        test_damage_input_validity()
        test_xml_damage_score_and_resupply()
        test_rqt_expression_editing()
        print("当前导航、XML 主树、战术评分与 RQT 编辑的 23 项测试全部通过")
