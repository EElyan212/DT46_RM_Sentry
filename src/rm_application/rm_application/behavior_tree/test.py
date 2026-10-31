"""导航收尾和 XML 导航流程测试。

使用模拟对象调用实际回调，不连接 Nav2，不代替仿真或实车联调。
"""

from concurrent.futures import Future
from pathlib import Path
from threading import Lock
from types import SimpleNamespace
from unittest.mock import Mock, patch

import py_trees

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult

from rm_application.behavior_tree.actions import PatrolZone, WaitBeforeRetry
from rm_application.behavior_tree.blackboard import Blackboard
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
        patrol_min_goal_distance=0.5,
        recovery_wait_seconds=0.5,
        recovery_max_retries=2,
        recovery_min_goal_distance=0.5,
        recovery_sample_attempts=20,
    )
    node.get_tactical_mode.side_effect = lambda: blackboard.tactical_mode
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
        patrol_min_goal_distance=0.5,
        recovery_wait_seconds=0.0,
        recovery_max_retries=2,
        recovery_min_goal_distance=2.0,
        recovery_sample_attempts=3,
    )
    node.get_match_progress.side_effect = lambda: blackboard.match_progress
    node.get_hp.side_effect = lambda: blackboard.self_hp
    node.get_tactical_mode.side_effect = lambda: blackboard.tactical_mode
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
            recovery_wait_seconds=0.0,
            recovery_max_retries=1,
            recovery_min_goal_distance=0.5,
            recovery_sample_attempts=20,
            patrol_min_goal_distance=0.5,
        )
        node.get_match_progress.side_effect = lambda: blackboard.match_progress
        node.get_hp.side_effect = lambda: blackboard.self_hp
        node.get_tactical_mode.side_effect = lambda: blackboard.tactical_mode
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
        recovery_wait_seconds=0.5,
        recovery_max_retries=2,
        recovery_min_goal_distance=0.5,
        recovery_sample_attempts=20,
        patrol_min_goal_distance=0.5,
    )
    node.get_match_progress.side_effect = lambda: blackboard.match_progress
    node.get_hp.side_effect = lambda: blackboard.self_hp
    node.get_tactical_mode.side_effect = lambda: blackboard.tactical_mode
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


if __name__ == "__main__":
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
    print("当前导航收尾与 XML 主树流程的 11 项测试全部通过")
