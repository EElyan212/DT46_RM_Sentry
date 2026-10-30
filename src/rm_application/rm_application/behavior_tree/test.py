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

from rm_application.behavior_tree.actions import PatrolZone
from rm_application.behavior_tree.blackboard import Blackboard
from rm_application.behavior_tree.builder import BehaviorTreeBuilder
from rm_application.behavior_tree.core import Status
from rm_application.behavior_tree.nav_zone import (
    NavigationResult,
    NavigationTask,
    PatrolRegion,
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
    state = SimpleNamespace(task=NavigationTask.CHASE, ready=False)
    node = SimpleNamespace(
        is_chase_ready=lambda: state.ready,
        get_navigation_task=lambda: state.task,
        set_navigation_task=lambda task: setattr(state, "task", task),
        set_patrol_region=Mock(),
        get_enemy_position=lambda: (1.0, 1.0),
        get_robot_position=lambda: (1.0, 1.0),
        get_control_zone=lambda: dict(
            enabled=True, x_min=0.0, x_max=2.0, y_min=0.0, y_max=2.0
        ),
        get_chase_boundary=lambda: dict(
            enabled=True, x_min=-10.0, x_max=10.0, y_min=-10.0, y_max=10.0
        ),
        is_enemy_within_lost_grace=lambda: False,
    )
    builder = BehaviorTreeBuilder(node)
    # 目标始终有效；两个实际策略都必须在冷却期间选择巡逻。
    for build in (
        builder._build_neutral_task_selection,
        builder._build_pressure_task_selection,
    ):
        state.task = NavigationTask.CHASE
        state.ready = False
        selection = build()
        assert selection.tick() == Status.SUCCESS
        assert state.task == NavigationTask.PATROL, (
            f"{build.__name__} 冷却期间仍然选择追击"
        )
        state.ready = True
        assert selection.tick() == Status.SUCCESS
        assert state.task == NavigationTask.CHASE
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
    assert action.tick() == Status.RUNNING
    state.region = PatrolRegion.ENEMY
    assert action.tick() == Status.RUNNING
    stop.assert_called_once()
    assert start.call_count == 1
    # 等待期间再次换区，应等旧导航结束后直接使用最新区域。
    state.region = PatrolRegion.HOME
    assert action.tick() == Status.RUNNING
    assert start.call_count == 1
    state.stopped = True
    assert action.tick() == Status.RUNNING
    stop.assert_called_once()
    assert [call.args[0] for call in start.call_args_list] == [
        points[PatrolRegion.CENTER], points[PatrolRegion.HOME],
    ]
    assert action.active_region == PatrolRegion.HOME
    action.get_patrol_result.assert_not_called()
    print("巡逻换区等待取消完成、等待中使用最新区域的测试通过\n")


def test_xml_navigation_task_selection():
    # 加载实际 XML 和已有执行树，仅模拟 ROS 数据及导航结果。
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
    xml_path = Path(__file__).resolve().parent / "tree" / "navigation_tree.xml"
    root = load_behavior_tree(node, xml_path)

    def tick(expected_task, expected_status):
        root.tick_once()
        assert blackboard.navigation_task == expected_task
        assert root.status == expected_status

    running = py_trees.common.Status.RUNNING
    # 已确认停止时，沿用原执行器无活动任务返回 FAILURE 的约定。
    tick(NavigationTask.STOP, py_trees.common.Status.FAILURE)
    assert events == []

    blackboard.match_progress = 4
    tick(NavigationTask.OCCUPY_CENTER, running)
    tick(NavigationTask.OCCUPY_CENTER, running)
    node.start_occupy.assert_called_once()

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

    # 比赛结束优先于低血回血，取消占点并等待最终停止。
    blackboard.match_progress = 5
    blackboard.self_hp = 50
    tick(NavigationTask.STOP, running)
    tick(NavigationTask.STOP, running)
    navigation.owner = None
    tick(NavigationTask.STOP, py_trees.common.Status.FAILURE)
    assert events == [
        "启动占点", "取消占点", "启动回血",
        "取消回血", "启动占点", "取消占点",
    ]
    node.start_chase.assert_not_called()
    node.start_patrol.assert_not_called()
    print("XML 选择与执行、回血阈值保持及导航取消交接测试通过\n")


if __name__ == "__main__":
    test_chase_cancel_confirmation()
    test_chase_server_waits_for_navigation_cancel()
    test_chase_reports_navigation_failure()
    test_chase_navigation_success_continues()
    test_chase_cooldown_state()
    test_chase_selection_respects_cooldown()
    test_patrol_region_switch_waits_for_cancel()
    test_xml_navigation_task_selection()
    print("当前导航收尾与 XML 导航流程的 8 项测试全部通过")
