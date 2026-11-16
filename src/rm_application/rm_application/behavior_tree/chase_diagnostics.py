"""只读汇总行为树未执行追击的原因，不发起或取消任务。"""

import time

import py_trees

from rm_application.behavior_tree.actions import ChaseEnemy
from rm_application.behavior_tree.conditions import (
    CanChase, CanChaseInsideCenterZone, IsChaseReady, IsTacticalMode, NeedEnterCenterZone,
)
from rm_application.behavior_tree.nav_zone import NavigationTask, TacticalMode
from rm_application.behavior_tree.recovery import RecoveryNode


TASK_NAMES = {
    NavigationTask.STOP: "停止",
    NavigationTask.RESUPPLY: "回补给",
    NavigationTask.OCCUPY_CENTER: "回中央点",
    NavigationTask.PATROL: "巡逻",
    NavigationTask.CHASE: "追击",
}
OWNER_NAMES = {"occupy": "回中央点", "resupply": "回补给", "patrol": "巡逻"}
MODE_NAMES = {
    TacticalMode.DEFENSE: "防守",
    TacticalMode.NEUTRAL: "中立",
    TacticalMode.PRESSURE: "前压",
}


class ChaseDiagnostics:
    def __init__(self, node):
        self.node = node
        items = list(node.behavior_tree.iterate())
        # 各战术分支有同类型条件，必须按所属模式分别保存。
        self.conditions = {}
        for item in items:
            if not isinstance(item, (
                CanChase, CanChaseInsideCenterZone, IsChaseReady, NeedEnterCenterZone,
            )):
                continue
            parent = item.parent
            while parent is not None:
                mode_condition = next((
                    child for child in parent.children
                    if isinstance(child, IsTacticalMode)
                ), None)
                if mode_condition is not None:
                    self.conditions[(mode_condition.expected_mode, type(item))] = item
                    break
                parent = parent.parent
        self.recovery = next((
            item for item in items
            if isinstance(item, RecoveryNode) and isinstance(item.action, ChaseEnemy)
        ), None)
        self.last_key = None
        self.last_print_at = float("-inf")

    def _vision(self):
        node = self.node
        interface = node.ros_interface
        enemy = interface.latest_enemy
        tracked_at = interface.last_enemy_tracked_time
        if enemy is None:
            return "missing", "行为树视觉=尚未收到目标消息"

        age = None if tracked_at is None else (
            node.get_clock().now() - tracked_at
        ).nanoseconds / 1e9
        age_text = "从未收到有效跟踪" if age is None else f"距最后有效跟踪={age:.3f}s"
        if not enemy.tracked:
            code, state = "lost", "无效，tracked=false"
        elif age is None or not 0.0 <= age <= node.vision_timeout:
            code, state = "expired", "无效，tracked=true但有效跟踪时间已过期或异常"
        else:
            code, state = "valid", "有效，tracked=true"
        return code, (
            f"行为树视觉={state}，{age_text}，视觉超时阈值={node.vision_timeout:.3f}s"
        )

    def _execution(self):
        interface = self.node.ros_interface
        if interface.chase_goal_pending:
            if interface.chase_cancel_pending:
                return "chase_cancel_pending", "等待旧追击接受后取消，尚未执行新的行为树目标"
            return "chase_pending", "等待追击Action接受请求"
        if interface.chase_goal_handle is not None:
            if interface.chase_cancel_pending:
                return "chase_canceling", "旧追击取消中，尚未执行新的行为树目标"
            return "chase_active", "追击Action仍在执行"
        owner = interface.navigation_owner
        name = OWNER_NAMES.get(owner, str(owner))
        if interface.navigation_cancel_pending:
            return f"canceling_{owner}", f"等待行为树旧{name}目标取消完成"
        if owner is not None:
            return f"running_{owner}", f"行为树{name}目标（非追击目标；导航归属仍保留）"
        return "idle", "尚无普通导航目标，等待启动或恢复重试"

    def _reason(self, task, mode, now, execution_code):
        node = self.node
        if task == NavigationTask.CHASE:
            if execution_code.startswith("chase_cancel"):
                return "cancel", "旧追击尚未取消结束，等待释放后才能重新启动", ""
            if execution_code == "chase_pending":
                return "pending", "追击请求已发出，等待服务端接受", ""
            if node.ros_interface.navigation_owner is not None:
                return "old_navigation", "已选择追击，等待行为树旧导航释放", ""
            if not node.ros_interface.chase_action_client.server_is_ready():
                return "server", "追击Action服务尚未就绪", ""
            if self.recovery is not None and self.recovery.recovering:
                return "recovery", "追击失败后正在等待恢复重试", ""
            return "not_started", "已选择追击，但尚未进入追击Action执行", ""
        if task == NavigationTask.RESUPPLY:
            return "resupply", "回血分支优先", f"当前血量={node.get_hp()}，恢复阈值={node.hp_up}"
        if task == NavigationTask.STOP and node.get_match_progress() != 4:
            return "match", "比赛未处于进行阶段", f"阶段值={node.get_match_progress()}"
        if mode == TacticalMode.DEFENSE:
            return "defense", "当前为防守模式，该模式不选择追击", ""

        condition_type = (
            NeedEnterCenterZone
            if mode == TacticalMode.NEUTRAL and task == NavigationTask.OCCUPY_CENTER
            else CanChaseInsideCenterZone if mode == TacticalMode.NEUTRAL
            else CanChase
        )
        ready = self.conditions.get((mode, IsChaseReady))
        # 中立模式先判断回点；该分支不能使用上一轮的冷却失败状态。
        if condition_type != NeedEnterCenterZone and ready is not None:
            if ready.status == py_trees.common.Status.FAILURE:
                remaining = max(0.0, node.blackboard.chase_retry_after - now)
                return "cooldown", "追击失败后的冷却尚未结束", f"冷却剩余={remaining:.3f}s"

        condition = self.conditions.get((mode, condition_type))
        if condition is None or not condition.feedback_message:
            return "unknown", "本轮未选择追击，追击条件未提供详细原因", ""
        details = []
        position = getattr(condition, "checked_robot_position", None)
        if position is not None:
            details.append(f"判定时自身位置map=({position[0]:.3f},{position[1]:.3f})")
        zone = node.get_control_zone()
        details.append(
            f"中央区启用={zone['enabled']}，"
            f"中央区x=[{zone['x_min']:.3f},{zone['x_max']:.3f}]、"
            f"y=[{zone['y_min']:.3f},{zone['y_max']:.3f}]"
        )
        if condition_type == CanChase:
            boundary = node.get_chase_boundary()
            details.append(
                f"追击边界启用={boundary['enabled']}，"
                f"追击范围x=({boundary['x_min']:.3f},{boundary['x_max']:.3f})、"
                f"y=({boundary['y_min']:.3f},{boundary['y_max']:.3f})（碰到边界也停止）"
            )
        return condition.feedback_message, condition.feedback_message, "，".join(details)

    def log_if_needed(self):
        node = self.node
        task = node.get_navigation_task()
        mode = node.get_tactical_mode()
        execution_code, execution = self._execution()
        if task == NavigationTask.CHASE and execution_code == "chase_active":
            # 不根据“没有新计算点”判断退出追击。
            self.last_key = None
            return

        now = time.monotonic()
        code, reason, details = self._reason(task, mode, now, execution_code)
        vision_code, vision = self._vision()
        key = (task, mode, execution_code, code, vision_code)
        if key == self.last_key and now - self.last_print_at < 1.0:
            return
        message = (
            f"[未追击原因] 行为树选择={TASK_NAMES.get(task, str(task))}，"
            f"当前执行={execution}，原因={reason}，"
            f"模式={MODE_NAMES.get(mode, str(mode))}，{vision}"
        )
        if details:
            message += f"，{details}"
        node.get_logger().info(message)
        self.last_key = key
        self.last_print_at = now
