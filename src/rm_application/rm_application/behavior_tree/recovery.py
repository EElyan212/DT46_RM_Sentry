import py_trees
from py_trees.ports import PortsMixin

# 公共行为树库使用的恢复控制节点
class RecoveryNode(PortsMixin, py_trees.composites.Composite):
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(
        self,
        name,
        children,
        max_retries=2,
        on_exhausted=None,
        read_progress=None,
        **kwargs,
    ):
        # 第一个子节点是正常动作，第二个子节点是恢复步骤
        if len(children) != 2:
            raise ValueError("恢复控制必须有正常动作和恢复步骤两个子节点")

        super().__init__(name=name, children=children, **kwargs)

        self.action = children[0]
        self.recovery_action = children[1]

        # 每轮允许的额外重试次数
        self.max_retries = int(max_retries)
        if self.max_retries < 0:
            raise ValueError("恢复重试次数不能小于 0")

        # 耗尽回调，以及读取正常动作到达计数的方法
        self.on_exhausted = on_exhausted
        self.read_progress = read_progress
        self.progress = self._read_progress()

        self.retry_count = 0
        self.recovering = False
        self.restart_round = False

    def _read_progress(self):
        if self.read_progress is None:
            return None

        return self.read_progress(self.action)

    def _report_exhausted(self):
        self.restart_round = True

        if self.on_exhausted is not None:
            self.on_exhausted()

    def tick(self):
        status = py_trees.common.Status

        if self.status != status.RUNNING:
            self.initialise()

        # 失败后先完成恢复步骤，再重新执行正常动作
        if self.recovering:
            self.current_child = self.recovery_action
            yield from self.recovery_action.tick()
            recovery_status = self.recovery_action.status

            if recovery_status == status.RUNNING:
                self.status = status.RUNNING
                yield self
                return

            self.recovery_action.stop(status.INVALID)

            # 恢复步骤失败时，本轮也向上报告失败
            if recovery_status == status.FAILURE:
                self._report_exhausted()
                self.stop(status.FAILURE)
                yield self
                return

            self.recovering = False

            if self.restart_round:
                self.retry_count = 0
                self.restart_round = False
            else:
                self.retry_count += 1

        self.current_child = self.action
        yield from self.action.tick()
        action_status = self.action.status

        # 到达目标后，即使动作仍在运行，也清零连续失败次数
        if self.read_progress is not None:
            progress = self._read_progress()

            if progress != self.progress:
                self.retry_count = 0
                self.progress = progress

        if action_status == status.SUCCESS:
            self.retry_count = 0
            self.stop(status.SUCCESS)

        elif action_status == status.RUNNING:
            self.status = status.RUNNING

        else:
            # 正常动作失败，停止动作并准备恢复
            self.action.stop(status.INVALID)
            self.recovery_action.stop(status.INVALID)
            self.recovering = True

            if self.retry_count >= self.max_retries:
                self._report_exhausted()
                self.stop(status.FAILURE)
            else:
                self.status = status.RUNNING

        yield self

    def reset(self):
        # 明确复位时使用；普通中断不清空恢复阶段和次数
        self.stop(py_trees.common.Status.INVALID)
        self.retry_count = 0
        self.recovering = False
        self.restart_round = False
        self.progress = self._read_progress()