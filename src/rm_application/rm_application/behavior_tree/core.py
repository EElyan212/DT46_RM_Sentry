from enum import Enum, auto

# 节点状态
class Status(Enum):
    SUCCESS = auto()
    FAILURE = auto()
    RUNNING = auto()

# 节点基类
class Node:
    def __init__(self, name):
        self.name = name

    def tick(self):
        # 由子类实现
        raise NotImplementedError("子节点必须实现 tick()")

    def halt(self):
        # 默认不执行停止操作
        pass

# 顺序节点
class Sequence(Node):
    def __init__(self, name, children):
        super().__init__(name)

        # 按执行顺序保存子节点
        self.children = children

        # 记录当前正在运行的子节点
        self.running_child = None

    def tick(self):
        # 每次从第一个子节点开始检查
        for child in self.children:
            status = child.tick()

            # 条件或动作失败
            if status == Status.FAILURE:
                # 停止之前正在运行的动作
                if self.running_child is not None and self.running_child is not child:
                    self.running_child.halt()

                self.running_child = None
                return Status.FAILURE

            # 当前子节点仍在运行
            if status == Status.RUNNING:
                # 如果运行节点发生变化，停止旧节点
                if self.running_child is not None and self.running_child is not child:
                    self.running_child.halt()

                self.running_child = child
                return Status.RUNNING

        # 所有子节点都执行成功
        self.running_child = None
        return Status.SUCCESS

    def halt(self):
        # 停止当前正在运行的子节点
        if self.running_child is not None:
            self.running_child.halt()

        self.running_child = None

# 选择节点
class Selector(Node):
    def __init__(self, name, children):
        super().__init__(name)
        # 按优先级保存子节点
        self.children = children

        # 记录当前正在运行的子节点
        self.running_child = None

    def tick(self):
        # 每次都从最高优先级开始检查
        for child in self.children:
            status = child.tick()

            # 当前子节点已经完成
            if status == Status.SUCCESS:
                # 如果之前运行的是另一个节点，就停止它
                if self.running_child is not None and self.running_child is not child:
                    self.running_child.halt()

                self.running_child = None
                return Status.SUCCESS

            # 当前子节点还在运行
            if status == Status.RUNNING:
                # 如果选择发生变化，就停止旧节点
                if self.running_child is not None and self.running_child is not child:
                    self.running_child.halt()

                # 记住现在运行的节点
                self.running_child = child
                return Status.RUNNING

        # 所有子节点都失败，停止之前运行的节点
        if self.running_child is not None:
            self.running_child.halt()

        self.running_child = None
        return Status.FAILURE

    def halt(self):
        # 只停止当前正在运行的子节点
        if self.running_child is not None:
            self.running_child.halt()

        self.running_child = None

# 按外部指定的任务切换子节点
class Switch(Node):
    def __init__(self, name, get_key, children, can_start=None):
        super().__init__(name)

        # 读取当前任务，例如导航任务牌
        self.get_key = get_key

        # 保存“任务：执行分支”的对应关系
        self.children = children

        # 查询旧执行器是否已经释放，允许启动新分支
        self.can_start = can_start

        # 记录当前选中的分支，便于切换时先停止它
        self.active_child = None

    def tick(self):
        # 等待期间也重新读取任务牌，以最新任务为准
        key = self.get_key()
        child = self.children.get(key)

        # 任务发生变化时，先请求旧分支停止
        if child is not self.active_child:
            self.halt()

        # 只有进入新分支前才检查
        # 正在运行的分支继续正常 tick
        if self.active_child is None:
            if self.can_start is not None and not self.can_start():
                return Status.RUNNING

            self.active_child = child

        # STOP 或没有对应分支的任务
        # 确认停止后，不启动新动作
        if child is None:
            return Status.FAILURE

        return child.tick()

    def halt(self):
        if self.active_child is not None:
            self.active_child.halt()

        self.active_child = None

# 并行节点
class Parallel(Node):
    def __init__(self, name, children):
        super().__init__(name)
        # 保存所有并行子节点
        self.children = children

    def tick(self):
        statuses = []

        # 每个子节点都执行一次
        for child in self.children:
            statuses.append(child.tick())

        # 有子节点仍在运行
        if Status.RUNNING in statuses:
            return Status.RUNNING

        # 所有子节点都成功
        if all(status == Status.SUCCESS for status in statuses):
            return Status.SUCCESS

        # 没有运行中的节点，且没有全部成功
        return Status.FAILURE

    def halt(self):
        # 停止所有子节点
        for child in self.children:
            child.halt()