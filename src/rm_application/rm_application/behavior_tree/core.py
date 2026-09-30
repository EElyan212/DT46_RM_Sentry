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