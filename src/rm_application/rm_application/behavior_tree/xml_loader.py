from py_trees.parsers.behaviour_tree_xml import (
    parse_behaviour_tree_xml,
)

from rm_application.behavior_tree.registry import (
    create_node_registry,
)


# 使用公共库加载 XML，创建并返回行为树
def load_behavior_tree(node, xml_path):
    # 根据当前节点提供的数据和接口，准备节点注册表
    registry = create_node_registry(node)

    # 读取 XML，按照注册表创建节点并连接成树
    return parse_behaviour_tree_xml(
        str(xml_path),
        node_registry=registry,
    )