#!/bin/bash

echo "正在打开独立终端运行 ROS 2 节点..."

# 获取当前运行脚本的绝对路径（假设你在工作空间根目录运行此脚本）
WS_DIR=$(pwd)

# 1. 在新终端启动 open_all
gnome-terminal --title="open_all" -- bash -c "cd $WS_DIR && source install/setup.bash && ros2 launch rm_application open_all.launch.py; exec bash"

sleep 1

# 2. 在新终端启动 serial_node
gnome-terminal --title="serial_node" -- bash -c "cd $WS_DIR && source install/setup.bash && ros2 launch rm_application serial_node.launch.py; exec bash"

sleep 1

# 3. 在新终端启动 decision
gnome-terminal --title="decision" -- bash -c "cd $WS_DIR && source install/setup.bash &&  ros2 run rm_application rm_decision --ros-args --params-file src/rm_application/config/decision_params.yaml; exec bash"

echo "3个终端已打开。如需关闭节点，请在各自的窗口中按 Ctrl+C。"