#!/bin/bash

echo "开始全面清理残留进程和相关终端窗口..."

# 1. 优先温柔地结束正在运行的 ROS 2 launch 进程
pkill -INT -f "ros2 launch rm_application"
sleep 1

# 2. 暴力猎杀可能卡死的底层 C++ 可执行文件（精确匹配进程名）
# 这一步是为了防止它们僵死并在后台占用 USB/串口或网络端口
pkill -9 -x "rm_decision"
pkill -9 -x "serial_node"
pkill -9 -x "livox_ros_driver2_node"
pkill -9 -x "pointlio_mapping"
pkill -9 -x "rviz2"

# 3. 杀掉刚才启动的那 3 个独立终端窗口背后的 bash 进程
# 匹配我们刚才下发的具体指令，连同窗口一起关掉
pkill -9 -f "ros2 launch rm_application open_all.launch.py"
pkill -9 -f "ros2 launch rm_application serial_node.launch.py"
pkill -9 -f "ros2 ros2 launch rm_application decision.launch.py"

# 4. 重启 ROS 2 的 DDS 发现守护进程（清除系统里的“幽灵节点”缓存）
ros2 daemon stop
ros2 daemon start

echo "清理完毕，环境已彻底重置。"