#!/bin/bash
set -e  # 出现任何错误立即退出

git subtree pull --prefix=src/rm_driver/livox_ros_driver2 livox_ros_driver2 master
