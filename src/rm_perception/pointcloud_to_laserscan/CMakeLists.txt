cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_laserscan)

# 1. 查找基础构建工具
find_package(ament_cmake_auto REQUIRED)

# 2. 显式查找所有必要的 ROS 2 依赖包
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)   # 新增
find_package(laser_geometry REQUIRED)    # 新增
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# 3. 显式查找 PCL
find_package(PCL REQUIRED COMPONENTS common filters)

# 4. 自动同步 package.xml 依赖
ament_auto_find_build_dependencies()

# --- 5. LaserScan To PointCloud 库 ---
add_library(laserscan_to_pointcloud SHARED
  src/laserscan_to_pointcloud_node.cpp
)

ament_target_dependencies(laserscan_to_pointcloud
  rclcpp
  sensor_msgs
  message_filters
  tf2_ros
  laser_geometry   # 解决 laser_geometry.hpp 缺失
  pcl_conversions
  pcl_ros
)

target_include_directories(laserscan_to_pointcloud PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(laserscan_to_pointcloud ${PCL_LIBRARIES})

rclcpp_components_register_node(laserscan_to_pointcloud
  PLUGIN "pointcloud_to_laserscan::LaserScanToPointCloudNode"
  EXECUTABLE laserscan_to_pointcloud_node
)

# --- 6. PointCloud To LaserScan 库 ---
add_library(pointcloud_to_laserscan SHARED
  src/pointcloud_to_laserscan_node.cpp
)

ament_target_dependencies(pointcloud_to_laserscan
  rclcpp
  sensor_msgs
  message_filters
  tf2_ros
  tf2_sensor_msgs  # 解决 tf2_sensor_msgs.hpp 缺失
  pcl_conversions
  pcl_ros
)

target_include_directories(pointcloud_to_laserscan PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(pointcloud_to_laserscan ${PCL_LIBRARIES})

rclcpp_components_register_node(pointcloud_to_laserscan
  PLUGIN "pointcloud_to_laserscan::PointCloudToLaserScanNode"
  EXECUTABLE pointcloud_to_laserscan_node
)

# --- 7. Dummy PointCloud Publisher ---
add_executable(dummy_pointcloud_publisher
  src/dummy_pointcloud_publisher.cpp
)
ament_target_dependencies(dummy_pointcloud_publisher rclcpp sensor_msgs)
target_include_directories(dummy_pointcloud_publisher PRIVATE 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# --- 8. 打包 ---
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE launch 
)