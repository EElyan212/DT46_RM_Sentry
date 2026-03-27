cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_laserscan)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

# 自动同步依赖
ament_auto_find_build_dependencies()

# --- 直接编译可执行文件 ---
# 不再创建 library，直接把源文件塞进 executable
add_executable(pointcloud_to_laserscan_node src/pointcloud_to_laserscan_node.cpp)

ament_target_dependencies(pointcloud_to_laserscan_node
  rclcpp 
  sensor_msgs 
  message_filters 
  tf2_ros 
  tf2_sensor_msgs 
  pcl_conversions 
  pcl_ros
)

target_include_directories(pointcloud_to_laserscan_node PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(pointcloud_to_laserscan_node ${PCL_LIBRARIES})

# --- 安装配置 ---
install(TARGETS pointcloud_to_laserscan_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 和 config (如果有的话)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_auto_package()