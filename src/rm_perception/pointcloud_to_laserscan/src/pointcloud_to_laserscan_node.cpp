#include "pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// 替换头文件：由 radius_outlier_removal 更改为 statistical_outlier_removal
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

namespace pointcloud_to_laserscan
{

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  // 1. 将 ROS 消息转换为 PCL 点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

  // 2. 预处理：高度滤波 (减少后续统计滤波的计算量)
  if (pcl_cloud->size() > 0) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcl_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_height_, max_height_);
    pass.filter(*pcl_cloud);
  }

  // 3. 执行统计滤波 (Statistical Outlier Removal)
  if (pcl_cloud->size() > 0) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(pcl_cloud);
    
    // 设置用于平均距离估计的邻临近点数目（建议 30-50）
    sor.setMeanK(50); 
    
    // 设置标准差倍数阈值（建议 1.0），值越小滤除的点越多
    sor.setStddevMulThresh(1.0); 
    
    sor.filter(*pcl_cloud);
  }

  // 4. 构建 LaserScan 输出
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;
  scan_msg->header.stamp = this->now();
  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  uint32_t ranges_size = std::ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
  
  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  // 5. 遍历滤波后的 PCL 点云并进行投影
  for (const auto& point : pcl_cloud->points)
  {
    double range = std::hypot(point.x, point.y);
    if (range < range_min_ || range > range_max_) {
      continue;
    }

    double angle = std::atan2(point.y, point.x);
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      continue;
    }

    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
    if (index >= 0 && index < static_cast<int>(ranges_size)) {
      if (range < scan_msg->ranges[index]) {
        scan_msg->ranges[index] = range;
      }
    }
  }

  pub_->publish(std::move(scan_msg));
}

} // namespace pointcloud_to_laserscan