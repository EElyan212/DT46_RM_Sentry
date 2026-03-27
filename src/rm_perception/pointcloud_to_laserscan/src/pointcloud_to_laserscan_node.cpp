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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

namespace pointcloud_to_laserscan
{

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: Node("pointcloud_to_laserscan", options)
{
  // [重要] 在构造函数中声明并获取参数，否则变量（如 min_height_）是随机值
  target_frame_ = this->declare_parameter("target_frame", "");
  min_height_ = this->declare_parameter("min_height", -1.0);
  max_height_ = this->declare_parameter("max_height", 1.0);
  angle_min_ = this->declare_parameter("angle_min", -3.14159);
  angle_max_ = this->declare_parameter("angle_max", 3.14159);
  angle_increment_ = this->declare_parameter("angle_increment", 0.0043);
  scan_time_ = this->declare_parameter("scan_time", 0.1);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", 30.0);
  use_inf_ = this->declare_parameter("use_inf", true);
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);

  // 初始化发布者
  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

  // 初始化订阅者（这里建议根据你的话题名修改）
  using std::placeholders::_1;
  sub_.subscribe(this, "cloud_in", rmw_qos_profile_sensor_data);

  // 节点启动成功的打印
  RCLCPP_INFO(this->get_logger(), "====================================================");
  RCLCPP_INFO(this->get_logger(), "PointCloud to LaserScan Node successfully started!");
  RCLCPP_INFO(this->get_logger(), "Target Frame: %s", target_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "Height Range: [%.2f, %.2f]", min_height_, max_height_);
  RCLCPP_INFO(this->get_logger(), "====================================================");
}

// [修复关键 1] 必须实现析构函数，否则会报 vtable 错误
PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
{
  if (subscription_listener_thread_.joinable()) {
    alive_ = false;
    subscription_listener_thread_.join();
  }
}

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
    "Data received! Points count: %d, Frame: %s", cloud_msg->width * cloud_msg->height, cloud_msg->header.frame_id.c_str());

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

  if (pcl_cloud->size() > 0) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcl_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_height_, max_height_);
    pass.filter(*pcl_cloud);
  }

  if (pcl_cloud->size() > 0) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(pcl_cloud);
    sor.setMeanK(50); 
    sor.setStddevMulThresh(1.0); 
    sor.filter(*pcl_cloud);
  }

  if (pcl_cloud->empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
      "No points left after height/statistical filtering!");
    return;
  }

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
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  for (const auto& point : pcl_cloud->points)
  {
    float range = std::hypot(point.x, point.y);
    if (range < range_min_ || range > range_max_) continue;

    float angle = std::atan2(point.y, point.x);
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) continue;

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

// [修复关键 2] main 函数完全移出命名空间
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // 使用之前 CMakeLists.txt 中定义的独立节点启动方式
  auto node = std::make_shared<pointcloud_to_laserscan::PointCloudToLaserScanNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}