#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"  // 引用正确的服务头文件

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class ClearCostmapClient : public rclcpp::Node
{
public:
  ClearCostmapClient()
  : Node("clear_costmap_client"), is_request_in_progress_(false)
  {
    // 创建服务客户端，注意使用正确的服务类型
    client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("local_costmap/clear_entirely_local_costmap");
    client_1 = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("global_costmap/clear_entirely_global_costmap");

    // 等待服务可用
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    // 创建定时器，设置每1秒调用一次清除代价地图的服务
    timer_ = this->create_wall_timer(
      1s, std::bind(&ClearCostmapClient::send_clear_request, this));
  }

private:
  // 发送清除代价地图请求的回调函数
  void send_clear_request()
  {
    // RCLCPP_INFO(this->get_logger(), "0\n");
    // 如果当前请求仍在处理中，跳过新的请求
    if (is_request_in_progress_) {
      return;
    }
    // RCLCPP_INFO(this->get_logger(), "1\n");
    // 标记请求为正在进行中
    is_request_in_progress_ = true;
    // RCLCPP_INFO(this->get_logger(), "2\n");
    // 构建请求
    auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    // RCLCPP_INFO(this->get_logger(), "3\n");
    // 异步发送请求并等待结果
    auto result = client_->async_send_request(request);
    auto result_1 = client_1->async_send_request(request);
    // RCLCPP_INFO(this->get_logger(), "4\n");
    // 等待结果完成
    //result.wait();
    // RCLCPP_INFO(this->get_logger(), "5\n");
    // 没有 `status` 字段，只需检查请求是否完成
    // if (result.get()) {
    //   RCLCPP_INFO(this->get_logger(), "Successfully cleared the local costmap.");
    // } else {
    //   RCLCPP_ERROR(this->get_logger(), "Failed to call service clear_entirely_local_costmap.");
    // }
    // RCLCPP_INFO(this->get_logger(), "6\n");
    // 请求完成，标记请求为已处理
    is_request_in_progress_ = false;
  }

  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client_1;
  rclcpp::TimerBase::SharedPtr timer_;

  bool is_request_in_progress_;  // 标记请求是否正在进行中
};

int main(int argc, char **argv)
{
  // 初始化 ROS2 节点
  rclcpp::init(argc, argv);

  // 创建 ClearCostmapClient 节点实例
  auto node = std::make_shared<ClearCostmapClient>();

  // 运行节点，等待定时器触发，只有一次调用 spin
  rclcpp::spin(node);  // 确保只调用一次 spin

  // 关闭 ROS2
  // rclcpp::shutdown();
  return 0;
}
