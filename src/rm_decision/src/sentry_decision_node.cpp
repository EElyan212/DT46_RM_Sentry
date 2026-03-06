#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/int32.hpp"

class SentryStrategyNode : public rclcpp::Node {
public:
    SentryStrategyNode() : Node("decision_node") {
        this->declare_parameter("hp_threshold", 150);
        this->declare_parameter("center_pose", std::vector<double>{0.0, 0.0, 0.0, 1.0});
        this->declare_parameter("supply_pose", std::vector<double>{0.0, 0.0, 0.0, 1.0});

        this->nav_client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");
        // 订阅血量话题
        hp_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "HP", 10, std::bind(&SentryStrategyNode::hp_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "决策节点启动");
    }

private:
    void hp_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        int hp_threshold = this->get_parameter("hp_threshold").as_int();
        
        // 当前血量低于阈值且之前不在补给状态
        if (msg->data < hp_threshold && current_strategy_ != "SUPPLY") {
            RCLCPP_INFO(this->get_logger(), "当前血量：%d，低于阈值，前往补给点", msg->data);
            current_strategy_ = "SUPPLY";
            send_goal_by_type("supply_pose");
        } 
        // 血量充足且之前不在中心点状态
        else if (msg->data >= hp_threshold && current_strategy_ != "CENTER") {
            RCLCPP_INFO(this->get_logger(), "当前血量：%d，状态良好，前往中心点", msg->data);
            current_strategy_ = "CENTER";
            send_goal_by_type("center_pose");
        }
    }

    void send_goal_by_type(const std::string & param_name) {
        if (!this->nav_client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "导航服务器未就绪！");
            return;
        }

        auto pose_vec = this->get_parameter(param_name).as_double_array();

        nav2_msgs::action::NavigateToPose::Goal goal_msg;
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = pose_vec[0];
        goal_msg.pose.pose.position.y = pose_vec[1];
        goal_msg.pose.pose.orientation.z = pose_vec[2];
        goal_msg.pose.pose.orientation.w = pose_vec[3];

        this->nav_client_ptr_->async_send_goal(goal_msg);
    }

    // 成员变量
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr hp_sub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_ptr_;
    std::string current_strategy_ = "NONE"; // 状态锁，防止重复发指令
}; 

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SentryStrategyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}