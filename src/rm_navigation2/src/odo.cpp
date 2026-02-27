#include <iostream>
#include <vector>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/odo_msg.hpp"
#include"geometry_msgs/msg/transform_stamped.hpp" //提供消息接口
#include"tf2/LinearMath/Quaternion.h"             //提供tf2::Quaternion类
#include"tf2_geometry_msgs/tf2_geometry_msgs.hpp" //提供消息类型转换函数
#include"tf2_ros/transform_broadcaster.h"         //提供坐标广播器类:动态广播



class OdoNode : public rclcpp::Node
{
public:
    volatile float x=0.0,y=0.0;
    volatile float radians=0.0;
    volatile double time,last_time;
    OdoNode(const char *nodeName) : Node(nodeName)
    {
        // 接收nav/control的数据
        sub_ = this->create_subscription<rm_interfaces::msg::OdoMsg>(
            "/nav/odo", 10, std::bind(&OdoNode::callback, this, std::placeholders::_1));
        tf_broadcaster_=std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
    }

    ~OdoNode()
    {
    }
    void publishTransform(float x,float y,float radians){
        geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp=this->get_clock()->now();
            transform.header.frame_id = "odom";
            transform.child_frame_id = "base_link";
            transform.transform.translation.x=x;
            transform.transform.translation.y=y;
            transform.transform.translation.z=0.0;
            tf2::Quaternion quat;
            quat.setRPY(0,0,radians);
            transform.transform.rotation = tf2::toMsg(quat);
            tf_broadcaster_->sendTransform(transform);
    }

private:
    
    void callback(const rm_interfaces::msg::OdoMsg::SharedPtr msg)
    {
        time = this->get_clock()->now().seconds();
        if(time-last_time<0.1 && msg->vx<10 && msg->vy<10 && msg->vx>-10 && msg->vy>-10 ){
            radians =msg->yaw * 3.141592653 / 180;
            vx = msg->vx*std::cos(radians)-msg->vy*std::sin(radians);
            vy = msg->vx*std::sin(radians)+msg->vy*std::cos(radians);
            x += vx * (time-last_time);
            y += vy * (time-last_time);
            publishTransform(x,y,radians);
        }
        // RCLCPP_INFO(this->get_logger(),"x:%f, y:%f\n",x,y);
        last_time = time;
    }
    

    rclcpp::Subscription<rm_interfaces::msg::OdoMsg>::SharedPtr sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    volatile float vx=0.0,vy=0.0,yaw=0.0;
    volatile long second = 0;
};

int main(int argc, char **argv)  
{
    rclcpp::init(argc, argv);
    auto odo_node = std::make_shared<OdoNode>("odo_node");
    while (rclcpp::ok())
    {
        if(odo_node->get_clock()->now().seconds() - odo_node->time > 0.5){
            odo_node->publishTransform(odo_node->x,odo_node->y,odo_node->radians);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));  //冷却一下
        }
        rclcpp::spin_some(odo_node);
    }
    rclcpp::spin(odo_node);
    

    rclcpp::shutdown();
    return 0;
}
