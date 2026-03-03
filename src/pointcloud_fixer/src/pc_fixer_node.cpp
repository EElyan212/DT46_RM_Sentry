#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstring>
#include <vector>

class PointCloudFixer : public rclcpp::Node {
public:
    PointCloudFixer() : Node("pc_fixer_node") {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10, std::bind(&PointCloudFixer::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_fixed", 10);
        RCLCPP_INFO(this->get_logger(), "Point-LIO Strict Fixer Started (xfer_format=0 mode).");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sensor_msgs::msg::PointCloud2 fixed_msg;
        fixed_msg.header = msg->header;
        fixed_msg.height = msg->height;
        fixed_msg.width = msg->width;
        fixed_msg.is_bigendian = msg->is_bigendian;
        fixed_msg.is_dense = msg->is_dense;

        // --- 强制构造 Livox PointXYZRTL 格式字段 ---
        fixed_msg.fields.clear();
        uint32_t offset = 0;

        auto add_field = [&](std::string name, uint32_t off, uint8_t type, uint32_t count) {
            sensor_msgs::msg::PointField f;
            f.name = name; f.offset = off; f.datatype = type; f.count = count;
            fixed_msg.fields.push_back(f);
        };

        add_field("x", offset, 7, 1); offset += 4; // FLOAT32
        add_field("y", offset, 7, 1); offset += 4;
        add_field("z", offset, 7, 1); offset += 4;
        add_field("intensity", offset, 7, 1); offset += 4;
        add_field("ring", offset, 4, 1); offset += 2;;  // UINT16(Point-LIO 里的 ring)
        add_field("time", offset, 7, 1); offset += 4;;  // FLOAT32

        fixed_msg.point_step = offset; // 应该是 26 字节
        fixed_msg.row_step = fixed_msg.point_step * fixed_msg.width;
        fixed_msg.data.resize(fixed_msg.row_step * fixed_msg.height);

        // --- 按照新结构搬运数据 ---
        const uint8_t* src_ptr = msg->data.data();
        uint8_t* dst_ptr = fixed_msg.data.data();

        for (size_t i = 0; i < fixed_msg.width * fixed_msg.height; ++i) {
            // 1. 拷贝 X, Y, Z, Intensity (假设原驱动输出也是这四个 float32 开头)
            // 此时原驱动 point_step 通常是 16 或更高
            std::memcpy(dst_ptr, src_ptr, 16); 

            // 2. 填充 tag 和 line (ring)
            dst_ptr[16] = 0; // tag
            dst_ptr[17] = 0; // line (ring)

            // 3. 填充 time (float64)
            double t = 0.0;
            std::memcpy(dst_ptr + 18, &t, 8);

            src_ptr += msg->point_step;
            dst_ptr += fixed_msg.point_step;
        }

        pub_->publish(fixed_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFixer>());
    rclcpp::shutdown();
    return 0;
}