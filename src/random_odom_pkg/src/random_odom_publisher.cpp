#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcutils/logging.h>
#include <random>

class RandomOdomPublisher : public rclcpp::Node
{
public:
    RandomOdomPublisher() : Node("random_odom_publisher")
    {
        // 创建里程计消息发布者
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/rtk_gps_node/odom", 10);
        if (!publisher_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create publisher");
        }
        // 创建定时器，每秒触发一次回调函数
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&RandomOdomPublisher::timer_callback, this));
        if (!timer_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create timer");
        }

        RCLCPP_INFO(this->get_logger(), "Random Odom Publisher Node Initialized");
        // 初始化随机数生成器
        std::random_device rd;
        gen_ = std::mt19937(rd());
        pos_dist_ = std::uniform_real_distribution<double>(-10.0, 10.0);
        orient_dist_ = std::uniform_real_distribution<double>(-1.0, 1.0);
    }

private:
    void timer_callback()
    {
        // 创建里程计消息对象
        auto msg = std::make_unique<nav_msgs::msg::Odometry>();

        // 随机生成位置和姿态信息
        double pos_x = pos_dist_(gen_);
        double pos_y = pos_dist_(gen_);
        double pos_z = pos_dist_(gen_);

        double orient_x = orient_dist_(gen_);
        double orient_y = orient_dist_(gen_);
        double orient_z = orient_dist_(gen_);
        double orient_w = orient_dist_(gen_);

        // 归一化四元数
        double norm = std::sqrt(orient_x * orient_x + orient_y * orient_y + orient_z * orient_z + orient_w * orient_w);
        orient_x /= norm;
        orient_y /= norm;
        orient_z /= norm;
        orient_w /= norm;

        // 设置固定的坐标值
        msg->pose.pose.position.x = pos_x;
        msg->pose.pose.position.y = pos_y;
        msg->pose.pose.position.z = pos_z;

        // 设置默认的四元数表示姿态
        msg->pose.pose.orientation.x = orient_x;
        msg->pose.pose.orientation.y = orient_y;
        msg->pose.pose.orientation.z = orient_z;
        msg->pose.pose.orientation.w = orient_w;

        // 设置消息的时间戳和坐标系信息
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "odom";
        msg->child_frame_id = "base_link";

        // 协方差矩阵初始化为零
        for (size_t i = 0; i < msg->pose.covariance.size(); ++i)
        {
            msg->pose.covariance[i] = 0.0;
        }
        for (size_t i = 0; i < msg->twist.covariance.size(); ++i)
        {
            msg->twist.covariance[i] = 0.0;
        }

        // 发布里程计消息
        if (publisher_) {
            RCLCPP_INFO(this->get_logger(), "About to publish the odometry message");
            publisher_->publish(std::move(msg));

            auto logger = this->get_logger();
            const char* logger_name = logger.get_name();
            if (logger_name != nullptr && *logger_name != '\0') {
                // 获取日志器的级别
                rcl_log_severity_t logger_level = static_cast<rcl_log_severity_t>(rcutils_logging_get_logger_level(logger_name));
                RCLCPP_INFO_STREAM(logger, "Logger name: " << logger_name << ", Level: " << static_cast<int>(logger_level));

                // 输出发布的里程计信息
                RCLCPP_INFO(logger, "Published: x=%.2f, y=%.2f, z=%.2f, orient_x=%.2f, orient_y=%.2f, orient_z=%.2f, orient_w=%.2f",
                            pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w);
            } else {
                std::cerr << "Logger is invalid!" << std::endl;
            }
        }
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> pos_dist_;
    std::uniform_real_distribution<double> orient_dist_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RandomOdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}    