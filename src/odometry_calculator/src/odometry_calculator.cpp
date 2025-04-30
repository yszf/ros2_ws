#include <fstream>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

// 计算欧几里得距离
double euclidean_distance_2d(const nav_msgs::msg::Odometry& odom1, const nav_msgs::msg::Odometry& odom2) {
    double dx = odom2.pose.pose.position.x - odom1.pose.pose.position.x;
    double dy = odom2.pose.pose.position.y - odom1.pose.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

double euclidean_distance_3d(const nav_msgs::msg::Odometry& odom1, const nav_msgs::msg::Odometry& odom2) {
    double dx = odom2.pose.pose.position.x - odom1.pose.pose.position.x;
    double dy = odom2.pose.pose.position.y - odom1.pose.pose.position.y;
    double dz = odom2.pose.pose.position.z - odom1.pose.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 获取当前日期和时间字符串
std::string get_current_datetime() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S")
       << '.' << std::setfill('0') << std::setw(3) << now_ms.count();
    return ss.str();
}

class OdometerNode : public rclcpp::Node
{
public:
    OdometerNode() : Node("odometer_node")
    {
        // 读取之前的总里程
        read_previous_distance();

        // 订阅 /rtk_gps_node/odom 话题
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rtk_gps_node/odom", 10, std::bind(&OdometerNode::odomCallback, this, std::placeholders::_1));

        // 发布 /APU/Nav/State/Odometer 话题
        odometer_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/APU/Nav/State/Odometer", 10);

        // 创建定时器，每隔 10 秒调用一次 timerCallback
        timer_ = this->create_wall_timer(
            std::chrono::seconds(10), std::bind(&OdometerNode::timerCallback, this));
    }

private:
    // 类成员变量
    double total_distance_2d = 0.0;
    double current_distance_2d = 0.0;
    double total_distance_3d = 0.0;
    double current_distance_3d = 0.0;
    nav_msgs::msg::Odometry previous_odom;
    bool first_odom_received = false;

    // 读取之前的总里程
    void read_previous_distance() {
        std::ifstream file("total_distance.txt");
        if (file.is_open()) {
            file >> total_distance_2d >> total_distance_3d;
            file.close();
        } else {
            RCLCPP_INFO(this->get_logger(), "[%s] total_distance.txt file not found.", get_current_datetime().c_str());
        }
    }

    // 保存里程信息到文件
    void save_distance() {
        std::ofstream file("total_distance.txt");
        if (file.is_open()) {
            file << total_distance_2d << " " << total_distance_3d << " " << current_distance_2d << " " << current_distance_3d;
            file.close();
            RCLCPP_INFO(this->get_logger(), "[%s] Distance saved: Total 2D: %.2f m, Total 3D: %.2f m, Current 2D: %.2f m, Current 3D: %.2f m",
                        get_current_datetime().c_str(), total_distance_2d, total_distance_3d, current_distance_2d, current_distance_3d);
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 打印接收到数据的日志
        RCLCPP_INFO(this->get_logger(), "[%s] Received odometry data. Position: (%.2f, %.2f, %.2f)", 
                    get_current_datetime().c_str(), msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        if (first_odom_received) {
            double d_2d = euclidean_distance_2d(previous_odom, *msg);
            double d_3d = euclidean_distance_3d(previous_odom, *msg);
            current_distance_2d += d_2d;
            total_distance_2d += d_2d;
            current_distance_3d += d_3d;
            total_distance_3d += d_3d;
        } else {
            first_odom_received = true;
        }
        previous_odom = *msg;

        // 发布里程计信息
        auto odometer_msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
        odometer_msg->data.resize(4);
        odometer_msg->data[0] = total_distance_2d;
        odometer_msg->data[1] = current_distance_2d;
        odometer_msg->data[2] = total_distance_3d;
        odometer_msg->data[3] = current_distance_3d;
        odometer_pub_->publish(std::move(odometer_msg));
    }

    void timerCallback()
    {
        save_distance();
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr odometer_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}    