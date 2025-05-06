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
        // 打开日志文件
        log_file_.open("odometry_log.txt", std::ios::app);
        if (!log_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open log file.");
        }

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

    ~OdometerNode()
    {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

private:
    // 类成员变量
    double total_distance_2d = 0.0;
    double current_distance_2d = 0.0;
    double total_distance_3d = 0.0;
    double current_distance_3d = 0.0;
    nav_msgs::msg::Odometry previous_odom;
    bool first_odom_received = false;
    std::ofstream log_file_;

    // 将日志信息写入文件的方法
    void write_log_to_file(const std::string& log_msg) {
        if (log_file_.is_open()) {
            log_file_ << log_msg << std::endl;
        }
    }

    // 读取之前的总里程
    void read_previous_distance() {
        std::ifstream file("total_distance.txt");
        if (file.is_open()) {
            file >> total_distance_2d >> total_distance_3d;
            file.close();
        } else {
            std::string log_msg = "[" + get_current_datetime() + "] total_distance.txt file not found.";
            RCLCPP_INFO(this->get_logger(), log_msg.c_str());
            write_log_to_file(log_msg);
        }
    }

    // 保存里程信息到文件
    void save_distance() {
        std::ofstream file("total_distance.txt");
        if (file.is_open()) {
            file << total_distance_2d << " " << total_distance_3d << " " << current_distance_2d << " " << current_distance_3d;
            file.close();
            std::string log_msg = "[" + get_current_datetime() + "] Distance saved: Total 2D: " + 
                std::to_string(total_distance_2d) + " m, Total 3D: " + std::to_string(total_distance_3d) + 
                " m, Current 2D: " + std::to_string(current_distance_2d) + " m, Current 3D: " + std::to_string(current_distance_3d) + " m";
            RCLCPP_INFO(this->get_logger(), log_msg.c_str());
            write_log_to_file(log_msg);
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 打印接收到数据的日志
        std::string log_msg = "[" + get_current_datetime() + "] Received odometry data. Position: (" + 
            std::to_string(msg->pose.pose.position.x) + ", " + std::to_string(msg->pose.pose.position.y) + ", " + 
            std::to_string(msg->pose.pose.position.z) + ")";
        RCLCPP_INFO(this->get_logger(), log_msg.c_str());
        write_log_to_file(log_msg);

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