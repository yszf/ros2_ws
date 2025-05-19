#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// GStreamer库
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

class OrbbecRTSPServer : public rclcpp::Node
{
public:
    explicit OrbbecRTSPServer(const rclcpp::NodeOptions &options)
        : Node("orbbec_rtsp_server", options)
    {
        // 初始化GStreamer
        gst_init(nullptr, nullptr);
        
        // 声明并获取参数
        declare_parameters();
        get_parameters();
        
        // 创建GStreamer推流管道
        create_gst_pipeline();
        
        // 订阅相机图像话题
        image_sub_ = image_transport::create_subscription(
            this,
            image_topic_,
            std::bind(&OrbbecRTSPServer::image_callback, this, std::placeholders::_1),
            "raw",
            rmw_qos_profile_sensor_data);
            
        RCLCPP_INFO(this->get_logger(), 
            "Orbbec RTSP Server started. Stream available at rtsp://%s:%s", 
            rtsp_host_.c_str(), rtsp_port_.c_str());
    }

    ~OrbbecRTSPServer()
    {
        // 清理GStreamer资源
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
    }

private:
    image_transport::Subscriber image_sub_;
    GstElement *pipeline_ = nullptr;
    GstElement *appsrc_ = nullptr;
    
    // 配置参数
    std::string image_topic_;
    int width_;
    int height_;
    int framerate_;
    std::string rtsp_host_;
    std::string rtsp_port_;
    std::string encoding_;
    int bitrate_;

    void declare_parameters()
    {
        declare_parameter("image_topic", "/camera/color/image_raw");
        declare_parameter("width", 640);
        declare_parameter("height", 480);
        declare_parameter("framerate", 30);
        declare_parameter("rtsp_host", "127.0.0.1");
        declare_parameter("rtsp_port", "8554");
        declare_parameter("encoding", "H264");
        declare_parameter("bitrate", 1000);  // kbps
    }

    void get_parameters()
    {
        image_topic_ = get_parameter("image_topic").as_string();
        width_ = get_parameter("width").as_int();
        height_ = get_parameter("height").as_int();
        framerate_ = get_parameter("framerate").as_int();
        rtsp_host_ = get_parameter("rtsp_host").as_string();
        rtsp_port_ = get_parameter("rtsp_port").as_string();
        encoding_ = get_parameter("encoding").as_string();
        bitrate_ = get_parameter("bitrate").as_int();
    }

    void create_gst_pipeline()
    {
        // 根据编码类型选择GStreamer元素
        std::string encoder, payloader;
        if (encoding_ == "H264") {
            encoder = "x264enc tune=zerolatency speed-preset=ultrafast bitrate=" + std::to_string(bitrate_);
            payloader = "rtph264pay config-interval=1 pt=96";
        } else if (encoding_ == "H265") {
            encoder = "x265enc tune=zerolatency bitrate=" + std::to_string(bitrate_);
            payloader = "rtph265pay config-interval=1 pt=96";
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unsupported encoding: %s", encoding_.c_str());
            return;
        }

        // 构建GStreamer管道字符串
        std::string pipeline_str = 
            "appsrc name=source is-live=true format=GST_FORMAT_TIME "
            "caps=video/x-raw,format=BGR,width=" + std::to_string(width_) + 
            ",height=" + std::to_string(height_) + 
            ",framerate=" + std::to_string(framerate_) + "/1 "
            "! videoconvert "
            "! " + encoder + " "
            "! " + payloader + " "
            "! udpsink host=" + rtsp_host_ + " port=" + rtsp_port_;

        // 创建GStreamer管道
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), nullptr);
        if (!pipeline_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create GStreamer pipeline!");
            return;
        }

        // 获取appsrc元素
        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "source");
        if (!appsrc_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get appsrc element!");
            return;
        }

        // 设置appsrc属性
        g_object_set(G_OBJECT(appsrc_), 
            "stream-type", 0, 
            "format", GST_FORMAT_TIME, 
            nullptr);
            
        // 启动管道
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        try {
            // 转换ROS图像消息为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
                msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;
            
            // 确保图像尺寸匹配
            if (image.cols != width_ || image.rows != height_) {
                cv::resize(image, image, cv::Size(width_, height_));
            }
            
            // 推送图像到GStreamer管道
            push_frame_to_gst(image);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void push_frame_to_gst(const cv::Mat &image)
    {
        if (!appsrc_) return;
        
        // 创建GStreamer缓冲区
        GstBuffer *buffer = gst_buffer_new_allocate(
            nullptr, image.total() * image.elemSize(), nullptr);
            
        // 复制图像数据到缓冲区
        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_WRITE);
        memcpy(map.data, image.data, image.total() * image.elemSize());
        gst_buffer_unmap(buffer, &map);
        
        // 设置时间戳
        static GstClockTime timestamp = 0;
        GST_BUFFER_PTS(buffer) = timestamp;
        GST_BUFFER_DTS(buffer) = timestamp;
        GST_BUFFER_DURATION(buffer) = 
            gst_util_uint64_scale_int(1, GST_SECOND, framerate_);
        timestamp += GST_BUFFER_DURATION(buffer);
        
        // 推送缓冲区到appsrc
        GstFlowReturn ret;
        g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
        
        // 释放资源
        gst_buffer_unref(buffer);
        
        if (ret != GST_FLOW_OK) {
            RCLCPP_WARN(this->get_logger(), "Failed to push buffer to pipeline");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OrbbecRTSPServer>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}