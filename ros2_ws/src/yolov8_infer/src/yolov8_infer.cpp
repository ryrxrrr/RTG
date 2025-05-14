#include "yolov8_infer.hpp"

// 如果要转换图像，可用 cv_bridge + OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// C++标准库
#include <iostream>

YoloV8InferNode::YoloV8InferNode(const rclcpp::NodeOptions & options)
: Node("yolov8_infer_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Initializing YoloV8InferNode...");

    // 1) 创建publisher，用于发布自定义消息(DetectionInfo)
    detection_pub_ = this->create_publisher<object_interface::msg::DetectionInfo>(
        "/detection_info", 
        10
    );

    // 2) 创建 message_filters 订阅者
    color_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, "/camera/camera/color/image_rect_raw");
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, "/camera/camera/aligned_depth_to_color/image_raw");
    info_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(
        this, "/camera/camera/color/camera_info");

    // 3) 创建同步器(近似时间同步)
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), 
        *color_sub_, 
        *depth_sub_, 
        *info_sub_);
    sync_->registerCallback(&YoloV8InferNode::syncCallback, this);

    RCLCPP_INFO(this->get_logger(), "YoloV8InferNode ready to run.");
}

void YoloV8InferNode::syncCallback(
    const sensor_msgs::msg::Image::SharedPtr & color_msg,
    const sensor_msgs::msg::Image::SharedPtr & depth_msg,
    const sensor_msgs::msg::CameraInfo::SharedPtr & info_msg)
{
    RCLCPP_INFO(this->get_logger(), "Received synchronized color + depth + info.");

    // ========== (A) 把ROS图像转为OpenCV (若需要) ==========
    cv_bridge::CvImagePtr color_cv;
    cv_bridge::CvImagePtr depth_cv;
    try {
        color_cv = cv_bridge::toCvCopy(color_msg, "bgr8");
        depth_cv = cv_bridge::toCvCopy(depth_msg, "passthrough");
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat color_frame = color_cv->image;
    cv::Mat depth_frame = depth_cv->image;

    // ========== (B) 从CameraInfo中获取内参 (fx, fy, cx, cy) 等 ==========
    const auto & K = info_msg->k; // 3x3
    float fx = K[0], fy = K[4];
    float cx = K[2], cy = K[5];

    // ========== (C) 这里本该调用 YOLOv8 做检测 + GG-CNN(或你自己的算法) ==========
    // 但 ultralytics/YOLOv8 是 Python-only，无法在C++直接调用。
    // 你可以在此放置自己C++的识别或推理逻辑，或留空做演示。

    // 假设我们识别到一个物品"apple" 并计算了 3D坐标 + 抓取信息:
    std::string object_name = "apple";
    float x = 0.12f;   // 例: 0.12 m
    float y = -0.05f; 
    float z = 0.60f;
    float angle = 0.7f;  // 弧度
    float width = 0.03f; // 夹爪宽度
    float score = 0.95f; // 抓取评分

    // ========== (D) 填充DetectionInfo并发布 ==========
    object_interface::msg::DetectionInfo msg;
    msg.object_name = object_name;
    msg.x = x;
    msg.y = y;
    msg.z = z;
    msg.angle = angle;
    msg.width = width;
    msg.score = score;

    detection_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Published detection info: %s (%.2f,%.2f,%.2f)",
                object_name.c_str(), x, y, z);

    // ========== (E) 若需要OpenCV可视化, 做debug ==========
    // cv::imshow("color", color_frame);
    // cv::waitKey(1);
}

// main函数
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YoloV8InferNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
