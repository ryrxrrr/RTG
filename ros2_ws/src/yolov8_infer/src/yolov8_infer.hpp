#ifndef YOLOV8_INFER_HPP_
#define YOLOV8_INFER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

// 可使用 message_filters 进行时间同步 (ApproximateTimeSynchronizer)
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// 引入自定义消息
#include "object_interface/msg/detection_info.hpp"

class YoloV8InferNode : public rclcpp::Node
{
public:
    explicit YoloV8InferNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // 同步回调
    void syncCallback(
        const sensor_msgs::msg::Image::SharedPtr & color_msg,
        const sensor_msgs::msg::Image::SharedPtr & depth_msg,
        const sensor_msgs::msg::CameraInfo::SharedPtr & info_msg);

    // ========== ROS 通信 ========== 
    rclcpp::Publisher<object_interface::msg::DetectionInfo>::SharedPtr detection_pub_;

    // message_filters
    using SyncPolicy = message_filters::sync_policies::ApproximateTime< 
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image,
        sensor_msgs::msg::CameraInfo
    >;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> color_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> info_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // 其他：如果你要做KF/推理，也可在此放置相关成员
};

#endif // YOLOV8_INFER_HPP_
