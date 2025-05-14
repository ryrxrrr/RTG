#ifndef MY_BRIDGE_PKG__BRIDGE_NODE_HPP_
#define MY_BRIDGE_PKG__BRIDGE_NODE_HPP_

#include <memory>
#include <string>
#include <cmath>
#include <iostream>
#include <deque>
#include <unordered_map>

// ROS2 基础头
#include <rclcpp/rclcpp.hpp>

// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// 消息 (已升级为数组字段)
#include "my_bridge_pkg/msg/grasp_point.hpp"

// geometry
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Eigen
#include <Eigen/Dense>

// 可视化
#include <visualization_msgs/msg/marker_array.hpp>

/**
 * 卡尔曼滤波类
 * 状态维度=10: [x, y, z, angle, width, dx, dy, dz, dAngle, dWidth]^T
 * 测量维度=5:   [x, y, z, angle, width]^T
 */
class KalmanFilterGrasp
{
public:
  KalmanFilterGrasp();

  void init_state(const Eigen::VectorXf &meas);
  bool isInitialized() const;

  void predict();
  void update(const Eigen::VectorXf &meas);

  void get_state(float &X, float &Y, float &Z, float &Angle, float &W) const;

private:
  float wrap_angle(float angle) const;

  int dim_state_;
  int dim_meas_;
  bool initialized_;

  Eigen::VectorXf x_;  // 10x1
  Eigen::MatrixXf P_;  // 10x10
  Eigen::MatrixXf F_;  // 10x10
  Eigen::MatrixXf H_;  // 5x10
  Eigen::MatrixXf Q_;  // 10x10
  Eigen::MatrixXf R_;  // 5x5
};

/**
 * BridgeNode: 
 *  - 订阅 /yolov8_infer/grasp_point (数组型，多目标)
 *  - 对同一帧中每个目标，依据其 object_label 分配到对应的 KF
 *  - 做相机坐标 -> world 坐标 TF 变换
 *  - 做卡尔曼滤波 (每个label各自的predict + update)
 *  - 发布到 /bridge/world_grasp_point (世界坐标)
 *  - 额外用 MarkerArray 可视化箭头
 */
class BridgeNode : public rclcpp::Node
{
public:
  BridgeNode();

private:
  // 回调：处理一帧的多个目标（msg中的数组）
  void graspCallback(const my_bridge_pkg::msg::GraspPoint::SharedPtr msg);

  // 针对"每个标签"维护一个 KF
  std::unordered_map<std::string, KalmanFilterGrasp> kf_map_;

  // 针对"每个标签"维持一个测量队列(最大5)，用于累积5次后取平均
  std::unordered_map<std::string, std::deque<Eigen::VectorXf>> label_measurements_;

  // 订阅 & 发布
  rclcpp::Subscription<my_bridge_pkg::msg::GraspPoint>::SharedPtr sub_;
  rclcpp::Publisher<my_bridge_pkg::msg::GraspPoint>::SharedPtr pub_;

  // 用于可视化抓取点的 Marker
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

#endif  // MY_BRIDGE_PKG__BRIDGE_NODE_HPP_
