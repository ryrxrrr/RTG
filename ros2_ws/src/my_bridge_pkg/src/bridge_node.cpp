#include <memory>
#include <string>
#include <cmath>
#include <iostream>
#include <deque>
#include <unordered_map>

// ROS2
#include <rclcpp/rclcpp.hpp>

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 消息
#include "my_bridge_pkg/msg/grasp_point.hpp"  // 数组字段
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Eigen
#include <Eigen/Dense>

// Marker
#include <visualization_msgs/msg/marker_array.hpp>

#include "my_bridge_pkg/bridge_node.hpp"

using std::placeholders::_1;

/*--------------------------------------------------------------------
 * 实现 KalmanFilterGrasp
 *------------------------------------------------------------------*/
KalmanFilterGrasp::KalmanFilterGrasp()
: dim_state_(10),
  dim_meas_(5),
  initialized_(false)
{
  x_ = Eigen::VectorXf::Zero(dim_state_);
  P_ = Eigen::MatrixXf::Identity(dim_state_, dim_state_) * 1000.0f;

  F_ = Eigen::MatrixXf::Identity(dim_state_, dim_state_);
  // x[0]+=dx; y[1]+=dy; z[2]+=dz; angle[3]+=dAngle; width[4]+=dWidth
  F_(0, 5) = 1.0f;
  F_(1, 6) = 1.0f;
  F_(2, 7) = 1.0f;
  F_(3, 8) = 1.0f;
  F_(4, 9) = 1.0f;

  H_ = Eigen::MatrixXf::Zero(dim_meas_, dim_state_);
  for(int i=0; i<dim_meas_; i++){
    H_(i, i) = 1.0f;  // meas[i] = state[i]
  }

  Q_ = Eigen::MatrixXf::Zero(dim_state_, dim_state_);
  // 可根据实际情况调整噪声大小
  Q_.diagonal() << 1e-4, 1e-4, 1e-4, 1e-5, 1e-5,
                   1e-3, 1e-3, 1e-3, 1e-4, 1e-4;

  R_ = Eigen::MatrixXf::Zero(dim_meas_, dim_meas_);
  R_.diagonal() << 1e-3, 1e-3, 1e-3, 1e-4, 1e-4;
}

void KalmanFilterGrasp::init_state(const Eigen::VectorXf &meas)
{
  // meas: [x, y, z, angle, width]
  x_.head(5) = meas;
  P_ = Eigen::MatrixXf::Identity(dim_state_, dim_state_) * 1.0f;
  initialized_ = true;
}

bool KalmanFilterGrasp::isInitialized() const
{
  return initialized_;
}

void KalmanFilterGrasp::predict()
{
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
  // wrap angle
  x_(3) = wrap_angle(x_(3));
}

void KalmanFilterGrasp::update(const Eigen::VectorXf &meas)
{
  // meas: [x, y, z, angle, width]
  Eigen::MatrixXf S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXf K = P_ * H_.transpose() * S.inverse();

  Eigen::VectorXf y = meas - H_ * x_;
  // wrap angle in residual
  y(3) = wrap_angle(y(3));

  x_ = x_ + K * y;

  Eigen::MatrixXf I = Eigen::MatrixXf::Identity(dim_state_, dim_state_);
  P_ = (I - K * H_) * P_;

  x_(3) = wrap_angle(x_(3));
}

void KalmanFilterGrasp::get_state(float &X, float &Y, float &Z, float &Angle, float &W) const
{
  X = x_(0);
  Y = x_(1);
  Z = x_(2);
  Angle = x_(3);
  W = x_(4);
}

float KalmanFilterGrasp::wrap_angle(float angle) const
{
  float res = std::fmod(angle + M_PI, 2.0f*M_PI);
  if(res < 0) res += 2.0f*M_PI;
  return res - M_PI;
}

/*--------------------------------------------------------------------
 * BridgeNode
 *  - 订阅 /yolov8_infer/grasp_point (数组)
 *  - 对 msg->object_label[i] 匹配到 kf_map_[label]
 *  - 做卡尔曼滤波 + TF变换 + 发布到 /bridge/world_grasp_point
 *  - 额外发布 MarkerArray 来可视化抓取点为箭头
 *------------------------------------------------------------------*/
BridgeNode::BridgeNode()
: Node("bridge_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  RCLCPP_INFO(this->get_logger(), "BridgeNode initializing...");

  // 订阅 (数组型GraspPoint)
  sub_ = create_subscription<my_bridge_pkg::msg::GraspPoint>(
    "/yolov8_infer/grasp_point",
    10,
    std::bind(&BridgeNode::graspCallback, this, _1)
  );

  // 发布
  pub_ = create_publisher<my_bridge_pkg::msg::GraspPoint>(
    "/bridge/world_grasp_point",
    10
  );

  // 新增：可视化 MarkerArray 的发布者
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/grasp_markers",
    10
  );

  RCLCPP_INFO(this->get_logger(), "BridgeNode initialized.");
}

void BridgeNode::graspCallback(const my_bridge_pkg::msg::GraspPoint::SharedPtr msg)
{
  // msg 里有多个目标
  size_t n = msg->object_label.size();
  if(n == 0) {
    RCLCPP_INFO(this->get_logger(), "Received empty GraspPoint array, skip.");
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Received GraspPoint array with %zu items => process each label separately",
    n
  );

  // 准备一个“本帧要发布”的结果(数组)
  my_bridge_pkg::msg::GraspPoint world_msg;
  world_msg.object_label.resize(n);
  world_msg.x.resize(n);
  world_msg.y.resize(n);
  world_msg.z.resize(n);
  world_msg.angle.resize(n);
  world_msg.width.resize(n);
  world_msg.score.resize(n);
  world_msg.confidence.resize(n);
  world_msg.frame_id.resize(n);

  // 准备一个 MarkerArray，用于可视化多个目标
  visualization_msgs::msg::MarkerArray marker_array_msg;

  // 遍历每个目标
  for(size_t i = 0; i < n; i++)
  {
    std::string label = msg->object_label[i];
    float cx = msg->x[i];
    float cy = msg->y[i];
    float cz = msg->z[i];
    float cangle = msg->angle[i];
    float cwidth = msg->width[i];
    float cscore = msg->score[i];
    float cconf = msg->confidence[i];
    std::string cframe = msg->frame_id[i];  // 通常是相机坐标系

    RCLCPP_INFO(
      this->get_logger(),
      "  [%zu] label=%s, camera=(%.3f,%.3f,%.3f), angle=%.2f, width=%.5f, score=%.2f, conf=%.2f",
      i, label.c_str(), cx, cy, cz, cangle, cwidth, cscore, cconf
    );    

    // -----------
    // (A) 先构造 PoseStamped (相机坐标系)
    // -----------
    geometry_msgs::msg::PoseStamped ps_in;
    ps_in.header.frame_id = cframe;  // 例如 "camera_link"
    // 建议使用 Node 当前时刻，也可以看情况用消息自带时间
    ps_in.header.stamp = this->now();  

    // 设置位置 (在相机系中的 x,y,z)
    ps_in.pose.position.x = cx;
    ps_in.pose.position.y = cy;
    ps_in.pose.position.z = cz;

    // 设置姿态：假设 angle 表示绕 Z 轴的旋转
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, cangle);
    ps_in.pose.orientation = tf2::toMsg(q);

    // -----------
    // (B) TF => target_frame (例如 "world")
    //     与 Python 逻辑类似，传入 1 秒的超时时间
    // -----------
    geometry_msgs::msg::TransformStamped transform_stamped;
    std::string target_frame = "world";  // 或者 "base_link"，看需求
    try {
      transform_stamped = tf_buffer_.lookupTransform(
        target_frame,                 // to
        ps_in.header.frame_id,        // from
        tf2::TimePointZero,           // 查最新可用的变换
        tf2::durationFromSec(1.0)     // 超时时间 1s
      );
    }
    catch(tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
      continue; // 跳过这个目标
    }

    geometry_msgs::msg::PoseStamped ps_out;
    tf2::doTransform(ps_in, ps_out, transform_stamped);

    // -----------
    // (C) 提取世界坐标系下的 (x, y, z, angle)
    // -----------
    float xw = ps_out.pose.position.x;
    float yw = ps_out.pose.position.y;
    float zw = ps_out.pose.position.z;

    tf2::Quaternion q_out;
    tf2::fromMsg(ps_out.pose.orientation, q_out);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_out).getRPY(roll, pitch, yaw);

    float angle_w = static_cast<float>(yaw);
    float width_w = cwidth; // 简单假设相同宽度

    // -----------
    // (D) 构造测量并做 KF predict + update
    // -----------
    Eigen::VectorXf meas(5);
    meas << xw, yw, zw, angle_w, width_w;

    auto it = kf_map_.find(label);
    if(it == kf_map_.end()) {
      // 不存在 => 创建
      KalmanFilterGrasp new_kf;
      new_kf.init_state(meas);
      kf_map_[label] = new_kf;
      label_measurements_[label].clear();  
      RCLCPP_INFO(this->get_logger(), "  => Create new KF for label=%s", label.c_str());
    }

    KalmanFilterGrasp &kf = kf_map_[label];
    kf.predict();

    // (D1) 累积测量
    label_measurements_[label].push_back(meas);
    if(label_measurements_[label].size() >= 5)
    {
      // 取平均后 update
      Eigen::VectorXf avg_meas = Eigen::VectorXf::Zero(5);
      for(auto &mm : label_measurements_[label]) {
        avg_meas += mm;
      }
      avg_meas /= static_cast<float>(label_measurements_[label].size());
      label_measurements_[label].clear();

      kf.update(avg_meas);
    }
    else
    {
      // 不满5次 => 直接 update
      kf.update(meas);
    }

    // (D2) 获取滤波后状态
    float Xf, Yf, Zf, Anglef, Wf;
    kf.get_state(Xf, Yf, Zf, Anglef, Wf);

    RCLCPP_INFO(
      this->get_logger(),
      "    => KF result: world=(%.3f, %.3f, %.3f), angle=%.2f, width=%.2f",
      Xf, Yf, Zf, Anglef, Wf
    );    

    // -----------
    // (E) 填充到要发布的 world_msg
    // -----------
    world_msg.object_label[i] = label;
    world_msg.x[i] = Xf;
    world_msg.y[i] = Yf;
    world_msg.z[i] = Zf;
    world_msg.angle[i] = Anglef;
    world_msg.width[i] = Wf;
    world_msg.score[i] = cscore;      // 沿用原 score
    world_msg.confidence[i] = cconf;  // 沿用原 confidence
    world_msg.frame_id[i] = target_frame; // 例如 "world"

    // -----------
    // (F) 为可视化创建一个 Marker (ARROW)
    // -----------
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = target_frame;   // 与 world_msg 一致
    marker.header.stamp = this->now();

    // 使用 label 作为 Marker 的命名空间
    marker.ns = label;
    // id 为 i 或者更复杂逻辑都可，这里简单用下标
    marker.id = static_cast<int>(i);

    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Marker的位姿
    marker.pose.position.x = Xf;
    marker.pose.position.y = Yf;
    marker.pose.position.z = Zf;

    // 用滤波后的 angle (假设只绕Z轴)
    tf2::Quaternion q_marker;
    q_marker.setRPY(0.0, 0.0, Anglef);
    marker.pose.orientation = tf2::toMsg(q_marker);

    // 箭头大小: x=长度, y=z=箭头的直径
    marker.scale.x = 0.15f;  // 箭身长度
    marker.scale.y = 0.03f;  // 箭头宽度
    marker.scale.z = 0.03f;

    // 颜色(红色), alpha=1.0
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    // 寿命(0表示不自动删除)
    marker.lifetime = rclcpp::Duration(0, 0);

    // 加入 MarkerArray
    marker_array_msg.markers.push_back(marker);
  }

  // 发布世界坐标下的 GraspPoint
  pub_->publish(world_msg);

  // 发布可视化 MarkerArray
  marker_pub_->publish(marker_array_msg);
}
