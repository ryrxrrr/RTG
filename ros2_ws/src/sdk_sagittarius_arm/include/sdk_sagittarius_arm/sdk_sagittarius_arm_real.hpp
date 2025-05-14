#ifndef SDK_SAGITTARIUS_ARM_REAL_
#define SDK_SAGITTARIUS_ARM_REAL_

#include <memory>
#include <string>
#include <vector>

// ---- ROS2 公共头文件 ----
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// ---- ROS2 消息/服务 ----
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

// ---- 其他依赖 ----
#include <sdk_sagittarius_arm/msg/arm_rad_control.hpp>
#include <sdk_sagittarius_arm/msg/single_rad_control.hpp>
#include <sdk_sagittarius_arm/srv/arm_info.hpp>
#include <sdk_sagittarius_arm/srv/servo_rt_info.hpp>
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_common_serial.hpp>  // 派生类

// -------------------
//  舵机结构体
// -------------------
struct Servo
{
  std::string name;      // 舵机所在的关节名
  uint8_t     servo_id;  // 舵机 ID
};

namespace sdk_sagittarius_arm
{

using FollowJointTrajectory     = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTraj = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class SagittariusArmReal : public rclcpp::Node
{
public:
  //------------------------------------------------
  //           构造函数 & 析构函数
  //------------------------------------------------
  explicit SagittariusArmReal(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~SagittariusArmReal();

  /**
   * @brief 后置初始化函数。请在外部用 make_shared 创建后，再调用 onInit()，
   * 避免在构造函数里 shared_from_this() 导致 std::bad_weak_ptr。
   */
  void onInit();

  //------------------------------------------------
  //          订阅回调
  //------------------------------------------------
  /// 从外部发布 /joint_states（若只是 rviz_control）时使用
  void JointStatesCb(const sensor_msgs::msg::JointState::SharedPtr cmd_arm);

  /// 直接接收 JointTrajectory 并存入 jnt_tra_msg
  void arm_joint_trajectory_msg_callback(
      const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  /// 直接接收 Gripper 的 JointTrajectory 并存入 gripper_tra_msg
  void arm_gripper_trajectory_msg_callback(
      const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  //------------------------------------------------
  //          Action 回调
  //------------------------------------------------
  /// 处理关节轨迹
  void arm_joint_trajectory_action_callback(
      std::shared_ptr<GoalHandleFollowJointTraj> goal_handle);

  /// 处理爪子轨迹
  void arm_gripper_trajectory_action_callback(
      std::shared_ptr<GoalHandleFollowJointTraj> goal_handle);

  //------------------------------------------------
  //         定时器回调
  //------------------------------------------------
  /// 定时执行关节轨迹
  void arm_execute_joint_trajectory();

  /// 定时执行爪子轨迹
  void arm_execute_gripper_trajectory();

  //------------------------------------------------
  //       关节/爪子具体控制函数
  //------------------------------------------------
  short arm_calculate_gripper_degree_position(const float dist);
  void  arm_set_gripper_linear_position(const float dist);
  void  arm_set_single_joint_degree_position(short g_degree);

  /// 指令控制关节
  void  arm_set_joint_positions(const double joint_positions[], double diff_time);
  void  arm_write_joint_commands(const sdk_sagittarius_arm::msg::ArmRadControl &msg);
  void  arm_write_gripper_command(const std_msgs::msg::Float64 &msg);

  /// 力矩控制
  void  ControlTorque(const std_msgs::msg::String::SharedPtr msg);

  //------------------------------------------------
  //         加载 servo 配置
  //------------------------------------------------
  void  arm_get_servo_configs();

  //------------------------------------------------
  //          服务回调
  //------------------------------------------------
  bool arm_get_servo_info(
      const std::shared_ptr<srv::ServoRtInfo::Request> request,
      std::shared_ptr<srv::ServoRtInfo::Response>      response);

  bool arm_get_robot_info(
      const std::shared_ptr<srv::ArmInfo::Request>  request,
      std::shared_ptr<srv::ArmInfo::Response>       response);

  //------------------------------------------------
  //   servo_velocity / servo_acceleration / torque
  //------------------------------------------------
  bool GetAndSetServoAcceleration(CSDarmCommon *pt);
  bool GetAndSetServoVelocity(CSDarmCommon *pt);
  bool GetAndSetServoTorque(CSDarmCommon *pt);

private:
  //------------------------------------------------
  //   如果要将本节点的关节信息发布为 /joint_states
  //------------------------------------------------
  /// 用于发布本节点当前关节角到 /joint_states 或其他话题
  void publish_current_joint_states();

  /// 发布器 & 定时器
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_state_;
  rclcpp::TimerBase::SharedPtr timer_joint_state_pub_;

private:
  //------------------------------------------------
  //   ActionServer
  //------------------------------------------------
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr joint_action_server_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr gripper_action_server_;

  // Joint
  rclcpp_action::GoalResponse handle_joint_traj_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const FollowJointTrajectory::Goal> goal);

  rclcpp_action::CancelResponse handle_joint_traj_cancel(
      const std::shared_ptr<GoalHandleFollowJointTraj> goal_handle);

  void handle_joint_traj_accepted(
      const std::shared_ptr<GoalHandleFollowJointTraj> goal_handle);

  // Gripper
  rclcpp_action::GoalResponse handle_gripper_traj_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const FollowJointTrajectory::Goal> goal);

  rclcpp_action::CancelResponse handle_gripper_traj_cancel(
      const std::shared_ptr<GoalHandleFollowJointTraj> goal_handle);

  void handle_gripper_traj_accepted(
      const std::shared_ptr<GoalHandleFollowJointTraj> goal_handle);

private:
  //------------------------------------------------
  //           关节信息
  //------------------------------------------------
  std::vector<Servo> arm_joints;
  std::vector<Servo> all_joints;

  //------------------------------------------------
  //          状态变量
  //------------------------------------------------
  bool   execute_joint_traj;
  bool   execute_gripper_traj;
  bool   torque_status;
  size_t joint_num_write;
  bool   rviz_control;
  bool   servo_control_trajectory;

  double joint_start_time;
  double gripper_start_time;

  // 轨迹
  trajectory_msgs::msg::JointTrajectory jnt_tra_msg;
  trajectory_msgs::msg::JointTrajectory gripper_tra_msg;

  // 定时器
  rclcpp::TimerBase::SharedPtr tmr_joint_traj_;
  rclcpp::TimerBase::SharedPtr tmr_gripper_traj_;

  // 订阅者
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr           sub_js_;
  rclcpp::Subscription<sdk_sagittarius_arm::msg::ArmRadControl>::SharedPtr sub_joint_commands_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr                 sub_gripper_command_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                  sub_ct_;

  // 服务
  rclcpp::Service<srv::ArmInfo>::SharedPtr     srv_get_robot_info_;
  rclcpp::Service<srv::ServoRtInfo>::SharedPtr srv_get_servo_info_;

  // SDK
  CSDarmCommon *pSDKarm;
  CSDarmCommon *pTest;

  float angle[10];
  std::vector<double> home_positions;
  std::vector<double> sleep_positions;

  uint8_t *joint_ids_read;
  uint8_t *joint_ids_write;

  // 机器人/模型
  const std::string robot_name_;
  const std::string robot_model_;

  // 延后初始化用
  bool        exit_free_torque_{false};
  std::string serial_name_;
  std::string baudrate_;
  int         iTimeLimit_{5};
};

} // namespace sdk_sagittarius_arm

#endif // SDK_SAGITTARIUS_ARM_REAL_
