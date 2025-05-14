#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <cstdint>  // for uint8_t

#include <rclcpp/rclcpp.hpp>  // 如果要使用 rclcpp::Node::SharedPtr，需要包含
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <yaml-cpp/yaml.h>

// 声明一个简单结构: "ServoInfo" 来存 YAML 加载的舵机信息
struct ServoInfo
{
  std::string joint_name;
  uint8_t     servo_id;
  double      home_position;   // 从yaml “sleep_positions”或“home_positions”而来
  double      lower_limit;     // 如果yaml里有, 或可从URDF/joint limit获得
  double      upper_limit;
  // 还可加其它信息(偏移offset, 二次ID, 等等)
};

namespace sdk_sagittarius_arm
{

class SagittariusSystemHardware 
  : public hardware_interface::SystemInterface
{
public:
  // 宏: 给此类加上共享指针别名
  RCLCPP_SHARED_PTR_DEFINITIONS(SagittariusSystemHardware)

  //==== 构造函数 & 析构函数 ====
  SagittariusSystemHardware();
  ~SagittariusSystemHardware() override;

  //=====================================================
  // 1) 生命周期方法 (ros2_control)
  //=====================================================
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  //=====================================================
  // 2) 导出 State / Command 接口
  //=====================================================
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  //=====================================================
  // 3) 读/写
  //=====================================================
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  //=====================================================
  // A) 从 hardware_info 中解析到的参数
  //=====================================================
  std::string serial_port_;       // 串口名, 例如 /dev/ttyUSB0
  int         baud_rate_;         // 波特率
  std::string servo_config_file_; // e.g. "/home/.../sagittarius_arm.yaml"

  //=====================================================
  // B) YAML 加载到内存的舵机列表
  //=====================================================
  std::vector<ServoInfo> servo_infos_;  // e.g. joint1..joint6..gripper

  //=====================================================
  // C) 与 ROS 2 Control 交互的关节
  //=====================================================
  std::vector<std::string> joint_names_;

  // 状态向量: 与 servo_infos_ 或 joint_names_ 对应  
  // （若二者顺序完全一致，可直接下标对应；否则需用一个映射表）
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;

  // 命令向量
  std::vector<double> joint_position_command_;

  //=====================================================
  // D) 串口相关
  //=====================================================
  int  fd_;          // 串口文件描述符
  bool port_opened_; // 串口是否已打开

  //=====================================================
  // E) 缓冲区/解析用 (若需要在 read() 中自己解析帧)
  //=====================================================
  static const int RECV_BUFFER_SIZE = 512;
  unsigned char mRecvBuffer[RECV_BUFFER_SIZE];  // 本次 read() 到的数据
  unsigned char mFrameBuffer[RECV_BUFFER_SIZE]; // 拼接成完整帧
  size_t        mDataLength;                    // 当前帧缓冲中已有的数据长度

  // 如果你要存当前关节角度(比如原先 sdk_sagittarius_arm_common.cpp 那种)
  double current_joint_positions[6]; // 如果你是6轴臂，可用6大小

  //=====================================================
  // F) 节点指针 (可用于日志、参数或别的ROS接口)
  //=====================================================
  rclcpp::Node::SharedPtr node_;

private:
  //=====================================================
  // G) 辅助函数
  //=====================================================
  /**
   * @brief 从 YAML 读取 servo_config_file_，并填充 servo_infos_
   * @param file_path YAML 配置文件路径
   * @return true 成功读取并解析; false 失败
   */
  bool read_servo_config_yaml(const std::string &file_path);

  /**
   * @brief 打开/配置串口(真实硬件时)
   * @param port 串口名称
   * @param baudrate 波特率
   * @return true 成功打开; false 打开失败
   */
  bool open_serial_port(const std::string &port, int baudrate);

  /**
   * @brief 关闭串口
   */
  void close_serial_port();

  /**
   * @brief 对收到的关节反馈帧进行解析，更新 current_joint_positions[]
   * @param payload 指向帧 payload 部分的数据
   */
  void parse_joint_feedback(const unsigned char* payload);
};

} // end namespace sdk_sagittarius_arm
