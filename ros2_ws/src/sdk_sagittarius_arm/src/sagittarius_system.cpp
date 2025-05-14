#include "sdk_sagittarius_arm/sagittarius_system.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <map>
#include <fcntl.h>      // ::open, ::close 等POSIX函数
#include <unistd.h>     // ::read, ::write
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>

namespace sdk_sagittarius_arm
{

//==================== 宏/常量定义 ====================
#ifndef RECV_BUFFER_SIZE
#define RECV_BUFFER_SIZE 512
#endif

#ifndef ExitError
#define ExitError -1
#endif

#ifndef ExitSuccess
#define ExitSuccess 0
#endif

#ifndef PI
#define PI 3.14159265359
#endif

//==================== 构造函数 / 析构函数 ====================
SagittariusSystemHardware::SagittariusSystemHardware()
: node_(nullptr)
, fd_(-1)
, port_opened_(false)
, mDataLength(0)
{
  // 用于暂存串口读取的帧
  memset(mRecvBuffer, 0, RECV_BUFFER_SIZE);
  memset(mFrameBuffer, 0, RECV_BUFFER_SIZE);

  // SDK 中常见的“current_joint_positions[]”，这里模拟放 6 个关节
  for(int i=0; i<6; i++)
  {
    current_joint_positions[i] = 0.0;
  }

  // 若需要的话，也可以在构造函数里初始化 rclcpp::Node（警告：此做法可能与 ros2_control 的架构不完全兼容）
  node_ = rclcpp::Node::make_shared("sagittarius_hw_node");
}

SagittariusSystemHardware::~SagittariusSystemHardware()
{
  close_serial_port();
  RCLCPP_INFO(rclcpp::get_logger("SagittariusSystemHardware"), 
              "SagittariusSystemHardware destructed.");
}

//=====================================================================
// on_init
//=====================================================================
hardware_interface::CallbackReturn 
SagittariusSystemHardware::on_init(const hardware_interface::HardwareInfo &info)
{
  if (hardware_interface::SystemInterface::on_init(info) 
      != hardware_interface::CallbackReturn::SUCCESS)
  {
    std::cerr << "[SagittariusSystemHardware] base on_init failed.\n";
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 1.1 从 <hardware><param> 里读串口名/波特率/servo_config_file
  if (info.hardware_parameters.find("serial_port") != info.hardware_parameters.end())
  {
    serial_port_ = info.hardware_parameters.at("serial_port");
  }
  else
  {
    serial_port_ = "/dev/ttyACM0"; // 默认
  }
  if (info.hardware_parameters.find("baudrate") != info.hardware_parameters.end())
  {
    baud_rate_ = std::stoi(info.hardware_parameters.at("baudrate"));
  }
  else
  {
    baud_rate_ = 115200;
  }
  if (info.hardware_parameters.find("servo_configs") != info.hardware_parameters.end())
  {
    servo_config_file_ = info.hardware_parameters.at("servo_configs");
  }
  else
  {
    servo_config_file_ = "/home/xxx/servo_configs/sagittarius_arm.yaml";
  }

  // 1.2 记录 joint_names
  for (auto &jinfo : info.joints)
  {
    joint_names_.push_back(jinfo.name);
  }
  size_t n_joints = joint_names_.size();
  joint_position_.resize(n_joints, 0.0);
  joint_velocity_.resize(n_joints, 0.0);
  joint_effort_.resize(n_joints, 0.0);
  joint_position_command_.resize(n_joints, 0.0);

  std::cout << "[SagittariusSystemHardware::on_init] #joints=" << n_joints 
            << ", serial=" << serial_port_ 
            << ", baud=" << baud_rate_
            << ", servo_config=" << servo_config_file_
            << std::endl;

  // 1.3 读取 YAML, 填充 servo_infos_
  if (!read_servo_config_yaml(servo_config_file_))
  {
    std::cerr << "[SagittariusSystemHardware] read_servo_config_yaml failed.\n";
    // 这里若读取失败要不要直接 ERROR，由你决定
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

//=====================================================================
// on_configure
//=====================================================================
hardware_interface::CallbackReturn 
SagittariusSystemHardware::on_configure(const rclcpp_lifecycle::State & /*prev_state*/)
{
  // 2.1 打开串口(真实硬件)
  if (!open_serial_port(serial_port_, baud_rate_))
  {
    std::cerr << "[SagittariusSystemHardware] on_configure: open_serial_port failed.\n";
    return hardware_interface::CallbackReturn::ERROR;
  }
  port_opened_ = true;

  // 2.2 [可选] 给舵机发初始化指令，比如 SetArmVel/SetArmAcc/SetTorque/LockOrFree 等
  // 这里演示用：
  RCLCPP_INFO(rclcpp::get_logger("SagittariusSystemHardware"), 
              "on_configure -> you can init servo speed/acc here.");

  // 2.3 [可选] 读取当前关节位置, 并设到 joint_position_
  //    这样在 on_activate() 或之后 read() 时就有初始值
  // ...

  std::cout << "[SagittariusSystemHardware] on_configure() done.\n";
  return hardware_interface::CallbackReturn::SUCCESS;
}

//=====================================================================
// on_activate
//=====================================================================
hardware_interface::CallbackReturn 
SagittariusSystemHardware::on_activate(const rclcpp_lifecycle::State & /*prev_state*/)
{
  // 可以在这里把舵机使能(例如发送锁定指令 SendArmLockOrFree(1))
  RCLCPP_INFO(rclcpp::get_logger("SagittariusSystemHardware"), 
              "on_activate -> servo enabled.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

//=====================================================================
// on_deactivate
//=====================================================================
hardware_interface::CallbackReturn 
SagittariusSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*prev_state*/)
{
  // 关闭扭矩(例如 SendArmLockOrFree(0))
  RCLCPP_INFO(rclcpp::get_logger("SagittariusSystemHardware"), 
              "on_deactivate -> torque off.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

//=====================================================================
// export_state_interfaces
//=====================================================================
std::vector<hardware_interface::StateInterface>
SagittariusSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_itfs;
  state_itfs.reserve(joint_names_.size() * 3);

  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    // position
    state_itfs.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i],
        hardware_interface::HW_IF_POSITION,
        &joint_position_[i]
      )
    );
    // velocity
    state_itfs.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i],
        hardware_interface::HW_IF_VELOCITY,
        &joint_velocity_[i]
      )
    );
    // effort
    state_itfs.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i],
        hardware_interface::HW_IF_EFFORT,
        &joint_effort_[i]
      )
    );
  }

  return state_itfs;
}

//=====================================================================
// export_command_interfaces
//=====================================================================
std::vector<hardware_interface::CommandInterface>
SagittariusSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmd_itfs;
  cmd_itfs.reserve(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    cmd_itfs.emplace_back(
      hardware_interface::CommandInterface(
        joint_names_[i],
        hardware_interface::HW_IF_POSITION,
        &joint_position_command_[i]
      )
    );
  }
  return cmd_itfs;
}

//=====================================================================
// 校验和（与之前 sdk_sagittarius_arm_common.cpp 中逻辑相似）
//=====================================================================
static unsigned char CheckSum(const unsigned char *buf)
{
  // buf[2] 表示数据长度，后面是实际 payload
  unsigned char sum = 0;
  int length = buf[2];
  for(int i = 0; i < length; i++)
  {
    sum += buf[3 + i];
  }
  return sum;
}

//=====================================================================
// 调试打印
//=====================================================================
static void print_hex(const unsigned char *buf, int len)
{
  for(int i=0; i<len; i++)
  {
    printf("%02X ", buf[i]);
  }
  printf("\n");
}

//=====================================================================
// 模拟 publishJointStates 的解析逻辑
// 将解析到的关节角度存到 current_joint_positions[] (6个关节)
//=====================================================================
void SagittariusSystemHardware::parse_joint_feedback(const unsigned char* payload)
{
  // payload包含 6*2=12字节的关节角数据
  // 例如：short raw1 = (short)(payload[0] | (payload[1]<<8));
  // 角度(弧度) = raw / 1800.0 * PI
  // 第1关节
  short raw1 = (short)(payload[0] | (payload[1]<<8));
  double joint1 = raw1 / 1800.0 * PI;
  current_joint_positions[0] = joint1;

  // 第2关节
  short raw2 = (short)(payload[2] | (payload[3]<<8));
  double joint2 = raw2 / 1800.0 * PI;
  current_joint_positions[1] = joint2;

  // 第3关节
  short raw3 = (short)(payload[4] | (payload[5]<<8));
  double joint3 = raw3 / 1800.0 * PI;
  current_joint_positions[2] = joint3;

  // 第4关节
  short raw4 = (short)(payload[6] | (payload[7]<<8));
  double joint4 = raw4 / 1800.0 * PI;
  current_joint_positions[3] = joint4;

  // 第5关节
  short raw5 = (short)(payload[8] | (payload[9]<<8));
  double joint5 = raw5 / 1800.0 * PI;
  current_joint_positions[4] = joint5;

  // 第6关节
  short raw6 = (short)(payload[10] | (payload[11]<<8));
  double joint6 = raw6 / 1800.0 * PI;
  current_joint_positions[5] = joint6;

  // 在真实项目里，这里也可发布 sensor_msgs::msg::JointState
  // 此示例仅将关节角度存到 current_joint_positions[] ，
  // 在 read() 函数结束时再同步到 joint_position_。
}

//=====================================================================
// read()
//   从硬件(串口)读取舵机当前角度(或速度/力), 填充 joint_position_/velocity_/effort_
//=====================================================================
hardware_interface::return_type
SagittariusSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!port_opened_)
  {
    // 如果串口都没开，啥也不做
    return hardware_interface::return_type::OK;
  }

  //=== 1) 从串口读取数据到 mRecvBuffer ===
  fd_set fs_read;
  FD_ZERO(&fs_read);
  FD_SET(fd_, &fs_read);

  struct timeval timeout;
  timeout.tv_sec  = 0;  // 不一定要阻塞
  timeout.tv_usec = 1000; // 1ms

  int fs_sel = select(fd_ + 1, &fs_read, NULL, NULL, &timeout);
  if(fs_sel > 0)
  {
    if(FD_ISSET(fd_, &fs_read))
    {
      int len = ::read(fd_, mRecvBuffer, RECV_BUFFER_SIZE);
      if(len < 0)
      {
        RCLCPP_WARN(rclcpp::get_logger("SagittariusSystemHardware"), 
                    "read() error: %s", strerror(errno));
        return hardware_interface::return_type::OK;
      }
      //=== 2) 将读取到的数据拷贝到 mFrameBuffer 并累计长度 ===
      if ((len < 255) && (mDataLength < 255))
      {
        memcpy(mFrameBuffer + mDataLength, mRecvBuffer, len);
        mDataLength += len;
        // 若帧头不对，则清空
        if(mFrameBuffer[0] != 0x55)
        {
          mDataLength = 0;
        }
      }
      else
      {
        mDataLength = 0;
      }
    }
  }
  //=== 3) 判断是否有完整帧可解析 ===
  if ((mDataLength > 3) && (mDataLength >= (mFrameBuffer[2] + 5)))
  {
    // 帧结构: 0x55 0xAA + LEN(1) + [LEN字节payload] + CHKSUM(1) + 0x7D(1)
    bool valid_frame =
      (mFrameBuffer[0] == 0x55) &&
      (mFrameBuffer[1] == 0xAA) &&
      (mFrameBuffer[mFrameBuffer[2] + 4] == 0x7D) &&
      (CheckSum(mFrameBuffer) == (unsigned char)mFrameBuffer[mFrameBuffer[2] + 3]);

    if(valid_frame)
    {
      //=== 4) 根据命令类型做解析 ===
      // mFrameBuffer[4] = command code, mFrameBuffer[3] = message type
      // 假设 0x06 & 0x01 是关节反馈:
      if(mFrameBuffer[4] == 0x06 && mFrameBuffer[3] == 0x01)
      {
        // payload 从 mFrameBuffer+5 开始
        parse_joint_feedback(mFrameBuffer + 5);

        // 把 current_joint_positions[] 拷贝到 joint_position_
        // 没有速度或力反馈则置 0
        for(size_t i=0; i<joint_position_.size(); i++)
        {
          if(i < 6) // 假设只有前6个joint
            joint_position_[i] = current_joint_positions[i];
          else
            joint_position_[i] = 0.0;

          joint_velocity_[i] = 0.0;
          joint_effort_[i]   = 0.0;
        }
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("SagittariusSystemHardware"), 
                    "Other command code=0x%02X", mFrameBuffer[4]);
      }
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("SagittariusSystemHardware"), 
                  "Invalid frame data!");
      print_hex(mFrameBuffer, (int)mDataLength);
    }
    // 不管是否成功，清空数据长度
    mDataLength = 0;
  }

  return hardware_interface::return_type::OK;
}

//=====================================================================
// write()
//   读取 joint_position_command_ 并通过串口发送给舵机
//=====================================================================
hardware_interface::return_type
SagittariusSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if(!port_opened_)
  {
    return hardware_interface::return_type::OK;
  }

  // 这里演示：将 6 个关节角度写到串口（假设CMD=0x03）
  // 取 joint_position_command_ 的前6个
  // 机械臂SDK中通常需要按协议打包
  // 示例: 6个关节，每个关节2字节 short，单位 0.1° => raw = rad * (1800/PI)
  unsigned char txbuf[30];
  // 帧头
  txbuf[0] = 0x55;
  txbuf[1] = 0xAA;
  txbuf[2] = 12; // payload长(6关节x2字节=12)
  txbuf[3] = 0x01; // TYPE_REQUEST_MESSAGE?
  txbuf[4] = 0x03; // 命令码(自定义)

  for(int i=0; i<6; i++)
  {
    double rad = joint_position_command_[i];
    short raw  = (short)( rad * (1800.0/PI) );
    txbuf[5 + i*2]   = raw & 0xFF;
    txbuf[5 + i*2+1] = (raw>>8)&0xFF;
  }

  // 校验和
  unsigned char sum = 0;
  for(int i=0; i<12; i++)
    sum += txbuf[3 + i];
  txbuf[5 + 6*2] = sum;         // 校验和放在 payload 后1字节
  txbuf[5 + 6*2 + 1] = 0x7D;    // 帧尾

  // 发送总长 = 2(帧头) + 1(LEN) + 12(payload) + 1(校验) + 1(帧尾) = 17
  int send_len = 2 + 1 + 12 + 1 + 1;

  int n = ::write(fd_, txbuf, send_len);
  if(n != send_len)
  {
    RCLCPP_WARN(rclcpp::get_logger("SagittariusSystemHardware"), 
                "write() mismatch: expect %d, got %d", send_len, n);
  }

  return hardware_interface::return_type::OK;
}

//=====================================================================
// 打开串口
//=====================================================================
bool SagittariusSystemHardware::open_serial_port(const std::string &port, int baudrate)
{
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if(fd_ < 0)
  {
    std::cerr << "[SagittariusSystemHardware] open_serial_port fail: " 
              << port << " err=" << strerror(errno) << std::endl;
    return false;
  }
  // 设置串口属性
  struct termios options;
  memset(&options, 0, sizeof(options));
  cfmakeraw(&options);

  // 根据 baudrate 设置
  speed_t speed = B115200;
  // 可以做个简单映射：若 baudrate=1000000 则 speed = B1000000，等等
  // 这里只演示默认用 B115200
  cfsetispeed(&options, speed);
  cfsetospeed(&options, speed);

  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~CRTSCTS;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_oflag &= ~OPOST;
  options.c_cc[VMIN]  = 0;
  options.c_cc[VTIME] = 0;

  tcflush(fd_, TCIFLUSH);
  if(tcsetattr(fd_, TCSANOW, &options) != 0)
  {
    std::cerr << "set device error!\n";
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  std::cout << "[SagittariusSystemHardware] open_serial_port OK: " << port 
            << ", fd=" << fd_ << std::endl;
  return true;
}

//=====================================================================
// 关闭串口
//=====================================================================
void SagittariusSystemHardware::close_serial_port()
{
  if(fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
    port_opened_ = false;
  }
  std::cout << "[SagittariusSystemHardware] close_serial_port.\n";
}

//=====================================================================
// 读取 YAML (简易示例)
//=====================================================================
bool SagittariusSystemHardware::read_servo_config_yaml(const std::string &file_path)
{
  try
  {
    YAML::Node doc = YAML::LoadFile(file_path);
    if (!doc || !doc.IsMap())
    {
      std::cerr << "YAML file invalid: " << file_path << std::endl;
      return false;
    }

    // 获取 "order" + "singles"
    std::vector<std::string> joint_names_in_yaml;
    if(doc["order"] && doc["order"].IsSequence())
    {
      for(size_t i=0; i<doc["order"].size(); i++)
      {
        joint_names_in_yaml.push_back(doc["order"][i].as<std::string>());
      }
    }
    if(doc["singles"] && doc["singles"].IsSequence())
    {
      for(size_t i=0; i<doc["singles"].size(); i++)
      {
        joint_names_in_yaml.push_back(doc["singles"][i].as<std::string>());
      }
    }

    // 逐个解析
    for(auto & jname : joint_names_in_yaml)
    {
      if(!doc[jname])
      {
        std::cerr << "No item for " << jname << " in yaml.\n";
        continue;
      }
      YAML::Node jnode = doc[jname];
      ServoInfo si;
      si.joint_name = jname;
      if(jnode["ID"])
        si.servo_id   = jnode["ID"].as<int>();
      if(jnode["MinAngle"])
        si.lower_limit = jnode["MinAngle"].as<double>();
      if(jnode["MaxAngle"])
        si.upper_limit = jnode["MaxAngle"].as<double>();

      // ...
      servo_infos_.push_back(si);
    }
  }
  catch(std::exception &e)
  {
    std::cerr << "[read_servo_config_yaml] exception: " << e.what() << std::endl;
    return false;
  }

  std::cout << "[SagittariusSystemHardware] read_servo_config_yaml ok, loaded " 
            << servo_infos_.size() << " servo items.\n";
  return true;
}

//=====================================================================
// pluginlib 导出
//=====================================================================
PLUGINLIB_EXPORT_CLASS(
  sdk_sagittarius_arm::SagittariusSystemHardware,
  hardware_interface::SystemInterface
)

} // end namespace sdk_sagittarius_arm
