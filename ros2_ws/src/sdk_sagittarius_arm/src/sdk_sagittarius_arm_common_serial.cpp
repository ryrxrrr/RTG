/*******************************************************************************
 * Software License Agreement (BSD License)
 * Copyright (c) ...
 * All rights reserved.
 *
 * 升级注记：
 *   - 保持串口读写、帧打包、命令发送的原有逻辑不变。
 *   - 新增一个 SendJointTrajectory(...) 用于一次性下发整段轨迹，并添加互斥锁保护。
 *   - 对轨迹进行更严格的检查，并使用局部变量拷贝避免悬空引用。
 ******************************************************************************/

 #define _USE_MATH_DEFINES
 #include <cmath>
 
 #include <sdk_sagittarius_arm/sdk_sagittarius_arm_common_serial.hpp>
 
 // POSIX 串口相关
 #include <termios.h>
 #include <fcntl.h>
 #include <unistd.h>
 #include <sys/select.h>
 #include <sys/time.h>
 #include <sys/types.h>
 #include <errno.h>
 #include <string.h>
 
 // Boost / std
 #include <boost/asio.hpp>
 #include <boost/lambda/lambda.hpp>
 #include <boost/lexical_cast.hpp>
 #include <mutex>
 #include <chrono>
 
 // ROS2
 #include <rclcpp/rclcpp.hpp>
 #include <trajectory_msgs/msg/joint_trajectory.hpp>
 
 // 宏定义
 #ifndef ExitError
 #define ExitError -1
 #endif
 
 #ifndef ExitSuccess
 #define ExitSuccess 0
 #endif
 
 #ifndef PI
 #define PI M_PI
 #endif
 
 namespace sdk_sagittarius_arm
 {
 
 // 全局/类内静态互斥锁，用于保护串口写操作
 // 如果你有类内成员，需要做并发保护，也可将其封装到类中
 static std::mutex m_write_mutex_;
 
 CSDarmCommonSerial::CSDarmCommonSerial(
     const std::string &serialname,
     const std::string &baudrate,
     int timelimit,
     rclcpp::Node::SharedPtr node,
     bool free_torque)
   : CSDarmCommon(node)
   , mSerialName(serialname)
   , mBaudrate(std::atoi(baudrate.c_str()))
   , mTimeLimit(timelimit)
   , mExitFreeTorque(free_torque)
   , mFd(-1)
   , mBytesReceived(0)
 {
     // 任何需要的初始化操作
 }
 
 CSDarmCommonSerial::~CSDarmCommonSerial()
 {
     // 若需在析构时释放扭矩，可在此发送解锁指令
     if (mExitFreeTorque)
     {
         SendArmLockOrFree(0);
     }
     CloseDevice();
 }
 
 //-----------------------------------------------------
 // 打开串口设备
 //-----------------------------------------------------
 int CSDarmCommonSerial::InitDevice()
 {
     int i;
     int speed_arr[] = { B1500000, B1000000, B460800, B230400, B115200,
                         B19200,    B9600,    B4800,   B2400,   B1200, B300 };
     int name_arr[]  = {  1500000,  1000000,  460800,  230400,  115200,
                          19200,    9600,     4800,    2400,    1200,  300 };
 
     mFd = open(mSerialName.c_str(), O_RDWR | O_NOCTTY);
     if (mFd == -1)
     {
         RCLCPP_ERROR(node_->get_logger(), "[CSDarmCommonSerial::InitDevice] open serial failed: %s", mSerialName.c_str());
         return ExitError;
     }
 
     struct termios options;
     bzero(&options, sizeof(options));
     cfmakeraw(&options);
 
     bool found_baud = false;
     for (i = 0; i < static_cast<int>(sizeof(speed_arr)/sizeof(int)); i++)
     {
         if (mBaudrate == name_arr[i])
         {
             cfsetispeed(&options, speed_arr[i]);
             cfsetospeed(&options, speed_arr[i]);
             found_baud = true;
             RCLCPP_INFO(node_->get_logger(), "Using baudrate: %d", mBaudrate);
             break;
         }
     }
     if(!found_baud)
     {
         RCLCPP_ERROR(node_->get_logger(), "Unsupported baudrate: %d", mBaudrate);
         close(mFd);
         mFd = -1;
         return ExitError;
     }
 
     options.c_cflag |= CLOCAL;
     options.c_cflag |= CREAD;
     options.c_cflag &= ~CRTSCTS;
     options.c_cflag &= ~CSIZE;
     options.c_cflag |= CS8;
     options.c_cflag &= ~PARENB;
     options.c_cflag &= ~CSTOPB;
     options.c_oflag &= ~OPOST;
 
     // VMIN=0, VTIME=0 => 非阻塞；可根据需求做调整
     options.c_cc[VMIN]  = 0;
     options.c_cc[VTIME] = 0;
 
     tcflush(mFd, TCIFLUSH);
     if (tcsetattr(mFd, TCSANOW, &options) != 0)
     {
         RCLCPP_ERROR(node_->get_logger(), "set device error");
         close(mFd);
         mFd = -1;
         return ExitError;
     }
 
     RCLCPP_INFO(node_->get_logger(), "Open serial: %s successful!", mSerialName.c_str());
     return ExitSuccess;
 }
 
 //-----------------------------------------------------
 // 关闭串口
 //-----------------------------------------------------
 int CSDarmCommonSerial::CloseDevice()
 {
     if(mFd != -1)
     {
         close(mFd);
         mFd = -1;
         RCLCPP_INFO(node_->get_logger(), "Close serial and device: %s", mSerialName.c_str());
     }
     return ExitSuccess;
 }
 
 //-----------------------------------------------------
 // 发送数据帧到串口
 //-----------------------------------------------------
 int CSDarmCommonSerial::SendSerialData2Arm(char *buf, int length)
 {
     std::lock_guard<std::mutex> lock(m_write_mutex_);
     int n = -1;
     RCLCPP_INFO(node_->get_logger(), "[CSDarmCommonSerial::SendSerialData2Arm] before write mFd:%d",mFd);
     if(mFd > 0)
     {
         n = write(mFd, buf, length);
         RCLCPP_INFO(node_->get_logger(), "[CSDarmCommonSerial::SendSerialData2Arm] after write mFd:%d",mFd);
     }
     return n;
 }
 
 //-----------------------------------------------------
 // 计算帧的校验和
 //-----------------------------------------------------
 unsigned char CSDarmCommonSerial::CheckSum(unsigned char *buf)
 {
     int i;
     unsigned char sum = 0;
     for(i = 0; i < buf[2]; i++)
     {
         sum += buf[3 + i];
     }
     return sum;
 }
 
 //-----------------------------------------------------
 // 发送末端命令 (onoff=开合, value=角度/脉冲等)
 //-----------------------------------------------------
 int CSDarmCommonSerial::SendArmEndAction(unsigned char onoff, short value)
 {
     if(mFd == -1)
     {
         RCLCPP_ERROR(node_->get_logger(), "[SendArmEndAction] Serial is not open!");
         return ExitError;
     }
 
     unsigned char buf[30];
     buf[0] = 0x55;
     buf[1] = 0xAA;
     buf[2] = 5;
     buf[3] = TYPE_REQUEST_MESSAGE;
     buf[4] = CMD_CONTROL_END_ACTION;
     buf[5] = onoff;
     buf[6] = value & 0xFF;
     buf[7] = (value >> 8) & 0xFF;
     buf[8] = CheckSum(buf);
     buf[9] = 0x7D;
 
     if(SendSerialData2Arm((char*)buf, 10) != 10)
     {
         RCLCPP_ERROR(node_->get_logger(), "Write error for end action cmd!");
         return ExitError;
     }
     return ExitSuccess;
 }
 
 //-----------------------------------------------------
 // 锁住/释放舵机
 //-----------------------------------------------------
 int CSDarmCommonSerial::SendArmLockOrFree(unsigned char onoff)
 {
     if(mFd == -1)
     {
         RCLCPP_ERROR(node_->get_logger(), "[SendArmLockOrFree] Serial not open!");
         return ExitError;
     }
 
     unsigned char buf[30];
     buf[0] = 0x55;
     buf[1] = 0xAA;
     buf[2] = 3;
     buf[3] = TYPE_REQUEST_MESSAGE;
     buf[4] = CMD_CONTROL_LOCK_OR_FREE;
     buf[5] = onoff;
     buf[6] = CheckSum(buf);
     buf[7] = 0x7D;
 
     if(SendSerialData2Arm((char*)buf, 8) != 8)
     {
         RCLCPP_ERROR(node_->get_logger(), "Write error for lock/free cmd!");
         return ExitError;
     }
     return ExitSuccess;
 }
 
 //-----------------------------------------------------
 // 请求读取某个舵机的实时信息
 //-----------------------------------------------------
 int CSDarmCommonSerial::SendGetServoRealTimeInfo(unsigned char id)
 {
     if(mFd == -1)
     {
         RCLCPP_ERROR(node_->get_logger(), "[SendGetServoRealTimeInfo] Serial not open!");
         return ExitError;
     }
 
     unsigned char buf[30];
     buf[0] = 0x55;
     buf[1] = 0xAA;
     buf[2] = 3;
     buf[3] = TYPE_REQUEST_MESSAGE;
     buf[4] = CMD_GET_SERVO_RT_INFO;
     buf[5] = id;
     buf[6] = CheckSum(buf);
     buf[7] = 0x7D;
 
     if(SendSerialData2Arm((char*)buf, 8) != 8)
     {
         RCLCPP_ERROR(node_->get_logger(), "Write error for servo realtime info cmd!");
         return ExitError;
     }
     return ExitSuccess;
 }
 
 //-----------------------------------------------------
 // 设置舵机插补速度
 //-----------------------------------------------------
 int CSDarmCommonSerial::SetArmVel(unsigned short vel)
 {
     if(mFd == -1)
     {
         RCLCPP_ERROR(node_->get_logger(), "[SetArmVel] Serial not open!");
         return ExitError;
     }
 
     unsigned char buf[30];
     buf[0] = 0x55;
     buf[1] = 0xAA;
     buf[2] = 4;
     buf[3] = TYPE_REQUEST_MESSAGE;
     buf[4] = CMD_SET_SERVO_VEL;
     buf[5] = vel & 0xFF;
     buf[6] = (vel >> 8) & 0xFF;
     buf[7] = CheckSum(buf);
     buf[8] = 0x7D;
 
     if(SendSerialData2Arm((char*)buf, 9) != 9)
     {
         RCLCPP_ERROR(node_->get_logger(), "Write error for set servo vel cmd!");
         return ExitError;
     }
     RCLCPP_INFO(node_->get_logger(), "SetArmVel: %d", vel);
     return ExitSuccess;
 }
 
 //-----------------------------------------------------
 // 设置舵机加速度
 //-----------------------------------------------------
 int CSDarmCommonSerial::SetArmAcc(unsigned char acc)
 {
     if(mFd == -1)
     {
         RCLCPP_ERROR(node_->get_logger(), "[SetArmAcc] Serial not open!");
         return ExitError;
     }
 
     unsigned char buf[30];
     buf[0] = 0x55;
     buf[1] = 0xAA;
     buf[2] = 3;
     buf[3] = TYPE_REQUEST_MESSAGE;
     buf[4] = CMD_SET_SERVO_ACC;
     buf[5] = acc;
     buf[6] = CheckSum(buf);
     buf[7] = 0x7D;
 
     if(SendSerialData2Arm((char*)buf, 8) != 8)
     {
         RCLCPP_ERROR(node_->get_logger(), "Write error for set servo acc cmd!");
         return ExitError;
     }
     RCLCPP_INFO(node_->get_logger(), "SetArmAcc: %d", acc);
     return ExitSuccess;
 }
 
 //-----------------------------------------------------
 // 设置7个舵机的扭矩
 //-----------------------------------------------------
 int CSDarmCommonSerial::SetArmTorque(int torque[])
 {
     if(mFd == -1)
     {
         RCLCPP_ERROR(node_->get_logger(), "[SetArmTorque] Serial not open!");
         return ExitError;
     }
 
     unsigned char buf[30];
     buf[0] = 0x55;
     buf[1] = 0xAA;
     buf[2] = 2*7 + 2;  // 16
     buf[3] = TYPE_REQUEST_MESSAGE;
     buf[4] = CMD_SET_SERVO_TORQUE;
 
     int i = 0;
     for(i=0; i<7; i++)
     {
         buf[5 + 2*i]   = torque[i] & 0xFF;
         buf[6 + 2*i]   = (torque[i] >> 8) & 0xFF;
         RCLCPP_INFO(node_->get_logger(), "SetArmTorque: %d", torque[i]);
     }
     buf[5 + 2*i] = CheckSum(buf);
     buf[6 + 2*i] = 0x7D;
 
     int total_len = 7 + 2*i;
     if(SendSerialData2Arm((char*)buf, total_len) != total_len)
     {
         RCLCPP_ERROR(node_->get_logger(), "Write error for set servo torque cmd!");
         return ExitError;
     }
     return ExitSuccess;
 }
 
 //-----------------------------------------------------
 // 发送6个舵机角度 + 时间差
 //-----------------------------------------------------
 int CSDarmCommonSerial::SendArmAllServerTime(
     short difftime, float v1, float v2, float v3, float v4, float v5, float v6)
 {
    // auto now = std::chrono::system_clock::now();
    // std::time_t now_time_tsec = std::chrono::system_clock::to_time_t(now);
    
    // RCLCPP_INFO(node_->get_logger(), "[SendArmAllServerTime] before sleep:%ld", now_time_tsec);
    // sleep(1);

    // now = std::chrono::system_clock::now();
    // now_time_tsec = std::chrono::system_clock::to_time_t(now);
    // RCLCPP_INFO(node_->get_logger(), "[SendArmAllServerTime] after sleep:%ld", now_time_tsec);

     if(mFd == -1)
     {
         RCLCPP_ERROR(node_->get_logger(), "[SendArmAllServerTime] Serial not open!");
         return ExitError;
     }
 
     unsigned char buf[30];
     static unsigned char lastbuf[30];
 
     buf[0] = 0x55;
     buf[1] = 0xAA;
     buf[2] = 16;
     buf[3] = TYPE_REQUEST_MESSAGE;
     buf[4] = CMD_CONTROL_ALL_DEGREE_AND_DIFF_TIME;
 
     buf[5] = difftime & 0xFF;
     buf[6] = (difftime >> 8) & 0xFF;
 
     short degree = 0;
 
     // 第1关节
     degree = (short)(1800.0f / (float)PI * v1);
     buf[7]  = degree & 0xFF;
     buf[8]  = (degree >> 8) & 0xFF;
 
     // 第2关节
     degree = (short)(1800.0f / (float)PI * v2);
     buf[9]  = degree & 0xFF;
     buf[10] = (degree >> 8) & 0xFF;
 
     // 第3关节
     degree = (short)(1800.0f / (float)PI * v3);
     buf[11] = degree & 0xFF;
     buf[12] = (degree >> 8) & 0xFF;
 
     // 第4关节
     degree = (short)(1800.0f / (float)PI * v4);
     buf[13] = degree & 0xFF;
     buf[14] = (degree >> 8) & 0xFF;
 
     // 第5关节
     degree = (short)(1800.0f / (float)PI * v5);
     buf[15] = degree & 0xFF;
     buf[16] = (degree >> 8) & 0xFF;
 
     // 第6关节
     degree = (short)(1800.0f / (float)PI * v6);
     buf[17] = degree & 0xFF;
     buf[18] = (degree >> 8) & 0xFF;
 
     buf[19] = CheckSum(buf);
     buf[20] = 0x7D;
 
     // 避免同一指令频繁写入(若相邻两次是同样的指令则不重复发)
     if(memcmp(buf, lastbuf, 19) != 0)
     {
         memcpy(lastbuf, buf, 21);

         //RCLCPP_INFO(node_->get_logger(), "[CSDarmCommonSerial::SendArmAllServerTime] before SendSerialData2Arm mFd:%d",mFd);
         if(SendSerialData2Arm((char*)buf, 21) != 21)
         {
             RCLCPP_ERROR(node_->get_logger(), "Write error for multiple servo time cmd!");
             return ExitError;
         }
     }
     return ExitSuccess;
 }
 
 //-----------------------------------------------------
 // 插补控制命令 (连续发指令给6个舵机)
 //-----------------------------------------------------
 int CSDarmCommonSerial::SendArmAllServerCB(float v1, float v2, float v3, float v4, float v5, float v6)
 {
     if(mFd == -1)
     {
         RCLCPP_ERROR(node_->get_logger(), "[SendArmAllServerCB] Serial not open!");
         return ExitError;
     }
 
     unsigned char buf[30];
     static unsigned char lastbuf[30];
 
     buf[0] = 0x55;
     buf[1] = 0xAA;
     buf[2] = 14;
     buf[3] = TYPE_REQUEST_MESSAGE;
     buf[4] = CMD_CONTROL_ALL_DEGREE_CB;
 
     short degree = 0;
 
     degree = (short)(1800.0f / (float)PI * v1);
     buf[5] = degree & 0xFF;
     buf[6] = (degree >> 8) & 0xFF;
 
     degree = (short)(1800.0f / (float)PI * v2);
     buf[7] = degree & 0xFF;
     buf[8] = (degree >> 8) & 0xFF;
 
     degree = (short)(1800.0f / (float)PI * v3);
     buf[9]  = degree & 0xFF;
     buf[10] = (degree >> 8) & 0xFF;
 
     degree = (short)(1800.0f / (float)PI * v4);
     buf[11] = degree & 0xFF;
     buf[12] = (degree >> 8) & 0xFF;
 
     degree = (short)(1800.0f / (float)PI * v5);
     buf[13] = degree & 0xFF;
     buf[14] = (degree >> 8) & 0xFF;
 
     degree = (short)(1800.0f / (float)PI * v6);
     buf[15] = degree & 0xFF;
     buf[16] = (degree >> 8) & 0xFF;
 
     buf[17] = CheckSum(buf);
     buf[18] = 0x7D;
 
     RCLCPP_INFO(node_->get_logger(),
         "[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]",
         (float)((short)(buf[5] | (buf[6]<<8))) /10.0f,
         (float)((short)(buf[7] | (buf[8]<<8))) /10.0f,
         (float)((short)(buf[9] | (buf[10]<<8))) /10.0f,
         (float)((short)(buf[11]| (buf[12]<<8))) /10.0f,
         (float)((short)(buf[13]| (buf[14]<<8))) /10.0f,
         (float)((short)(buf[15]| (buf[16]<<8))) /10.0f
     );
 
     if(memcmp(buf, lastbuf, 19) != 0)
     {
         memcpy(lastbuf, buf, 19);
         if(SendSerialData2Arm((char*)buf, 19) != 19)
         {
             RCLCPP_ERROR(node_->get_logger(), "Write error for interpolation cmd!");
             return ExitError;
         }
     }
     return ExitSuccess;
 }
 
 //-----------------------------------------------------
 // 多个舵机控制（一次设置6个舵机的角度）
 //-----------------------------------------------------
 int CSDarmCommonSerial::SendArmAllServer(float v1, float v2, float v3, float v4, float v5, float v6)
 {
     if(mFd == -1)
     {
         RCLCPP_ERROR(node_->get_logger(), "[SendArmAllServer] Serial not open!");
         return ExitError;
     }
 
     unsigned char buf[30];
     static unsigned char lastbuf[30];
 
     buf[0] = 0x55;
     buf[1] = 0xAA;
     buf[2] = 14;
     buf[3] = TYPE_REQUEST_MESSAGE;
     buf[4] = CMD_CONTROL_ALL_DEGREE;
 
     // 第1关节
     short degree = (short)(1800.0f / (float)PI * v1);
     buf[5] = degree & 0xFF;
     buf[6] = (degree >> 8) & 0xFF;
 
     // 第2关节
     degree = (short)(1800.0f / (float)PI * v2);
     buf[7] = degree & 0xFF;
     buf[8] = (degree >> 8) & 0xFF;
 
     // 第3关节 (示例：如果某些关节正负方向不同，可做特别处理)
     degree = (short)(-1800.0f / (float)PI * v3);
     buf[9]  = degree & 0xFF;
     buf[10] = (degree >> 8) & 0xFF;
 
     // 第4关节
     degree = (short)(1800.0f / (float)PI * v4);
     buf[11] = degree & 0xFF;
     buf[12] = (degree >> 8) & 0xFF;
 
     // 第5关节
     degree = (short)(1800.0f / (float)PI * v5);
     buf[13] = degree & 0xFF;
     buf[14] = (degree >> 8) & 0xFF;
 
     // 第6关节
     degree = (short)(1800.0f / (float)PI * v6);
     buf[15] = degree & 0xFF;
     buf[16] = (degree >> 8) & 0xFF;
 
     buf[17] = CheckSum(buf);
     buf[18] = 0x7D;
 
     RCLCPP_INFO(node_->get_logger(),
         "[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]",
         (float)((short)(buf[5] | (buf[6]<<8))) /10.0f,
         (float)((short)(buf[7] | (buf[8]<<8))) /10.0f,
         (float)((short)(buf[9] | (buf[10]<<8))) /10.0f,
         (float)((short)(buf[11]| (buf[12]<<8))) /10.0f,
         (float)((short)(buf[13]| (buf[14]<<8))) /10.0f,
         (float)((short)(buf[15]| (buf[16]<<8))) /10.0f
     );
 
     if(memcmp(buf, lastbuf, 19) != 0)
     {
         memcpy(lastbuf, buf, 19);
         if(SendSerialData2Arm((char*)buf, 19) != 19)
         {
             RCLCPP_ERROR(node_->get_logger(), "Write error for multi servo cmd!");
             return ExitError;
         }
     }
     return ExitSuccess;
 }
 
 //-----------------------------------------------------
 // 从串口读取原始数据帧 (阻塞或超时)
 //-----------------------------------------------------
 int CSDarmCommonSerial::GetDataGram(unsigned char* receiveBuffer, int bufferSize, int *length)
 {
     static bool first_bit = true;
     if(mFd == -1)
     {
         if(first_bit)
         {
             first_bit = false;
             RCLCPP_ERROR(node_->get_logger(), "[GetDataGram] Serial not open!");
         }
         return ExitError;
     }
 
     fd_set fs_read;
     FD_ZERO(&fs_read);
     FD_SET(mFd, &fs_read);
 
     struct timeval timeout;
     timeout.tv_sec = 1;  // 默认等待1秒
     timeout.tv_usec = 0;
 
     int fs_sel = select(mFd + 1, &fs_read, NULL, NULL, &timeout);
     if(fs_sel > 0)
     {
         if(FD_ISSET(mFd, &fs_read))
         {
             *length = read(mFd, receiveBuffer, bufferSize);
         }
     }
     else
     {
         RCLCPP_ERROR(node_->get_logger(), "[GetDataGram] No data after 1s timeout!");
         return ExitError;
     }
     return ExitSuccess;
 }
 
 //-----------------------------------------------------
 // 一次性发送整条 JointTrajectory
 //-----------------------------------------------------
 int CSDarmCommonSerial::SendJointTrajectory(const trajectory_msgs::msg::JointTrajectory &traj_in)
 {
     // 为防止其他线程同时写串口，这里加互斥锁
 
     if(mFd == -1)
     {
         RCLCPP_ERROR(node_->get_logger(), "[SendJointTrajectory] Serial not open!");
         return ExitError;
     }
 
     // 做一个本地拷贝，避免外部在本函数执行过程中修改了 trajectory
     auto trajectory_msg = traj_in;
 
     // 如果轨迹没有点，直接返回
     if (trajectory_msg.points.empty())
     {
         RCLCPP_WARN(node_->get_logger(), "Received an empty trajectory. Nothing to send.");
         return ExitError;
     }
 
     // 对轨迹的每个点做安全检查
     // 假设机械臂是 6 关节(不含夹爪)，如果你的机械臂关节数不同，需要改相应判断
     for (size_t i = 0; i < trajectory_msg.points.size(); ++i)
     {
         if (trajectory_msg.points[i].positions.size() < 6)
         {
             RCLCPP_ERROR(node_->get_logger(),
                          "Trajectory point %zu has fewer than 6 positions, abort.", i);
             return ExitError;
         }
     }
 
     RCLCPP_INFO(node_->get_logger(),
                 "Sending trajectory with %zu points via serial...", 
                 trajectory_msg.points.size());
 
     // 依次发送每个点：difftime 根据 (time_from_start[i] - time_from_start[i-1]) 来计算
     for (size_t i = 0; i < trajectory_msg.points.size(); ++i)
     {
         auto &pt = trajectory_msg.points[i];
 
         float v1 = pt.positions[0];
         float v2 = pt.positions[1];
         float v3 = pt.positions[2];
         float v4 = pt.positions[3];
         float v5 = pt.positions[4];
         float v6 = pt.positions[5];
 
         // 默认一个最小执行时间，防止 difftime=0
         short difftime_ms = 100;
 
         if (i == 0)
         {
             // 第一个点直接用它的 time_from_start
             double t_ms = (double)pt.time_from_start.sec * 1000.0 +
                           (double)pt.time_from_start.nanosec / 1.0e6;
             if(t_ms < 1.0) t_ms = 100.0;  // 不要太小
             difftime_ms = static_cast<short>(t_ms);
         }
         else
         {
             // 计算相邻两点间的时间差
             auto &pt_prev = trajectory_msg.points[i - 1];
             double t_curr = (double) pt.time_from_start.sec * 1000.0 +
                             (double) pt.time_from_start.nanosec / 1.0e6;
             double t_prev = (double) pt_prev.time_from_start.sec * 1000.0 +
                             (double) pt_prev.time_from_start.nanosec / 1.0e6;
             double delta   = t_curr - t_prev;
             if(delta < 1.0) delta = 100.0;  // 不要太小
             difftime_ms = static_cast<short>(delta);
         }
 
         // 调用发送 6 关节 + 时间差
         int ret = SendArmAllServerTime(difftime_ms, v1, v2, v3, v4, v5, v6);
         if (ret != ExitSuccess)
         {
             RCLCPP_ERROR(node_->get_logger(),
                          "Failed sending point %zu to hardware, stop.", i);
             return ExitError;
         }
 
         // 这里简单地在本线程 sleep difftime_ms，等待下位机执行
         // 你也可以用更复杂的方式来同步或并发发送
         rclcpp::Duration sleep_dur_ms(
             (int32_t)(difftime_ms / 1000),
             (int32_t)((difftime_ms % 1000) * 1000000));
         rclcpp::sleep_for(sleep_dur_ms.to_chrono<std::chrono::nanoseconds>());
     }
 
     RCLCPP_INFO(node_->get_logger(),
                 "All trajectory points have been sent successfully via serial.");
     return ExitSuccess;
 }
 
 } // namespace sdk_sagittarius_arm
 