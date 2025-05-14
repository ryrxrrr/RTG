/*******************************************************************************
 * Software License Agreement (BSD License)
 * Copyright (c) ...
 * All rights reserved.
 *
 * 升级注记：
 *   - 增加了将关节角度存储到 current_joint_positions[] 的逻辑。
 *   - 保持原有的串口帧解析和 JointState 发布功能。
 ******************************************************************************/

 #include <sdk_sagittarius_arm/sdk_sagittarius_arm_common.hpp>

 #include <cstdio>
 #include <cstring>
 #include <sys/time.h>
 #include <unistd.h>
 
 // ROS2
 #include <rclcpp/rclcpp.hpp>
 #include <sensor_msgs/msg/joint_state.hpp>
 
 // C++ 标准库
 #include <thread>
 #include <chrono>
 
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
 #define PI 3.14159265358979323846
 #endif
 
 namespace sdk_sagittarius_arm
 {
 
 CSDarmCommon::CSDarmCommon(rclcpp::Node::SharedPtr node)
  : node_(node)
  , mDiagUpdater(
      node->get_node_base_interface(),
      node->get_node_clock_interface(),
      node->get_node_logging_interface(),
      node->get_node_parameters_interface(),
      node->get_node_timers_interface(),
      node->get_node_topics_interface(),
      1.0
  )
  , mThrcv(nullptr)
  , mDataLength(0)
 {
     // 初始化接收缓冲区
     memset(mRecvBuffer, 0, RECV_BUFFER_SIZE);
     memset(mFrameBuffer, 0, RECV_BUFFER_SIZE);
 
     // 初始化6关节位置为0
     for(int i=0; i<6; ++i){
       current_joint_positions[i] = 0.0;
     }
 
     // 创建 JointState 发布器
     mMotionPlanPub = node_->create_publisher<sensor_msgs::msg::JointState>(
         "joint_states", 10);
 
     RCLCPP_INFO(node_->get_logger(), "CSDarmCommon constructor with diagnostic_updater done!");
 }
 
 CSDarmCommon::~CSDarmCommon()
 {
     // 终止后台接收线程
     DestroyThread(&mThrcv);
     RCLCPP_INFO(node_->get_logger(), "sdk_sagittarius_arm driver exiting.");
 }
 
 //-----------------------------------------------------
 // 用于安全退出后台线程
 //-----------------------------------------------------
 bool CSDarmCommon::DestroyThread(std::thread **th)
 {
     if(*th != nullptr)
     {
         if((*th)->joinable())
         {
             (*th)->join();
         }
         delete *th;
         *th = nullptr;
         return true;
     }
     return false;
 }
 
 int CSDarmCommon::StopArmLock()
 {
     // 空实现，如果需要可以在此发送舵机解锁指令
     return 0;
 }
 
 bool CSDarmCommon::RebootDevice()
 {
     // 空实现，如果需要可在此处实现硬件复位逻辑
     return true;
 }
 
 //-----------------------------------------------------
 // 高层初始化入口：调用 InitDevice() + InitArm()
 //-----------------------------------------------------
 int CSDarmCommon::Init()
 {
     int result = InitDevice();  // 子类负责实现
     if(result != ExitSuccess)
     {
         RCLCPP_FATAL(node_->get_logger(), "Failed to init device");
         return result;
     }
 
     // 可选：对机械臂做默认锁定
     result = InitArm();
     if(result != ExitSuccess)
     {
         RCLCPP_FATAL(node_->get_logger(), "Failed to init arm");
     }
     return result;
 }
 
 int CSDarmCommon::InitArm()
 {
     // 默认锁住舵机
     SendArmLockOrFree(1);
     return ExitSuccess;
 }
 
 //-----------------------------------------------------
 // 计算校验和
 //-----------------------------------------------------
 unsigned char CSDarmCommon::CheckSum(unsigned char *buf)
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
 // 打印字节流(用于调试)
 //-----------------------------------------------------
 void CSDarmCommon::print_hex(unsigned char *buf, int len)
 {
 #if 1
     for(int i=0; i<len; i++)
     {
         printf("%02x ", buf[i]);
     }
     printf("\n");
 #endif
 }
 
 //-----------------------------------------------------
 // 启动后台串口接收线程
 //-----------------------------------------------------
 void CSDarmCommon::StartReceiveSerial()
 {
     if(mThrcv == nullptr)
     {
         mThrcv = new std::thread(&CSDarmCommon::LoopRcv, this);
     }
 }
 
 //-----------------------------------------------------
 // 后台循环
 //-----------------------------------------------------
 void CSDarmCommon::LoopRcv()
 {
     while(rclcpp::ok())
     {
         LoopOnce();
         // 如果需要做诊断更新:
         // mDiagUpdater.update();
         std::this_thread::sleep_for(std::chrono::milliseconds(1));
     }
 }
 
 //-----------------------------------------------------
 // 解析成功后，发布关节信息 + 存储到 current_joint_positions[]
 //-----------------------------------------------------
 void CSDarmCommon::PublishJointStates(unsigned char *buf)
 {
     sensor_msgs::msg::JointState joint_state;
     joint_state.header.stamp = node_->now();
 
     // 机械臂 6 关节 + 2 个爪子
     joint_state.name.resize(8);
     joint_state.position.resize(8);
 
     // 第1关节
     short raw1 = (short)(buf[0] | (buf[1]<<8));
     double joint1_rad = raw1 / 1800.0 * PI;
     joint_state.name[0]      = "joint1";
     joint_state.position[0]  = joint1_rad;
     current_joint_positions[0] = joint1_rad; // 存储
 
     // 第2关节
     short raw2 = (short)(buf[2] | (buf[3]<<8));
     double joint2_rad = raw2 / 1800.0 * PI;
     joint_state.name[1]      = "joint2";
     joint_state.position[1]  = joint2_rad;
     current_joint_positions[1] = joint2_rad;
 
     // 第3关节
     short raw3 = (short)(buf[4] | (buf[5]<<8));
     double joint3_rad = raw3 / 1800.0 * PI;
     joint_state.name[2]      = "joint3";
     joint_state.position[2]  = joint3_rad;
     current_joint_positions[2] = joint3_rad;
 
     // 第4关节
     short raw4 = (short)(buf[6] | (buf[7]<<8));
     double joint4_rad = raw4 / 1800.0 * PI;
     joint_state.name[3]      = "joint4";
     joint_state.position[3]  = joint4_rad;
     current_joint_positions[3] = joint4_rad;
 
     // 第5关节
     short raw5 = (short)(buf[8] | (buf[9]<<8));
     double joint5_rad = raw5 / 1800.0 * PI;
     joint_state.name[4]      = "joint5";
     joint_state.position[4]  = joint5_rad;
     current_joint_positions[4] = joint5_rad;
 
     // 第6关节
     short raw6 = (short)(buf[10] | (buf[11]<<8));
     double joint6_rad = raw6 / 1800.0 * PI;
     joint_state.name[5]      = "joint6";
     joint_state.position[5]  = joint6_rad;
     current_joint_positions[5] = joint6_rad;
 
     // 爪子，使用同一个值表示左右爪
     short raw_g = (short)(buf[12] | (buf[13]<<8));
     double gripper_val = -(raw_g) * 0.026 / 900.0;
 
     joint_state.name[6]     = "joint_gripper_left";
     joint_state.position[6] = gripper_val;
     joint_state.name[7]     = "joint_gripper_right";
     joint_state.position[7] = gripper_val;
 
     // 发布
     mMotionPlanPub->publish(joint_state);
 }
 
 //-----------------------------------------------------
 // 循环读串口 -> 若帧完整则解析
 //-----------------------------------------------------
 int CSDarmCommon::LoopOnce()
{
    // 1) 从串口读取数据
    int dataLength = 0;
    int result = GetDataGram(mRecvBuffer, RECV_BUFFER_SIZE, &dataLength);
    if (result != ExitSuccess)
    {
        RCLCPP_WARN(node_->get_logger(), 
                    "[LoopOnce] Failed to read from serial (error code: %d).", result);
        return ExitError;
    }

    // 2) 若本次读取到字节数为0，则直接返回
    if (dataLength <= 0)
    {
        // 如果想要调试可以打更详细的日志，这里仅略作提示
        // RCLCPP_DEBUG(node_->get_logger(), "[LoopOnce] Read 0 bytes this round.");
        return ExitSuccess;
    }

    // 3) 将本次读取的字节追加到 mFrameBuffer
    if ((dataLength < 255) && (mDataLength < 255))
    {
        // 拷贝到尾部
        memcpy(mFrameBuffer + mDataLength, mRecvBuffer, dataLength);
        mDataLength += dataLength;

        // 若帧头不等于 0x55，则说明不是我们关心的帧结构，清空
        if (mFrameBuffer[0] != 0x55)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "[LoopOnce] Invalid first byte (0x%02X). Discarding buffer.",
                        (unsigned int)mFrameBuffer[0]);
            mDataLength = 0;
            return ExitSuccess;
        }
    }
    else
    {
        // 如果 dataLength 或 mDataLength 超过范围，就清空
        RCLCPP_WARN(node_->get_logger(),
                    "[LoopOnce] dataLength=%d or mDataLength=%d is too large. Discarding buffer.",
                    dataLength, mDataLength);
        mDataLength = 0;
        return ExitSuccess;
    }

    // 4) 判断是否已收满一帧
    //    先判断 mDataLength 是否足够读到至少 3 个字节(header + DataLen)
    //    这里 mFrameBuffer[2] 代表 DataLen
    if (mDataLength > 3)
    {
        // 取出 DataLen
        int data_len = static_cast<int>(mFrameBuffer[2]);

        // 如果 DataLen 无效(过大或过小)，直接丢掉
        if (data_len < 0 || data_len > 100)  // 比如我们支持最大DataLen=100
        {
            RCLCPP_WARN(node_->get_logger(),
                        "[LoopOnce] DataLen(%d) out of valid range. Discarding buffer.",
                        data_len);
            print_hex(mFrameBuffer, mDataLength);
            mDataLength = 0;
            return ExitSuccess;
        }

        // 如果当前接收的总长度 >= (DataLen + 5)，说明本帧完整
        // 结构: [0]=0x55, [1]=0xAA, [2]=DataLen, 后面 DataLen字节, [CRC], [0x7D]
        // 最小长度: 3(头+DataLen字段) + DataLen + 2(校验+帧尾)
        if (mDataLength >= (data_len + 5))
        {
            // 5) 校验帧头、帧尾、校验和
            bool valid_frame = 
                (mFrameBuffer[0] == 0x55) &&
                (mFrameBuffer[1] == 0xAA) &&
                (mFrameBuffer[data_len + 4] == 0x7D) &&
                (CheckSum(mFrameBuffer) == (unsigned char)mFrameBuffer[data_len + 3]);

            if (valid_frame)
            {
                // 根据命令类型 mFrameBuffer[4] 做处理
                // -------------------------------
                if (mFrameBuffer[4] == 0x0A)
                {
                    RCLCPP_INFO(node_->get_logger(), "升级命令 (cmd=0x0A)");
                }
                else if (mFrameBuffer[4] == 0x09)
                {
                    if (mFrameBuffer[3] == 0x02)
                    {
                        RCLCPP_INFO(node_->get_logger(),
                                    "version is %s", (char*)(mFrameBuffer+5));
                    }
                }
                else if (mFrameBuffer[4] == 0x06)
                {
                    // 0x06 => 关节反馈
                    if (mFrameBuffer[3] == 0x01)
                    {
                        PublishJointStates(mFrameBuffer + 5);
                    }
                }
                else if (mFrameBuffer[4] == CMD_GET_SERVO_RT_INFO)
                {
                    // 舵机实时信息返回
                    if (mFrameBuffer[3] == 0x02)
                    {
                        RCLCPP_INFO(node_->get_logger(),
                                    "Servo response for CMD_GET_SERVO_RT_INFO");
                        servo_state.servo_id = mFrameBuffer[5];
                        servo_state.speed    = (mFrameBuffer[6]  | (mFrameBuffer[7] << 8));
                        servo_state.payload  = (mFrameBuffer[8]  | (mFrameBuffer[9] << 8));
                        servo_state.voltage  =  mFrameBuffer[10];
                        servo_state.current  = (mFrameBuffer[11] | (mFrameBuffer[12]<<8));
                        servo_state.flag     = 1;
                    }
                }
                else
                {
                    RCLCPP_INFO(node_->get_logger(),
                                "其它命令: 0x%02X", mFrameBuffer[4]);
                }
            }
            else
            {
                // 如果校验不通过，则打印 warning
                RCLCPP_WARN(node_->get_logger(), "[LoopOnce] Frame check failed!");
                print_hex(mFrameBuffer, mDataLength);
            }

            // 不管帧对错，都清空缓存，等待下一帧
            mDataLength = 0;
        }
        // else 说明数据还没收够一帧，继续等待
    }

    return ExitSuccess;
 }
 
}