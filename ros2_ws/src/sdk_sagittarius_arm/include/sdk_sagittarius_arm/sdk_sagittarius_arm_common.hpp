/*******************************************************************************
 * Software License Agreement (BSD License)
 * Copyright (c) ...
 * All rights reserved.
 *
 * 升级注记：
 *  1. 在基类中增加了 `current_joint_positions[6]` 数组，用于存储当前机械臂 6 个关节的实际弧度。
 *  2. 保留原有的纯虚函数接口，让派生类（如 CSDarmCommonSerial）实现具体串口操作。
 *  3. (可选) 增加一个虚函数: SendJointTrajectory()，用于一次性发送整条轨迹。
 ******************************************************************************/

 #ifndef SDK_ARM_COMMON_H_
 #define SDK_ARM_COMMON_H_
 
 #include <cstdio>
 #include <cstdlib>
 #include <string>
 #include <cstring>
 #include <vector>
 #include <thread>   // 用 std::thread 替代 boost::thread
 #include <atomic>
 
 // ROS2 头
 #include <rclcpp/rclcpp.hpp>
 #include <sensor_msgs/msg/joint_state.hpp>
 #include <diagnostic_updater/diagnostic_updater.hpp>
 #include <trajectory_msgs/msg/joint_trajectory.hpp>
 
 // 该项目中的一些常量或定义
 #include "sdk_sagittarius_arm_constants.h"
 
 #ifndef RECV_BUFFER_SIZE
 #define RECV_BUFFER_SIZE 1024
 #endif
 
 // 用于存储某个舵机的实时状态(比如 speed/voltage/current 等)
 typedef struct
 {
     uint8_t flag;
     uint8_t servo_id;
     int16_t speed;
     int16_t payload;
     uint8_t voltage;
     uint16_t current;
 } Str_ServoState;
 
 namespace sdk_sagittarius_arm
 {
 
 /**
  * @brief CSDarmCommon: 一个抽象基类，封装了机器人臂的共性操作和ROS相关逻辑。
  *        具体的硬件通讯（如串口）在派生类中实现 (CSDarmCommonSerial)。
  */
 class CSDarmCommon
 {
 public:
     /**
      * @brief 构造函数
      * @param node  ROS2节点指针，用于日志、话题发布等
      */
     explicit CSDarmCommon(rclcpp::Node::SharedPtr node);
 
     virtual ~CSDarmCommon();
 
     /**
      * @brief 通用的初始化流程：InitDevice() + InitArm()
      * @return 0表示成功，否则失败
      */
     virtual int Init();
 
     /**
      * @brief 主要的串口循环入口，读一次数据。
      * @return 0表示成功，-1表示失败
      */
     int LoopOnce();
 
     /**
      * @brief 启动后台接收线程，不停调用LoopOnce()
      */
     void StartReceiveSerial();
 
     /**
      * @brief 解析关节反馈帧后，发布为 ROS 的 JointState 消息，同时更新 current_joint_positions[]
      * @param buf  指向数据帧里存放角度的位置
      */
     void PublishJointStates(unsigned char *buf);
 
     /**
      * @brief 重启设备（可在子类中实现具体逻辑）。这里是空实现。
      * @return 默认true
      */
     virtual bool RebootDevice();
 
     /**
      * @brief 当前查询到的舵机状态
      */
     Str_ServoState servo_state;
 
     /**
      * @brief 存储机械臂 6 个关节的实际弧度 (由 PublishJointStates() 填充)
      *        若你的机器人是 7 关节，可自行改为7；或扩展更多。
      */
     double current_joint_positions[6];
 
     //====================================================
     // 以下纯虚函数，子类必须实现
     //====================================================
     virtual int InitDevice() = 0;  ///< 打开硬件(串口等)
     virtual int CloseDevice() = 0; ///< 关闭硬件
 
     // 基本控制指令
     virtual int SendArmAllServer(float v1, float v2, float v3, float v4, float v5, float v6) = 0;
     virtual int SendArmAllServerTime(short difftime, float v1, float v2, float v3, float v4, float v5, float v6)= 0;
     virtual int SendArmAllServerCB(float v1, float v2, float v3, float v4, float v5, float v6) = 0;
     virtual int SendArmEndAction(unsigned char onoff, short value) = 0;
     virtual int SendArmLockOrFree(unsigned char onoff) = 0;
 
     // 设置舵机属性
     virtual int SetArmVel(unsigned short vel) = 0;
     virtual int SetArmAcc(unsigned char acc)  = 0;
     virtual int SetArmTorque(int torque[])     = 0;
 
     // 串口发送/读取
     virtual int SendSerialData2Arm(char *buf, int length) = 0;
     virtual int GetDataGram(unsigned char* receiveBuffer, int bufferSize, int *length) = 0;
     virtual unsigned char CheckSum(unsigned char *buf) = 0;
 
     virtual int SendGetServoRealTimeInfo(unsigned char id) = 0;
 
     //====================================================
     // (可选) 一次性发送整条关节轨迹
     //====================================================
     /**
      * @brief 默认实现(或纯虚函数)；子类 CSDarmCommonSerial 可重载
      */
     virtual int SendJointTrajectory(const trajectory_msgs::msg::JointTrajectory &trajectory_msg)
     {
         RCLCPP_WARN(node_->get_logger(),
                     "[CSDarmCommon] Base class does not implement SendJointTrajectory!");
         return -1;
     }
 
 protected:
     /**
      * @brief 初始化机械臂（缺省实现为锁住舵机）
      * @return 0表示成功
      */
     virtual int InitArm();
 
     /**
      * @brief 停止舵机锁的操作 (空实现)
      */
     virtual int StopArmLock();
 
     /**
      * @brief 打印十六进制，用于调试
      */
     virtual void print_hex(unsigned char *buf, int len);
 
     /**
      * @brief 后台循环，持续调用LoopOnce()
      */
     void LoopRcv();
 
     /**
      * @brief 安全销毁后台线程
      * @param th 指向要销毁的线程指针
      * @return true 如果销毁成功
      */
     bool DestroyThread(std::thread **th);
 
     // ROS2 Node
     rclcpp::Node::SharedPtr node_;
 
     // ROS2 诊断器
     diagnostic_updater::Updater mDiagUpdater;
 
     // 后台读取线程
     std::thread *mThrcv;
 
     // 串口缓冲
     unsigned char mRecvBuffer[RECV_BUFFER_SIZE];
     unsigned char mFrameBuffer[RECV_BUFFER_SIZE];
     int           mDataLength;
 
     // 用于发布 JointState
     rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mMotionPlanPub;
 };
 
 } // namespace sdk_sagittarius_arm
 
 #endif // SDK_ARM_COMMON_H_
 