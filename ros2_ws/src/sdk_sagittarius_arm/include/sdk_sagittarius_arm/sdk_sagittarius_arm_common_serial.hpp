/*******************************************************************************
 * Software License Agreement (BSD License)
 * Copyright (c) ...
 * All rights reserved.
 *
 * 升级注记：
 *  1. 派生自基类 CSDarmCommon，专门实现串口通信的细节 (InitDevice / GetDataGram / SendSerialData2Arm 等)。
 *  2. 保留原有接口函数，只做注释和风格上的微调。
 *  3. 在此声明 SendJointTrajectory(...)，由 .cpp 中实现。
 ******************************************************************************/

 #ifndef SDK_SAGITTARIUS_ARM_COMMON_SERIAL_HPP_
 #define SDK_SAGITTARIUS_ARM_COMMON_SERIAL_HPP_
 
 #include <string>
 #include <rclcpp/rclcpp.hpp>
 #include <trajectory_msgs/msg/joint_trajectory.hpp>
 
 #include <sdk_sagittarius_arm/sdk_sagittarius_arm_common.hpp>  // 基类 CSDarmCommon
 
 namespace sdk_sagittarius_arm
 {
 
 /**
  * @brief CSDarmCommonSerial: 使用串口通信的实现类，继承自 CSDarmCommon(ROS2)
  */
 class CSDarmCommonSerial : public CSDarmCommon
 {
 public:
     /**
      * @brief 构造函数
      * @param serialname     串口设备名 (e.g. "/dev/ttyUSB0")
      * @param baudrate       波特率 (字符串 -> int)
      * @param timelimit      超时时间(秒) 或其他含义
      * @param node           rclcpp::Node::SharedPtr (用来记录日志、创建定时器等)
      * @param free_torque    程序结束时是否释放舵机扭矩
      */
     CSDarmCommonSerial(
         const std::string &serialname,
         const std::string &baudrate,
         int timelimit,
         rclcpp::Node::SharedPtr node,
         bool free_torque
     );
 
     virtual ~CSDarmCommonSerial();
 
 protected:
     //=====================================
     //  实现基类的纯虚函数 (常规动作/控制)
     //=====================================
     virtual int InitDevice() override;    ///< 打开串口
     virtual int CloseDevice() override;   ///< 关闭串口
 
     virtual int SendArmAllServer(float v1, float v2, float v3, float v4, float v5, float v6) override;
     virtual int SendArmAllServerTime(short difftime, float v1, float v2, float v3, float v4, float v5, float v6) override;
     virtual int SendArmAllServerCB(float v1, float v2, float v3, float v4, float v5, float v6) override;
     virtual int SendArmLockOrFree(unsigned char onoff) override;
     virtual int SendArmEndAction(unsigned char onoff, short value) override;
     virtual int SetArmVel(unsigned short vel) override;
     virtual int SetArmAcc(unsigned char acc) override;
     virtual int SetArmTorque(int torque[]) override;
     virtual int SendGetServoRealTimeInfo(unsigned char id) override;
 
     //=====================================
     //  实现基类的纯虚函数 (底层串口读写)
     //=====================================
     virtual int GetDataGram(unsigned char *receiveBuffer, int bufferSize, int *length) override;
     virtual int SendSerialData2Arm(char *buf, int length) override;
     virtual unsigned char CheckSum(unsigned char *buf) override;
 
     //=====================================
     //  新增：一次性发送 JointTrajectory
     //=====================================
     virtual int SendJointTrajectory(
         const trajectory_msgs::msg::JointTrajectory &trajectory_msg
     ) override;
 
 private:
     int         mFd;              ///< 串口文件描述符
     size_t      mBytesReceived;   ///< 已接收字节数(可选)
     std::string mSerialName;      ///< 串口名
     int         mBaudrate;        ///< 波特率
     int         mTimeLimit;       ///< 超时时间
     bool        mExitFreeTorque;  ///< 在析构时是否释放舵机扭矩
 };
 
 } // namespace sdk_sagittarius_arm
 
 #endif // SDK_SAGITTARIUS_ARM_COMMON_SERIAL_HPP_
 