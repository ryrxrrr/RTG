/*******************************************************************************
 * Software License Agreement (BSD License)
 * Copyright (c) ...
 * All rights reserved.
 *
 * 升级注记：
 *  1. 在 publish_current_joint_states() 中，读取 pSDKarm->current_joint_positions[] 并发布，
 *     避免出现关节角为 0 的问题。
 *  2. 其余逻辑尽量保持与原版一致。
 ******************************************************************************/

 #include <sdk_sagittarius_arm/sdk_sagittarius_arm_real.hpp>
 #include <unistd.h>      // sleep, usleep
 #include <yaml-cpp/yaml.h>
 #include <thread>
 #include <chrono>
 
 #ifndef PI
 #define PI 3.14159265359
 #endif
 
 namespace sdk_sagittarius_arm
 {
 
 SagittariusArmReal::SagittariusArmReal(const rclcpp::NodeOptions &options)
 : Node("sdk_sagittarius_arm_real", options),
   execute_joint_traj(false),
   execute_gripper_traj(false),
   torque_status(true),
   joint_num_write(0),
   rviz_control(false),
   servo_control_trajectory(false),
   joint_start_time(0.0),
   gripper_start_time(0.0),
   robot_name_("sagittarius_arm"),
   robot_model_("sagittarius_arm_model"),
   pSDKarm(nullptr),
   pTest(nullptr),
   joint_ids_read(nullptr),
   joint_ids_write(nullptr)
 {
   //====================
   // 1) 声明 & 获取参数
   //====================
   this->declare_parameter<bool>("just_rviz_control", false);
   this->declare_parameter<bool>("servo_control_trajectory", false);
   this->declare_parameter<bool>("exit_free_torque", false);
   this->declare_parameter<std::string>("serialname", "/dev/ttyACM0");
   this->declare_parameter<std::string>("baudrate", "115200");
   this->declare_parameter<std::string>("servo_configs", "/home/xxx/servo_configs/");
   this->declare_parameter<int>("timelimit", 5);
   this->declare_parameter<int>("arm_velocity", 1000);
   this->declare_parameter<int>("arm_acceleration", 0);
   for(int i=1; i<=7; i++)
   {
     std::string pname = "servo_torque" + std::to_string(i);
     this->declare_parameter<int>(pname, 1000);
   }
 
   bool tmp_exit_free_torque = false;
   std::string tmp_serial_name, tmp_baudrate;
   int tmp_time_limit = 5;
 
   this->get_parameter("just_rviz_control", rviz_control);
   this->get_parameter("servo_control_trajectory", servo_control_trajectory);
   this->get_parameter("exit_free_torque", tmp_exit_free_torque);
   this->get_parameter("serialname", tmp_serial_name);
   this->get_parameter("baudrate", tmp_baudrate);
   this->get_parameter("timelimit", tmp_time_limit);
 
   // 将这些暂存到类成员 (给 onInit() 用)
   exit_free_torque_ = tmp_exit_free_torque;
   serial_name_      = tmp_serial_name;
   baudrate_         = tmp_baudrate;
   iTimeLimit_       = tmp_time_limit;
 
   RCLCPP_INFO(this->get_logger(), "rviz_control: %d", rviz_control);
   RCLCPP_INFO(this->get_logger(), "servo_control_trajectory: %d", servo_control_trajectory);
   RCLCPP_INFO(this->get_logger(), "exit_free_torque: %d", exit_free_torque_);
   RCLCPP_INFO(this->get_logger(), "serialname: %s, baudrate: %s",
               serial_name_.c_str(), baudrate_.c_str());
 
   //====================
   // 2) 获取 servo 配置
   //====================
   arm_get_servo_configs();  // 在构造函数中即可
 
   //====================
   // 3) 创建基础服务 & 订阅 
   //====================
   srv_get_robot_info_ = this->create_service<srv::ArmInfo>(
       "get_robot_info",
       [this](const std::shared_ptr<srv::ArmInfo::Request> req,
              std::shared_ptr<srv::ArmInfo::Response> res)
       {
         bool ok = this->arm_get_robot_info(req, res);
         (void)ok;
       });
 
   srv_get_servo_info_ = this->create_service<srv::ServoRtInfo>(
       "get_servo_info",
       [this](const std::shared_ptr<srv::ServoRtInfo::Request> req,
              std::shared_ptr<srv::ServoRtInfo::Response> res)
       {
         bool ok = this->arm_get_servo_info(req, res);
         (void)ok;
       });
 
   // 控制扭矩
   sub_ct_ = this->create_subscription<std_msgs::msg::String>(
       "control_torque", 1,
       std::bind(&SagittariusArmReal::ControlTorque, this, std::placeholders::_1));
 
   // 若是 rviz_control，则从 /joint_states 订阅外部指令
   if(rviz_control)
   {
     sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
         "joint_states", 1,
         std::bind(&SagittariusArmReal::JointStatesCb, this, std::placeholders::_1));
   }
 
   // 其它：关节命令
   sub_joint_commands_ = this->create_subscription<sdk_sagittarius_arm::msg::ArmRadControl>(
       "joint/commands", 100,
       [this](sdk_sagittarius_arm::msg::ArmRadControl::SharedPtr msg)
       {
         if (msg) { this->arm_write_joint_commands(*msg); }
       });
 
   // 爪子的单轴
   sub_gripper_command_ = this->create_subscription<std_msgs::msg::Float64>(
       "gripper/command", 100,
       [this](std_msgs::msg::Float64::SharedPtr msg)
       {
         if (msg) { this->arm_write_gripper_command(*msg); }
       });
 
   RCLCPP_INFO(this->get_logger(), "[Constructor] Done. Please call onInit() next.");
 }
 
 SagittariusArmReal::~SagittariusArmReal()
 {
   if(pSDKarm) {
     delete pSDKarm;
     pSDKarm = nullptr;
   }
   if(joint_ids_write) {
     delete [] joint_ids_write;
     joint_ids_write = nullptr;
   }
   RCLCPP_INFO(this->get_logger(), "SagittariusArmReal destructor.");
 }
 
 //--------------------------------------
 //    延后初始化，避免 bad_weak_ptr
 //--------------------------------------
 void SagittariusArmReal::onInit()
 {
   auto self_shared = this->shared_from_this();
 
   // 1) 创建 SDK 实例
   pSDKarm = new CSDarmCommonSerial(
       serial_name_,
       baudrate_,
       iTimeLimit_,
       self_shared,  
       exit_free_torque_
   );
 
   int result = pSDKarm->Init();
   if(result != 0)
   {
     RCLCPP_ERROR(this->get_logger(), "pSDKarm->Init() failed!");
   }
 
   // 2) 若不是 rviz_control，则启动: 串口接收 + 定时器 + ActionServer
   if(!rviz_control)
   {
     pSDKarm->StartReceiveSerial();
 
     // 定时器: joint 轨迹
     tmr_joint_traj_ = this->create_wall_timer(
         std::chrono::milliseconds(10),
         std::bind(&SagittariusArmReal::arm_execute_joint_trajectory, this));
 
     // 定时器: gripper 轨迹
     tmr_gripper_traj_ = this->create_wall_timer(
         std::chrono::milliseconds(10),
         std::bind(&SagittariusArmReal::arm_execute_gripper_trajectory, this));
 
     // Action Server
     joint_action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
       self_shared,
       "sagittarius_arm_controller/follow_joint_trajectory",
       std::bind(&SagittariusArmReal::handle_joint_traj_goal, this,
                 std::placeholders::_1, std::placeholders::_2),
       std::bind(&SagittariusArmReal::handle_joint_traj_cancel, this,
                 std::placeholders::_1),
       std::bind(&SagittariusArmReal::handle_joint_traj_accepted, this,
                 std::placeholders::_1)
     );
 
     gripper_action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
       self_shared,
       "sagittarius_gripper_controller/follow_joint_trajectory",
       std::bind(&SagittariusArmReal::handle_gripper_traj_goal, this,
                 std::placeholders::_1, std::placeholders::_2),
       std::bind(&SagittariusArmReal::handle_gripper_traj_cancel, this,
                 std::placeholders::_1),
       std::bind(&SagittariusArmReal::handle_gripper_traj_accepted, this,
                 std::placeholders::_1)
     );
   }
 
   // 3) 初始化舵机速度/加速度/力矩
   GetAndSetServoVelocity(pSDKarm);
   std::this_thread::sleep_for(std::chrono::seconds(2));
 
   GetAndSetServoAcceleration(pSDKarm);
   std::this_thread::sleep_for(std::chrono::seconds(2));
 
   GetAndSetServoTorque(pSDKarm);
   std::this_thread::sleep_for(std::chrono::seconds(2));
 
   // 4) 若不是 rviz_control，则检查关节越界
   if(!rviz_control)
   {
     std::string yaml_file;
     this->get_parameter("servo_configs", yaml_file);
 
     yaml_file += robot_model_ + ".yaml";
     YAML::Node sag_config = YAML::LoadFile(yaml_file.c_str());
     if (sag_config.IsNull())
     {
       RCLCPP_ERROR(this->get_logger(), "Config file not found: %s", yaml_file.c_str());
     }
     else
     {
       RCLCPP_INFO(this->get_logger(), "You might want to check servo states here...");
     }
   }
 
   // ++++++【新增】关节状态发布 +++++++ 
   publisher_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>(
       "/joint_states", 10);
 
   // 这里设为 20Hz (50ms 发布一次)
   timer_joint_state_pub_ = this->create_wall_timer(
       std::chrono::milliseconds(50),
       std::bind(&SagittariusArmReal::publish_current_joint_states, this)
   );
 
   RCLCPP_INFO(this->get_logger(), "[onInit] Done. Node fully initialized.");
 }
 
 //===============================
 //   获取 servo 配置
 //===============================
 void SagittariusArmReal::arm_get_servo_configs()
 {
   std::string yaml_file;
   if (!this->get_parameter("servo_configs", yaml_file))
   {
     RCLCPP_ERROR(this->get_logger(),
                  "No 'servo_configs' param found, using fallback or aborting...");
     return;
   }
   if (!yaml_file.empty() && yaml_file.back() != '/')
     yaml_file.push_back('/');
 
   yaml_file += robot_model_ + ".yaml";
   YAML::Node sag_config = YAML::LoadFile(yaml_file.c_str());
   if (sag_config.IsNull())
   {
     RCLCPP_ERROR(this->get_logger(),
                  "Config file not found or invalid: %s", yaml_file.c_str());
     return;
   }
 
   YAML::Node sleep_node = sag_config["sleep"];
   for (auto const &value : sleep_node)
   {
     sleep_positions.push_back(value.as<double>());
     home_positions.push_back(0.0);
   }
 
   YAML::Node order_node   = sag_config["order"];
   YAML::Node singles_node = sag_config["singles"];
   std::vector<YAML::Node> nodes{order_node, singles_node};
 
   joint_num_write = 0;
   for (auto const &node : nodes)
   {
     for(size_t i = 0; i < node.size(); i++)
     {
       std::string name = node[i].as<std::string>();
       YAML::Node item = sag_config[name];
       int32_t id = item["ID"].as<int32_t>();
       int32_t secondary_id = item["Secondary_ID"].as<int32_t>();
 
       if (node == order_node && name != "joint_gripper_left" && secondary_id == 255)
       {
         joint_num_write++;
       }
     }
   }
 
   joint_ids_write = new uint8_t[joint_num_write];
   size_t cntr = 0;
 
   for (auto const &node : nodes)
   {
     for (size_t i = 0; i < node.size(); i++)
     {
       std::string name = node[i].as<std::string>();
       YAML::Node item  = sag_config[name];
       int32_t id       = item["ID"].as<int32_t>();
       int32_t secondary_id = item["Secondary_ID"].as<int32_t>();
 
       for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); ++it_item)
       {
         std::string item_name = it_item->first.as<std::string>();
         int32_t value = it_item->second.as<int32_t>();
         if (item_name == "ID") continue;
 
         if (item_name == "Secondary_ID" && value == 255)
         {
           Servo joint = {name, static_cast<uint8_t>(id)};
           all_joints.push_back(joint);
 
           if (node == order_node)
           {
             arm_joints.push_back(joint);
             if (name != "joint_gripper_left")
             {
               joint_ids_write[cntr] = static_cast<uint8_t>(id);
               cntr++;
             }
           }
         }
       }
     }
   }
 }
 
 //===============================
 //   Servo速度/加速度/力矩
 //===============================
 bool SagittariusArmReal::GetAndSetServoVelocity(CSDarmCommon *pt)
 {
   int arm_vel = 1000;
   this->get_parameter("arm_velocity", arm_vel);
   RCLCPP_INFO(this->get_logger(), "arm_vel is %d", arm_vel);
   if(pt)
   {
     pt->SetArmVel(static_cast<unsigned short>(arm_vel));
   }
   return true;
 }
 
 bool SagittariusArmReal::GetAndSetServoAcceleration(CSDarmCommon *pt)
 {
   int arm_acc = 0;
   this->get_parameter("arm_acceleration", arm_acc);
   RCLCPP_INFO(this->get_logger(), "arm_acceleration is %d", arm_acc);
   if(pt)
   {
     pt->SetArmAcc(static_cast<unsigned char>(arm_acc));
   }
   return true;
 }
 
 bool SagittariusArmReal::GetAndSetServoTorque(CSDarmCommon *pt)
 {
   int arm_torque[7];
   for(int i=1; i<=7; i++)
   {
     std::string pname = "servo_torque" + std::to_string(i);
     this->get_parameter(pname, arm_torque[i-1]);
     RCLCPP_INFO(this->get_logger(), "%s: %d", pname.c_str(), arm_torque[i-1]);
   }
   if(pt)
   {
     pt->SetArmTorque(arm_torque);
   }
   return true;
 }
 
 //===============================
 //     服务回调
 //===============================
 bool SagittariusArmReal::arm_get_servo_info(
   const std::shared_ptr<srv::ServoRtInfo::Request> request,
   std::shared_ptr<srv::ServoRtInfo::Response>      response)
 {
   if (!request || !pSDKarm) {
     RCLCPP_ERROR(this->get_logger(), "arm_get_servo_info: invalid request or pSDKarm null.");
     return false;
   }
   if (request->servo_id <= 0)
   {
     RCLCPP_ERROR(this->get_logger(), "the servo id must be more than 0");
     return false;
   }
 
   pSDKarm->servo_state.flag = 0;
   pSDKarm->SendGetServoRealTimeInfo(request->servo_id);
 
   int t_cnt = 500;
   while ((pSDKarm->servo_state.flag == 0) && (t_cnt--))
   {
     std::this_thread::sleep_for(std::chrono::milliseconds(2));
   }
 
   if(pSDKarm->servo_state.flag)
   {
     response->speed    = pSDKarm->servo_state.speed;
     response->voltage  = pSDKarm->servo_state.voltage;
     response->current  = pSDKarm->servo_state.current;
     response->payload  = pSDKarm->servo_state.payload;
     return true;
   }
   else
   {
     RCLCPP_ERROR(this->get_logger(), "get the servo state: timeout");
     return false;
   }
 }
 
 bool SagittariusArmReal::arm_get_robot_info(
   const std::shared_ptr<srv::ArmInfo::Request>  request,
   std::shared_ptr<srv::ArmInfo::Response>       response)
 {
   (void)request; // 未用
 
   // 纯示例: 假定没有URDF
   bool have_urdf = false;
   if (!have_urdf)
   {
     RCLCPP_ERROR(this->get_logger(), "No URDF found. (dummy code)");
     return false;
   }
 
   for (auto const &joint : all_joints)
   {
     if (joint.name == "joint_gripper_left")
     {
       response->use_gripper = true;
     }
     response->joint_names.push_back(joint.name);
     response->joint_ids.push_back(joint.servo_id);
   }
 
   response->home_pos          = home_positions;
   response->sleep_pos         = sleep_positions;
   response->num_joints        = joint_num_write;
   response->num_single_joints = all_joints.size();
   return true;
 }
 
 //===============================
 //   订阅回调
 //===============================
 void SagittariusArmReal::ControlTorque(const std_msgs::msg::String::SharedPtr msg)
 {
   if(!msg) return;
   RCLCPP_INFO(this->get_logger(), "ControlTorque: %s", msg->data.c_str());
   if (msg->data == "open")
   {
     torque_status = true;
     if(pSDKarm) pSDKarm->SendArmLockOrFree(1);
   }
   else
   {
     torque_status = false;
     if(pSDKarm) pSDKarm->SendArmLockOrFree(0);
   }
 }
 
 void SagittariusArmReal::JointStatesCb(const sensor_msgs::msg::JointState::SharedPtr cmd_arm)
 {
   // 只在 rviz_control==true 时，才用此订阅器来控制手臂
   if(!cmd_arm || !pSDKarm) return;
   if (cmd_arm->position.size() < 6) return;
 
   angle[0] = cmd_arm->position[0];
   angle[1] = cmd_arm->position[1];
   angle[2] = cmd_arm->position[2];
   angle[3] = cmd_arm->position[3];
   angle[4] = cmd_arm->position[4];
   angle[5] = cmd_arm->position[5];
 
   if(torque_status)
   {
     pSDKarm->SendArmAllServerCB(angle[0], angle[1], angle[2], angle[3], angle[4], angle[5]);
     // 如果消息里还有第7个位置，就当做爪子的开合
     if(cmd_arm->position.size() > 6)
     {
       arm_set_gripper_linear_position(cmd_arm->position[6] * 2.0);
     }
   }
 }
 
 //===============================
 //   轨迹消息回调 (ActionServer会调用)
 //===============================
 void SagittariusArmReal::arm_joint_trajectory_msg_callback(
     const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
 {
   if(!msg) return;
   if (!execute_joint_traj)
   {
     jnt_tra_msg = *msg; // 拷贝
     RCLCPP_INFO(this->get_logger(), "Got a new joint trajectory!");
     joint_start_time   = this->now().seconds();
     execute_joint_traj = true;
   }
   else
   {
     RCLCPP_WARN(this->get_logger(), "Arm is still moving, ignoring new trajectory!");
   }
 }
 
 void SagittariusArmReal::arm_gripper_trajectory_msg_callback(
     const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
 {
   if(!msg) return;
   if (!execute_gripper_traj)
   {
     gripper_tra_msg = *msg;
     RCLCPP_INFO(this->get_logger(), "Got a new gripper trajectory!");
     gripper_start_time   = this->now().seconds();
     execute_gripper_traj = true;
   }
   else
   {
     RCLCPP_WARN(this->get_logger(), "Gripper is still moving, ignoring new gripper trajectory!");
   }
 }
 
 //===============================
 //   ActionServer: JointTrajectory
 //===============================
 rclcpp_action::GoalResponse SagittariusArmReal::handle_joint_traj_goal(
   const rclcpp_action::GoalUUID &uuid,
   std::shared_ptr<const FollowJointTrajectory::Goal> goal)
 {
   RCLCPP_INFO(this->get_logger(), "[handle_joint_traj_goal] New goal request");
   (void)uuid;
   if (!goal) {
     return rclcpp_action::GoalResponse::REJECT;
   }
   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
 }
 
 rclcpp_action::CancelResponse SagittariusArmReal::handle_joint_traj_cancel(
   const std::shared_ptr<GoalHandleFollowJointTraj> goal_handle)
 {
   RCLCPP_INFO(this->get_logger(), "[handle_joint_traj_cancel] Cancel request");
   (void)goal_handle;
   execute_joint_traj = false;
   return rclcpp_action::CancelResponse::ACCEPT;
 }
 
 void SagittariusArmReal::handle_joint_traj_accepted(
   const std::shared_ptr<GoalHandleFollowJointTraj> goal_handle)
 {
   RCLCPP_INFO(this->get_logger(), "[handle_joint_traj_accepted] Executing goal...");
   std::thread(
     [this, goal_handle]()
     {
       this->arm_joint_trajectory_action_callback(goal_handle);
     }
   ).detach();
 }
 
 void SagittariusArmReal::arm_joint_trajectory_action_callback(
   std::shared_ptr<GoalHandleFollowJointTraj> goal_handle)
 {
   if (!goal_handle) return;
   auto goal   = goal_handle->get_goal();
   auto result = std::make_shared<FollowJointTrajectory::Result>();
 
   if (goal->trajectory.points.size() < 2)
   {
     result->error_code = result->INVALID_GOAL;
     goal_handle->abort(result);
     return;
   }
   RCLCPP_INFO(this->get_logger(), "[arm_joint_trajectory_action_callback] receive goal->trajectory.points.size():%ld", goal->trajectory.points.size());
 
   auto traj_ptr = std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal->trajectory);
   this->arm_joint_trajectory_msg_callback(traj_ptr);
 
   while(rclcpp::ok() && execute_joint_traj)
   {
     if(goal_handle->is_canceling())
     {
       execute_joint_traj = false;
       goal_handle->canceled(result);
       RCLCPP_INFO(this->get_logger(), "Joint trajectory canceled by client");
       return;
     }
     std::this_thread::sleep_for(std::chrono::milliseconds(10));
   }

   std::this_thread::sleep_for(std::chrono::milliseconds(200));
 
   result->error_code = result->SUCCESSFUL;
   goal_handle->succeed(result);
   RCLCPP_INFO(this->get_logger(), "Joint trajectory done, succeed!");
 }
 
 //===============================
 //   ActionServer: Gripper
 //===============================
 rclcpp_action::GoalResponse SagittariusArmReal::handle_gripper_traj_goal(
   const rclcpp_action::GoalUUID &uuid,
   std::shared_ptr<const FollowJointTrajectory::Goal> goal)
 {
   RCLCPP_INFO(this->get_logger(), "[handle_gripper_traj_goal] New goal request");
   (void)uuid;
   if(!goal) {
     return rclcpp_action::GoalResponse::REJECT;
   }
   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
 }
 
 rclcpp_action::CancelResponse SagittariusArmReal::handle_gripper_traj_cancel(
   const std::shared_ptr<GoalHandleFollowJointTraj> goal_handle)
 {
   RCLCPP_INFO(this->get_logger(), "[handle_gripper_traj_cancel] Cancel request");
   (void)goal_handle;
   execute_gripper_traj = false;
   return rclcpp_action::CancelResponse::ACCEPT;
 }
 
 void SagittariusArmReal::handle_gripper_traj_accepted(
   const std::shared_ptr<GoalHandleFollowJointTraj> goal_handle)
 {
   RCLCPP_INFO(this->get_logger(), "[handle_gripper_traj_accepted] executing...");
   std::thread(
     [this, goal_handle]()
     {
       this->arm_gripper_trajectory_action_callback(goal_handle);
     }
   ).detach();
 }
 
 void SagittariusArmReal::arm_gripper_trajectory_action_callback(
   std::shared_ptr<GoalHandleFollowJointTraj> goal_handle)
 {
   if (!goal_handle) return;
   auto goal   = goal_handle->get_goal();
   auto result = std::make_shared<FollowJointTrajectory::Result>();
 
   if (goal->trajectory.points.size() < 2)
   {
     result->error_code = result->INVALID_GOAL;
     goal_handle->abort(result);
     return;
   }
 
   auto traj_ptr = std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal->trajectory);
   this->arm_gripper_trajectory_msg_callback(traj_ptr);
 
   while(rclcpp::ok() && execute_gripper_traj)
   {
     if(goal_handle->is_canceling())
     {
       execute_gripper_traj = false;
       goal_handle->canceled(result);
       RCLCPP_INFO(this->get_logger(), "Gripper trajectory canceled by client");
       return;
     }
     std::this_thread::sleep_for(std::chrono::milliseconds(10));
   }
 
   result->error_code = result->SUCCESSFUL;
   goal_handle->succeed(result);
   RCLCPP_INFO(this->get_logger(), "Gripper trajectory done, succeed!");
 }
 
 //===============================
 //   定时器回调: 执行关节轨迹
 //===============================
 void SagittariusArmReal::arm_execute_joint_trajectory()
 {
   static uint8_t cntr = 0;
   if (!execute_joint_traj)
   {
     if (cntr != 0)
     {
       RCLCPP_INFO(this->get_logger(), "Joint trajectory stopped");
       cntr = 0;
     }
     return;
   }
 
   int traj_size = (int)jnt_tra_msg.points.size();
   double time_now = this->now().seconds() - joint_start_time;
   if (cntr >= traj_size)
   {
     RCLCPP_INFO(this->get_logger(), "Joint trajectory done.");
     execute_joint_traj = false;
     cntr = 0;
     return;
   }
 
   double time_from_start = jnt_tra_msg.points[cntr].time_from_start.sec
                          + jnt_tra_msg.points[cntr].time_from_start.nanosec / 1e9;
 
   if(time_now > time_from_start)
   {
      RCLCPP_INFO(this->get_logger(), "time_now > time_from_start");
     cntr++;
     if(cntr < traj_size)
     // TODO: 这里插入关节分段控制命令
     {
      // 获取当前轨迹点的目标时间
      double current_time_from_start = jnt_tra_msg.points[cntr].time_from_start.sec +
                                       jnt_tra_msg.points[cntr].time_from_start.nanosec / 1e9;

      // 获取当前轨迹点的 6 个关节角度
      double pos_array[6];
      for (int i = 0; i < 6; i++)
      {
        pos_array[i] = jnt_tra_msg.points[cntr].positions[i];
      }

      // 计算与上一轨迹点之间的时间差（单位：秒）
      double prev_time = 0.0;
      if(cntr > 0)
      {
        prev_time = jnt_tra_msg.points[cntr - 1].time_from_start.sec +
                  jnt_tra_msg.points[cntr - 1].time_from_start.nanosec / 1e9;
      }
      double delta = current_time_from_start - prev_time;
      if(delta < 0.001)
      {
        delta = 1;  // 保底，防止时间差太小导致运动无效果
      }

      // 调试打印：显示下发的关节角度和时间间隔
      RCLCPP_INFO(this->get_logger(), "Sending trajectory point %d: positions=[%f, %f, %f, %f, %f, %f], delta=%f",
                  cntr,
                  pos_array[0], pos_array[1], pos_array[2],
                  pos_array[3], pos_array[4], pos_array[5],
                  delta);

      // 调用硬件下发函数，让机械臂在 delta 秒内从上一点过渡到当前轨迹点
      this->arm_set_joint_positions(pos_array, delta);
      // rclcpp::sleep_for(std::chrono::milliseconds(100)); 

      // 更新全局的 time_from_start 变量为当前点的目标时间，便于下一段比较
      time_from_start = current_time_from_start;
     }

     else
     {
       RCLCPP_INFO(this->get_logger(), "Joint trajectory fully executed!");
       execute_joint_traj = false;
       cntr = 0;
     }
   }
   else
   {
     RCLCPP_INFO(this->get_logger(), "time_now < time_from_start");
   }
  }
 
 //===============================
 //   定时器回调: 执行爪子轨迹
 //===============================
 void SagittariusArmReal::arm_execute_gripper_trajectory()
 {
   static uint8_t cntr = 0;
   if (!execute_gripper_traj)
   {
     if (cntr != 0)
     {
       RCLCPP_INFO(this->get_logger(), "Gripper trajectory stopped");
       cntr = 0;
     }
     return;
   }
 
   int traj_size = (int)gripper_tra_msg.points.size();
   double time_now = this->now().seconds() - gripper_start_time;
   if (cntr >= traj_size)
   {
     RCLCPP_INFO(this->get_logger(), "Gripper trajectory done.");
     execute_gripper_traj = false;
     cntr = 0;
     return;
   }
 
   double time_from_start = gripper_tra_msg.points[cntr].time_from_start.sec
                          + gripper_tra_msg.points[cntr].time_from_start.nanosec / 1e9;
 
   if(time_now > time_from_start)
   {
     while(time_now > time_from_start && cntr < (traj_size - 1))
     {
       cntr++;
       time_from_start = gripper_tra_msg.points[cntr].time_from_start.sec
                       + gripper_tra_msg.points[cntr].time_from_start.nanosec / 1e9;
     }
     if(cntr < (traj_size - 1))
     {
       arm_set_gripper_linear_position(gripper_tra_msg.points[cntr].positions[0] * 2.0);
     }
     else
     {
       arm_set_gripper_linear_position(gripper_tra_msg.points[cntr].positions[0] * 2.0);
       RCLCPP_INFO(this->get_logger(), "Gripper trajectory fully executed!");
       execute_gripper_traj = false;
       cntr = 0;
     }
   }
 }
 
 //===============================
 //   控制函数
 //===============================
 short SagittariusArmReal::arm_calculate_gripper_degree_position(const float dist)
 {
   double half_dist = dist / 2.0;
   short result = static_cast<short>(-3462.0 * half_dist * 10.0);
   return result;
 }
 
 void SagittariusArmReal::arm_set_gripper_linear_position(const float dist)
 {
   short g_degree = arm_calculate_gripper_degree_position(dist);
   arm_set_single_joint_degree_position(g_degree);
 }
 
 void SagittariusArmReal::arm_set_single_joint_degree_position(short g_degree)
 {
   if(torque_status && pSDKarm)
   {
     pSDKarm->SendArmEndAction(0, g_degree);
   }
 }
 
 void SagittariusArmReal::arm_set_joint_positions(const double joint_positions[], double diff_time)
 {
   if(!pSDKarm) return;
   angle[0] = (float)joint_positions[0];
   angle[1] = (float)joint_positions[1];
   angle[2] = (float)joint_positions[2];
   angle[3] = (float)joint_positions[3];
   angle[4] = (float)joint_positions[4];
   angle[5] = (float)joint_positions[5];
 
   short difftime = (short)(diff_time * 1000.0);
   if(torque_status)
   {
    RCLCPP_INFO(this->get_logger(), "[SagittariusArmReal::arm_set_joint_positions] before pSDKarm->SendArmAllServerTime");
    pSDKarm->SendArmAllServerTime(difftime, angle[0], angle[1], angle[2], angle[3], angle[4], angle[5]);
   }
 }
 
 void SagittariusArmReal::arm_write_joint_commands(const sdk_sagittarius_arm::msg::ArmRadControl &msg)
 {
   if(msg.rad.empty()) return;
   std::vector<double> joint_positions(msg.rad.begin(), msg.rad.end());
   arm_set_joint_positions(joint_positions.data(), 0.0);
 }
 
 void SagittariusArmReal::arm_write_gripper_command(const std_msgs::msg::Float64 &msg)
 {
   arm_set_gripper_linear_position(msg.data * 2.0);
 }
 
 // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 // 周期发布当前关节状态：直接从 pSDKarm->current_joint_positions[] 获取
 // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 void SagittariusArmReal::publish_current_joint_states()
 {
   sensor_msgs::msg::JointState js;
   js.header.stamp = this->now();
 
   // 与 MoveIt SRDF 里对应
   js.name.resize(6);
   js.position.resize(6);
 
   js.name[0] = "joint1";
   js.name[1] = "joint2";
   js.name[2] = "joint3";
   js.name[3] = "joint4";
   js.name[4] = "joint5";
   js.name[5] = "joint6";
 
   // 若 pSDKarm 存在，就把其 current_joint_positions[] 写入
   if(pSDKarm)
   {
     for(int i=0; i<6; i++)
     {
       js.position[i] = pSDKarm->current_joint_positions[i];
     }
   }
   else
   {
     for(int i=0; i<6; i++)
     {
       js.position[i] = 0.0;
     }
   }
 
   // 发布
   publisher_joint_state_->publish(js);
 }
 
 } // namespace sdk_sagittarius_arm
 