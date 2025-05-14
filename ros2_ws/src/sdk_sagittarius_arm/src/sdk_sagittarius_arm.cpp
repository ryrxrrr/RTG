/*
 * Software License Agreement (BSD License)
 * Copyright (c) ...
 * All rights reserved.
 * Author: ...
 *
 * Migrated to ROS2 by ...
 */

 #include <rclcpp/rclcpp.hpp>
 #include <sdk_sagittarius_arm/sdk_sagittarius_arm_real.hpp>
 // 这里假设 sdk_sagittarius_arm_real.hpp 里定义了
 // class SagittariusArmReal : public rclcpp::Node { ... };
 
 int main(int argc, char** argv)
 {
   // 1) 初始化 ROS2
   rclcpp::init(argc, argv);
 
   // 2) 创建一个 SagittariusArmReal 节点
   auto node = std::make_shared<sdk_sagittarius_arm::SagittariusArmReal>(
                   rclcpp::NodeOptions());
 
   // 如果需要 onInit() 来完成 shared_from_this() 等延后初始化，就调：
   node->onInit();
 
   // 显示一些启动信息
   RCLCPP_INFO(node->get_logger(), "sagittarius_arm_node is up and running (ROS2)");
 
   // 3) 进入 spin 循环
   rclcpp::spin(node);
 
   // 4) 退出
   rclcpp::shutdown();
   return 0;
 }
 