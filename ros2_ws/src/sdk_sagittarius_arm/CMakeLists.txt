cmake_minimum_required(VERSION 3.8)
project(sdk_sagittarius_arm)

###############################################################################
# 1) 查找所需依赖
###############################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)  # 如果启用 ROS2 组件模式，需要
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

###############################################################################
# 2) 生成消息 / 服务 / 动作接口
###############################################################################
# 列出您需要的 .msg / .srv
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ArmInfo.srv"
  "srv/ServoRtInfo.srv"
  "msg/ArmRadControl.msg"
  "msg/SingleRadControl.msg"
)

###############################################################################
# 3) 包含头文件目录
###############################################################################
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

###############################################################################
# 4) 生成共享库（例如 sdk_sagittarius_arm_lib）
###############################################################################
add_library(sdk_sagittarius_arm_lib SHARED
  src/sdk_sagittarius_arm_common.cpp
  # 如果还有其他公共源文件，可放在这里
)

ament_target_dependencies(sdk_sagittarius_arm_lib
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  trajectory_msgs
  moveit_msgs
  diagnostic_updater
  urdf
  rosidl_default_runtime
)

target_link_libraries(sdk_sagittarius_arm_lib
  yaml-cpp
)

###############################################################################
# 5) 生成可执行文件 sdk_sagittarius_arm_node
###############################################################################
add_executable(sdk_sagittarius_arm_node
  src/sdk_sagittarius_arm.cpp
  src/sdk_sagittarius_arm_common_serial.cpp
  src/sdk_sagittarius_arm_real.cpp
  # 如果还有其他源文件，一并加在此
)

ament_target_dependencies(sdk_sagittarius_arm_node
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  std_msgs
  trajectory_msgs
  moveit_msgs
  diagnostic_updater
  control_msgs
  urdf
  rosidl_default_runtime
  std_srvs
)

target_link_libraries(sdk_sagittarius_arm_node
  sdk_sagittarius_arm_lib
  yaml-cpp
  # rclcpp_components // 如果需要可取消注释
)

target_include_directories(sdk_sagittarius_arm_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

###############################################################################
# 6) 使用 rosidl_get_typesupport_target
###############################################################################
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(sdk_sagittarius_arm_node ${typesupport_target})

###############################################################################
# 7) 安装规则
###############################################################################
install(TARGETS
  sdk_sagittarius_arm_node
  sdk_sagittarius_arm_lib
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS sdk_sagittarius_arm_lib
  DESTINATION lib
)

install(
  FILES cfg/sagittarius_arm_model.yaml
  DESTINATION share/${PROJECT_NAME}/cfg
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes
  PATTERN ".svn" EXCLUDE
)

#下面这这两个不知道该不该有，我就先注释了
#install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf
#  PATTERN ".svn" EXCLUDE
#)   

#install(DIRECTORY plugins/ DESTINATION share/${PROJECT_NAME}/plugins
#  PATTERN ".svn" EXCLUDE
#)

###############################################################################
# 8) ament_package
###############################################################################
ament_package()