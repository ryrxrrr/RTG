cmake_minimum_required(VERSION 3.8)
project(my_bridge_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)

# 1) 生成自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraspPoint.msg"
)
ament_export_dependencies(rosidl_default_runtime)

# 2) 添加头文件路径（包括 Eigen3）
include_directories(
  include  # 如果有自定义头文件，位于 include/ 目录下
  ${EIGEN3_INCLUDE_DIRS}
)

# 3) 添加可执行文件
add_executable(bridge_node
  src/bridge_node.cpp
  src/bridge_node_main.cpp
)

ament_target_dependencies(bridge_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  Eigen3
  visualization_msgs
)

# 将生成的消息头文件目录加入到 include 路径中
target_include_directories(bridge_node PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# 使用 rosidl_get_typesupport_target() 链接消息类型支持目标（plain 签名）
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(bridge_node ${cpp_typesupport_target})

# 4) 安装
install(TARGETS
  bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
