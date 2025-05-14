cmake_minimum_required(VERSION 3.8)
project(object_interface)

#################################################
# 1) 找到依赖
#################################################
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # 生成消息
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(my_bridge_pkg REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# 若使用Qt5:
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)


#################################################
# 3) 编译你的可执行文件 (ui_node)
#################################################
# 包含头文件搜索路径
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 源文件
set(SRC_FILES
  src/main.cpp
  src/qt_node.cpp
)

# 生成可执行文件
add_executable(ui_node ${SRC_FILES})

# ament_target_dependencies 指定**外部**依赖
ament_target_dependencies(ui_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  my_bridge_pkg
  moveit_ros_planning_interface
)

# 如果使用 OpenCV、Qt5，则链接对应库
target_link_libraries(ui_node
  ${OpenCV_LIBS}
  Qt5::Core
  Qt5::Widgets
)


#################################################
# 5) 安装可执行文件
#################################################
install(TARGETS 
  ui_node
  DESTINATION lib/${PROJECT_NAME}
)

#################################################
# 6) 若有 launch 文件，则安装
#################################################
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

#################################################
# 7) 完成
#################################################
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

