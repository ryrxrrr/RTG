cmake_minimum_required(VERSION 3.5)
project(sagittarius_descriptions)

# 找到必要的包
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# 安装文件夹
install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)



# 为 ROS2 提供支持
ament_package()
