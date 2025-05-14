#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 声明一个可通过命令行传入的参数，如：ros2 launch sdk_sagittarius_arm forward_kinematics.launch.py robot_name:=sgr532
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='sgr532',
        description='Name of the robot namespace'
    )

    # 获取 sagittarius_moveit 包的 share 目录（需要在 ros2 下已经有这个包）
    # 假设 demo_true.launch.py 就放在 sagittarius_moveit/launch/ 下面
    sagittarius_moveit_share = get_package_share_directory('sagittarius_moveit')
    demo_true_launch_file = PathJoinSubstitution([
        sagittarius_moveit_share,
        'launch',
        'demo_true.launch.py'  # 这里是你迁移后的 ROS2 版本文件名
    ])

    # 将上面声明的参数取出
    robot_name = LaunchConfiguration('robot_name')

    # 包含另外一个 launch 文件
    demo_true_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo_true_launch_file),
        launch_arguments={'robot_name': robot_name}.items()
    )

    # 启动 Forward_kinematics 节点（Python脚本）
    forward_kinematics_node = launch_ros.actions.Node(
        package='sdk_sagittarius_arm',
        executable='Forward_kinematics.py',   # 注意，这里要在 CMakeLists 或 setup.py 里正确安装成可执行
        namespace=robot_name,
        name='Forward_kinematics',
        output='screen'
    )

    return launch.LaunchDescription([
        robot_name_arg,
        demo_true_include,
        forward_kinematics_node
    ])
