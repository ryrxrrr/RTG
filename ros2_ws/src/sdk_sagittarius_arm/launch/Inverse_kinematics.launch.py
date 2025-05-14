#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 声明一个可通过命令行传入的参数，如：ros2 launch sdk_sagittarius_arm inverse_kinematics.launch.py robot_name:=sgr532
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='sgr532',
        description='Name of the robot namespace'
    )

    # 获取 sagittarius_moveit 包的 share 目录
    # 假设 demo_true.launch.py 就在 sagittarius_moveit/launch/ 中
    sagittarius_moveit_share = get_package_share_directory('sagittarius_moveit')
    demo_true_launch_file = PathJoinSubstitution([
        sagittarius_moveit_share,
        'launch',
        'demo_true.launch.py'  # 这里需是 ROS2 版的
    ])

    # 从命令行参数中获取 robot_name
    robot_name = LaunchConfiguration('robot_name')

    # include 另一个 launch (demo_true.launch.py)
    demo_true_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo_true_launch_file),
        launch_arguments={'robot_name': robot_name}.items()
    )

    # 启动 Inverse_kinematics 节点
    inverse_kinematics_node = launch_ros.actions.Node(
        package='sdk_sagittarius_arm',
        executable='Inverse_kinematics.py',  # 注意脚本需在 CMakeLists或setup.py里正确安装
        namespace=robot_name,
        name='Inverse_kinematics',
        output='screen'
    )

    return launch.LaunchDescription([
        robot_name_arg,
        demo_true_include,
        inverse_kinematics_node
    ])
