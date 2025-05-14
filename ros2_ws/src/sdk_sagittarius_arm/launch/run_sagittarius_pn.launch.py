#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
对应原先的 run_sagittarius_pn.launch，但迁移到 ROS2 (Python Launch)
"""

import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    The setting of velocity and acceleration is the internal parameter of the servo. 
    If you are using MoveIt to plan, you also need to modify the joint parameters 
    of MoveIt planning at the same time (in sagittarius_moveit/config/joint_limits.yaml).

    e.g.
      If velocity is 3.14 rad/s (180 deg/s):
        - joint_limits.yaml: max_velocity=3.14 rad/s
        - Here in 'arm_velocity': 2048 step/s

      If acceleration is 1.57 rad/s^2 (90 deg/s^2):
        - joint_limits.yaml: max_acceleration=1.57 rad/s^2
        - Here in 'arm_acceleration': 1024 step/s^2 = 10.24 100step/s^2
    """

    # ---- Declare Launch Arguments (相当于原先 <arg> 标签) ----
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='sgr532',
        description='Name of the robot namespace'
    )
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value=LaunchConfiguration('robot_name'),  # 默认与 robot_name 相同
        description='Robot model name'
    )
    arm_velocity_arg = DeclareLaunchArgument(
        'arm_velocity',
        default_value='2048',
        description='Internal servo velocity parameter'
    )
    arm_acceleration_arg = DeclareLaunchArgument(
        'arm_acceleration',
        default_value='10',
        description='Internal servo acceleration parameter'
    )
    servo_torque1_arg = DeclareLaunchArgument('servo_torque1', default_value='300')
    servo_torque2_arg = DeclareLaunchArgument('servo_torque2', default_value='300')
    servo_torque3_arg = DeclareLaunchArgument('servo_torque3', default_value='300')
    servo_torque4_arg = DeclareLaunchArgument('servo_torque4', default_value='200')
    servo_torque5_arg = DeclareLaunchArgument('servo_torque5', default_value='300')
    servo_torque6_arg = DeclareLaunchArgument('servo_torque6', default_value='200')
    servo_torque7_arg = DeclareLaunchArgument('servo_torque7', default_value='100')

    joint_pub_gui_arg = DeclareLaunchArgument('joint_pub_gui', default_value='false')
    use_joint_pub_arg = DeclareLaunchArgument('use_joint_pub', default_value='false')
    just_rviz_control_arg = DeclareLaunchArgument('just_rviz_control', default_value='false')

    # ---- Include another launch file (sagittarius_descriptions/launch/description.launch.py) ----
    # 假设你也已经将 description.launch 迁移到 ROS2, 并命名为 description.launch.py
    sagittarius_desc_share = get_package_share_directory('sagittarius_descriptions')
    description_launch_file = PathJoinSubstitution([
        sagittarius_desc_share,
        'launch',
        'description.launch.py'  # 必须是 ROS2 可用的 Python/XML
    ])

    include_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_file),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'robot_model': LaunchConfiguration('robot_model'),
            'use_joint_pub': LaunchConfiguration('use_joint_pub'),
            'joint_pub_gui': LaunchConfiguration('joint_pub_gui')
        }.items()
    )

    # ---- 启动 sdk_sagittarius_arm 节点并传参 ----
    # respawn=True 相当于 ROS1 的 respawn="true"
    # namespace=LaunchConfiguration('robot_name') 相当于 ns="$(arg robot_name)"
    # 下面 parameters=[] 中的 Key-Value 对应原先 <param ...> 的配置
    sdk_sagittarius_arm_share = get_package_share_directory('sdk_sagittarius_arm')
    sdk_sagittarius_arm_node = launch_ros.actions.Node(
        package='sdk_sagittarius_arm',
        executable='sdk_sagittarius_arm',  # 你的可执行文件（或安装后的Python脚本）名字
        name='sdk_sagittarius_arm',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        respawn=True,
        parameters=[{
            'robot_name': LaunchConfiguration('robot_name'),
            'robot_model': LaunchConfiguration('robot_model'),
            'arm_velocity': LaunchConfiguration('arm_velocity'),
            'arm_acceleration': LaunchConfiguration('arm_acceleration'),
            'servo_torque1': LaunchConfiguration('servo_torque1'),
            'servo_torque2': LaunchConfiguration('servo_torque2'),
            'servo_torque3': LaunchConfiguration('servo_torque3'),
            'servo_torque4': LaunchConfiguration('servo_torque4'),
            'servo_torque5': LaunchConfiguration('servo_torque5'),
            'servo_torque6': LaunchConfiguration('servo_torque6'),
            'servo_torque7': LaunchConfiguration('servo_torque7'),

            'serialname': '/dev/sagittarius',
            'baudrate': '1000000',
            'timelimit': 5000,  # int
            # 使用 get_package_share_directory + join 来替代 $(find sdk_sagittarius_arm)/cfg/
            'servo_configs': os.path.join(sdk_sagittarius_arm_share, 'cfg'),
            'just_rviz_control': LaunchConfiguration('just_rviz_control'),
            'servo_control_trajectory': False
        }]
    )

    # ---- 组装并返回 LaunchDescription ----
    return launch.LaunchDescription([
        # 声明全部 Launch Argument
        robot_name_arg,
        robot_model_arg,
        arm_velocity_arg,
        arm_acceleration_arg,
        servo_torque1_arg,
        servo_torque2_arg,
        servo_torque3_arg,
        servo_torque4_arg,
        servo_torque5_arg,
        servo_torque6_arg,
        servo_torque7_arg,
        joint_pub_gui_arg,
        use_joint_pub_arg,
        just_rviz_control_arg,

        # 包含描述文件
        include_description,

        # 启动 sdk_sagittarius_arm 节点
        sdk_sagittarius_arm_node
    ])
