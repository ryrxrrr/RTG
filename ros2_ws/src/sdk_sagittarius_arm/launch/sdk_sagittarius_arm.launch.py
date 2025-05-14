#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Corresponds to the original sdk_sagittarius_arm.launch (ROS1).
    Migrated to ROS2 (Python launch).
    """

    # --- 1) Declare Launch Arguments (原先 <arg> 标签) ---
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='sgr532',
        description='Namespace for the robot'
    )
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value=LaunchConfiguration('robot_name'),
        description='Robot model name, default same as robot_name'
    )
    serialname_arg = DeclareLaunchArgument(
        'serialname',
        #！！！！！！！！！！！！！这里串口名称根据电脑测试！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        default_value='/dev/ttyACM0',
        description='Serial device for the sagittarius arm'
    )

    arm_velocity_arg = DeclareLaunchArgument('arm_velocity', default_value='2048')
    arm_acceleration_arg = DeclareLaunchArgument('arm_acceleration', default_value='10')

    servo_torque1_arg = DeclareLaunchArgument('servo_torque1', default_value='300')
    servo_torque2_arg = DeclareLaunchArgument('servo_torque2', default_value='300')
    servo_torque3_arg = DeclareLaunchArgument('servo_torque3', default_value='300')
    servo_torque4_arg = DeclareLaunchArgument('servo_torque4', default_value='200')
    servo_torque5_arg = DeclareLaunchArgument('servo_torque5', default_value='300')
    servo_torque6_arg = DeclareLaunchArgument('servo_torque6', default_value='200')
    servo_torque7_arg = DeclareLaunchArgument('servo_torque7', default_value='100')

    exit_free_torque_arg = DeclareLaunchArgument('exit_free_torque', default_value='false')

    # --- 2) 启动 sdk_sagittarius_arm 节点 ---
    # respawn=True 等价于 ROS1 中的 respawn="true"
    # namespace=LaunchConfiguration('robot_name') 即 ns="$(arg robot_name)"
    # 下面 parameters=[] 对应原先 <param ...>；用 LaunchConfiguration(...) 关联到上面的 <arg>
    sdk_sagittarius_arm_share = get_package_share_directory('sdk_sagittarius_arm')
    sdk_sagittarius_arm_node = launch_ros.actions.Node(
        package='sdk_sagittarius_arm',
        executable='sdk_sagittarius_arm_node',
        name='sdk_sagittarius_arm',
       # namespace=LaunchConfiguration('robot_name'),改为全局变量了，可以设置回去
        output='screen',
        respawn=False,########这里由True改为False，这样不会不断重启，而是一次性报错方便检查
        prefix='gdb -ex run --args',  # 使用 prefix 指定调试前缀
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

            'range_min': 0.01,   # 与 <param name="range_min" type="double" value="0.01" /> 等价
            'range_max': 30.0,
            'serialname': LaunchConfiguration('serialname'),
            'baudrate': '1000000',
            'timelimit': 5000,
            # 使用 get_package_share_directory + 拼接替代 $(find sdk_sagittarius_arm)/cfg/
            'servo_configs': os.path.join(sdk_sagittarius_arm_share, 'cfg', ''), #末尾加入了''，这样就解决了最终得到的字符串是 ".../share/sdk_sagittarius_arm/cfg" 不含末尾 /

            'just_rviz_control': False,              # <param name="just_rviz_control" type="bool" value="false" />
            'servo_control_trajectory': False,       # <param name="servo_control_trajectory" type="bool" value="false" />
            'exit_free_torque': LaunchConfiguration('exit_free_torque'),
        }]
    )

    # --- 3) 组装并返回 LaunchDescription ---
    return launch.LaunchDescription([
        # 声明全部 Launch Arguments
        robot_name_arg,
        robot_model_arg,
        serialname_arg,
        arm_velocity_arg,
        arm_acceleration_arg,
        servo_torque1_arg,
        servo_torque2_arg,
        servo_torque3_arg,
        servo_torque4_arg,
        servo_torque5_arg,
        servo_torque6_arg,
        servo_torque7_arg,
        exit_free_torque_arg,

        # 启动节点
        sdk_sagittarius_arm_node
    ])