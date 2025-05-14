#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    This launch file corresponds to the original run_sagittarius.launch (ROS1),
    migrated to ROS2 with Python-style launch.
    """

    # --- 1) Declare Launch Arguments ---
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
        default_value='/dev/sagittarius',
        description='Serial device used by the robot arm'
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

    joint_pub_gui_arg = DeclareLaunchArgument('joint_pub_gui', default_value='false')
    use_joint_pub_arg = DeclareLaunchArgument('use_joint_pub', default_value='true')
    just_rviz_control_arg = DeclareLaunchArgument('just_rviz_control', default_value='false')
    use_default_rviz_arg = DeclareLaunchArgument('use_default_rviz', default_value='true')
    use_world_frame_arg = DeclareLaunchArgument('use_world_frame', default_value='true')

    # --- 2) Include another launch file (description.launch.py) ---
    #    假设已将 sagittarius_descriptions/launch/description.launch 迁移至 ROS2 (Python/XML)
    sagittarius_desc_share = get_package_share_directory('sagittarius_descriptions')
    description_launch_file = PathJoinSubstitution([
        sagittarius_desc_share,
        'launch',
        'description.launch.py'  # 如果你用的是 XML，可改成 .launch.xml 并用相应的 LaunchDescriptionSource
    ])

    include_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_file),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'robot_model': LaunchConfiguration('robot_model'),
            'use_joint_pub': LaunchConfiguration('use_joint_pub'),
            'joint_pub_gui': LaunchConfiguration('joint_pub_gui'),
            'use_default_rviz': LaunchConfiguration('use_default_rviz'),
            'use_world_frame': LaunchConfiguration('use_world_frame'),
        }.items()
    )

    # --- 3) Launch the sdk_sagittarius_arm node ---
    #     注意: executable='sdk_sagittarius_arm' 要和你实际编译出的名字对应
    sdk_sagittarius_arm_share = get_package_share_directory('sdk_sagittarius_arm')
    sdk_sagittarius_arm_node = launch_ros.actions.Node(
        package='sdk_sagittarius_arm',
        executable='sdk_sagittarius_arm',
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
            'serialname': LaunchConfiguration('serialname'),
            'baudrate': '1000000',
            'timelimit': 5000,
            'servo_configs': os.path.join(sdk_sagittarius_arm_share, 'cfg'),
            'just_rviz_control': LaunchConfiguration('just_rviz_control'),
            'servo_control_trajectory': False
        }]
    )

    # --- 4) Construct & Return LaunchDescription ---
    return launch.LaunchDescription([
        # Declare all Launch Arguments
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
        joint_pub_gui_arg,
        use_joint_pub_arg,
        just_rviz_control_arg,
        use_default_rviz_arg,
        use_world_frame_arg,

        # Include the description.launch
        include_description,

        # Launch sdk_sagittarius_arm node
        sdk_sagittarius_arm_node
    ])
