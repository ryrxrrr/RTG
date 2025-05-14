#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 获取 gazebo_ros 包的共享目录
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    # 假设 ROS2 下的 empty_world.launch.py 替换为 gazebo.launch.py 或者其他名称
    # 如果有对应的空世界启动文件，请根据实际文件名修改
    gazebo_launch_file = os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')

    # 静态TF发布器：在ROS2中使用tf2_ros包的static_transform_publisher
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint', '40'],
        output='screen'
    )

    # spawn_entity.py 用于在Gazebo中生成模型
    # 使用 -file 指定URDF路径，-entity 指定模型名称
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.join(get_package_share_directory('sagittarius_descriptions'), 'urdf', 'sgr532.urdf.xacro'),
            '-entity', 'sagittarius_descriptions'
        ],
        output='screen'
    )

    # 假设你只需要发布一次 calibrated 话题为true
    # 如果需要持续发布，请移除 --once 或者使用 -r 设置频率
    calibrated_pub = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/calibrated', 'std_msgs/Bool', 'data: true', '--once'],
        output='screen'
    )

    return LaunchDescription([
        # 引入Gazebo启动文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_file])
        ),
        static_tf_node,
        spawn_entity_node,
        calibrated_pub
    ])
